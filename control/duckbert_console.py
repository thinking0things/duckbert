#!/usr/bin/env python3
"""Interactive console for the Duckbert walk firmware.

A calibration session is six joints, three questions each - is it at the pose the model calls
zero, does it turn the way the model turns, and does it reach both limits without the printed
parts fighting it - so this is a live table of all six with single keys that act on the one
under the cursor. Nothing here knows the gait; it is the firmware's own command set driven
from a keyboard, with the state polled back so you can watch a trim land.

    python3 duckbert_console.py                     # finds the board itself
    python3 duckbert_console.py --port /dev/cu.usbmodem101
    python3 duckbert_console.py --raw               # plain line monitor, no curses

Calibrate with the robot OFF the ground, and save with 'w' - trims live in flash, so an
unsaved session is lost on the next reset.
"""
import argparse
import collections
import curses
import datetime
import glob
import pathlib
import queue
import socket
import sys
import threading
import time

import serial

BAUD = 115200
LOGS = pathlib.Path(__file__).resolve().parent / 'logs'   # 'R 1' recordings land here as CSV
POLL_S = 0.25                      # how often to ask the board for its state
JOINTS = 6


def board_ports():
    return sorted(glob.glob('/dev/cu.usbmodem*') + glob.glob('/dev/ttyACM*'))


def find_port():
    """The C3 SuperMini enumerates as a USB CDC device; there is normally exactly one."""
    ports = board_ports()
    if not ports:
        sys.exit('no board found - pass --port')
    return ports[0]


class TcpPort:
    """The robot's Wi-Fi link, shaped like the pyserial object Link already drives."""

    def __init__(self, url):
        host, _, port = url[len('tcp://'):].partition(':')
        # IPv4 on purpose. Asked for "duckbert.local", create_connection also gets the IPv6 addresses
        # mDNS offers and can spend its whole timeout on one that never answers - which is what made
        # the bridge time out against a robot that was answering on 192.168.1.4 the whole time.
        ip = socket.gethostbyname(host)
        self.sock = socket.create_connection((ip, int(port or 2323)), timeout=4)
        self.sock.settimeout(0.1)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def read(self, n):
        try:
            data = self.sock.recv(n)
        except socket.timeout:
            return b''
        if not data:                        # the robot closed it: reset, flash, out of range
            raise OSError('tcp link closed')
        return data

    def write(self, data):
        self.sock.sendall(data)

    def close(self):
        self.sock.close()


DEFAULT_TCP = 'tcp://duckbert.local:2323'


class Link:
    """Serial link plus the last state the board reported."""

    def __init__(self, port, wifi_fallback=False):
        self.wifi_fallback = wifi_fallback
        self.ser, self.port, self._wanted = None, port or 'waiting for a board', port
        if port:
            try:
                self._open(port)             # no port, or not reachable yet: the reader keeps looking
            except (serial.SerialException, OSError, ValueError):
                self.ser = None
        self.log = []                        # every line the board sent, newest last
        self.joints = []                     # parsed 'J ...' rows
        self.mode, self.speed, self.sel = '?', 1.0, 0
        self.pwm_ok = True                   # firmware reports pwm=0 if ledcAttach failed
        self.gait, self.stand_ok, self.amp, self.sway = 0, False, 1.0, 1.0
        self.imu_ok = False
        self.kv = {}
        # 'M 1' streams "I <ms> ax ay az gx gy gz" at 50 Hz; kept apart from the log, counted so a
        # client can ask for only what it has not seen.
        self.imu_samples = collections.deque(maxlen=600)
        self.imu_count = 0
        self.last_rx = 0.0
        # 'R 1' streams one 'D ...' line a tick. It is written straight to a CSV as it arrives
        # rather than kept in memory: a walk is a few thousand lines and the point of recording is
        # to still have it after the robot falls over and the link drops.
        self.rec, self.rec_path, self.rec_rows, self.rec_t = None, None, 0, 0.0
        self.rec_label = ''          # names the next file, so a run can be found by what it was
        self.surface = 'mat'         # what it walks on: the rubber mat grips, the bare table does not
        self.feet = 'rocker'         # which feet are on: written into every recording's header,
                                     # so the comparison replays it in the right model
        self.rec_cols = []
        self.lock = threading.Lock()
        self._pending = []                   # J rows of the block being read
        self._alive = True
        threading.Thread(target=self._reader, daemon=True).start()

    def _open(self, port):
        self.ser = TcpPort(port) if port.startswith('tcp://') else serial.Serial(port, BAUD, timeout=0.1)
        self.port = port
        self.opened_at = time.time()

    def _drop(self, why):
        """Close a dead port and let the reader find the board again.

        The C3's USB CDC port vanishes on every reset - a flash, the RTS pulse, a replug - and
        often comes back under a different name (usbmodem101 one time, usbmodem2101 the next).
        This used to end the reader thread, which left the page showing a live-looking table
        that no longer talked to anything.
        """
        with self.lock:
            if self.ser is not None:
                self.log.append(f'!! {why} - reconnecting')
                del self.log[:-400]
            try:
                if self.ser is not None:
                    self.ser.close()
            except Exception:
                pass
            self.ser = None

    def _reconnect(self):
        time.sleep(1.0)                      # let the port finish re-enumerating before opening it
        ports = board_ports()
        if self._wanted and self._wanted.startswith('tcp://'):
            port = self._wanted                 # asked for Wi-Fi: keep trying Wi-Fi
        else:
            port = self._wanted if self._wanted in ports else (ports[0] if ports else None)
            if port is None and self.wifi_fallback:
                port = DEFAULT_TCP              # no cable: look for it on the network
        if port is None:
            return
        try:
            self._open(port)
            with self.lock:
                self.log.append(f'reconnected on {port}')
        except (serial.SerialException, OSError, ValueError):
            self.ser = None

    def _reader(self):
        buf = b''
        while self._alive:
            if self.ser is None:
                buf = b''
                self._reconnect()
                continue
            try:
                buf += self.ser.read(256)
            except (serial.SerialException, OSError, TypeError, AttributeError) as e:
                self._drop(f'serial link lost ({e.__class__.__name__})')
                continue
            # A Wi-Fi link can go dead without either end noticing - after the robot reboots, for
            # one, the bridge was left on a connection that accepted every '?' and answered none.
            # The state is asked for four times a second, so three silent seconds mean it is gone.
            if isinstance(self.ser, TcpPort) and time.time() - max(self.last_rx, self.opened_at) > 3.0:
                self._drop('no answer from the robot for 3 s')
                continue
            while b'\n' in buf:
                raw, buf = buf.split(b'\n', 1)
                try:
                    self._line(raw.decode('utf-8', 'replace').rstrip('\r'))
                except Exception as e:          # nothing a board says may end this thread
                    with self.lock:
                        self.log.append(f'!! bad line ignored ({e.__class__.__name__})')
                        del self.log[:-400]

    @property
    def connected(self):
        return time.time() - self.last_rx < 2.0

    def _line(self, line):
        with self.lock:
            self.last_rx = time.time()
            if line.startswith('STATE '):
                for tok in line[6:].split():
                    key, _, val = tok.partition('=')
                    self.kv[key] = val               # every field, so new ones need no parser
                    if key == 'mode':
                        self.mode = val
                    elif key == 'speed':
                        self.speed = float(val)
                    elif key == 'sel':
                        self.sel = int(val)
                    elif key == 'pwm':
                        self.pwm_ok = val == '1'
                    elif key == 'gait':
                        self.gait = int(val)
                    elif key == 'stand':
                        self.stand_ok = val == '1'
                    elif key == 'amp':
                        self.amp = float(val)
                    elif key == 'sway':
                        self.sway = float(val)
                    elif key == 'imu':
                        self.imu_ok = val == '1'
                self._pending = []
            elif line.startswith('DHDR '):
                self._rec_open(line)
            elif line.startswith('D ') and self.rec is not None:
                f = line.split()[1:]
                if len(f) == len(self.rec_cols):
                    self.rec.write(','.join(f) + '\n')
                    self.rec_rows += 1
                    self.rec_t = time.time()
            elif line.startswith('I '):
                f = line.split()
                if len(f) == 8:
                    try:
                        self.imu_samples.append([int(x) for x in f[1:]])
                        self.imu_count += 1
                    except ValueError:
                        pass
            elif line.startswith('J '):
                # A Wi-Fi link drops bytes under the 50 Hz recording stream, and a half a line
                # arrives looking like a J row with an empty number in it. This used to raise out
                # of the reader thread and kill it: the page stayed up, the board kept walking, and
                # nothing could stop it any more. A broken line is worth exactly one skipped row.
                try:
                    f = line.split()
                    row = dict(idx=int(f[1]), name=f[2])
                    for tok in f[3:]:
                        key, _, val = tok.partition('=')
                        row[key] = float(val) if '.' in val else int(val)
                except (ValueError, IndexError):
                    self._pending = []
                else:
                    self._pending.append(row)
            elif line == 'END':
                if len(self._pending) == JOINTS:
                    self.joints = self._pending
                self._pending = []
            elif line:
                self.log.append(line)
                del self.log[:-400]

    def _rec_open(self, hdr):
        """Start a CSV for the recording the board just announced, header kept as a comment."""
        self._rec_close()
        try:
            LOGS.mkdir(exist_ok=True)
            stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            self.rec_cols = next((t[5:].split(',') for t in hdr.split() if t.startswith('cols=')), [])
            tag = ('_' + self.rec_label) if self.rec_label else ''
            self.rec_path = LOGS / f'walk_{stamp}{tag}.csv'
            self.rec = open(self.rec_path, 'w')
            self.rec.write('# ' + hdr + f' feet={self.feet} surface={self.surface}\n')
            self.rec.write(','.join(self.rec_cols) + '\n')
            self.rec_rows, self.rec_t = 0, time.time()
            self.log.append(f'recording to {self.rec_path.name}')
        except OSError as e:
            self.rec, self.rec_path = None, None
            self.log.append(f'!! cannot record ({e})')

    def _rec_close(self):
        if self.rec is not None:
            self.rec.close()
            self.log.append(f'recorded {self.rec_rows} lines to {self.rec_path.name}')
            del self.log[:-400]
        self.rec = None

    def note(self, text, cm=None, file=None):
        """Write down what a run did on the floor. The IMU cannot measure distance - integrating
        its accelerometer twice drifts metres in seconds - so the only honest source is a ruler
        and the person holding it. Kept as its own file so no recording is ever rewritten."""
        with self.lock:
            name = file or (self.rec_path.name if self.rec_path else 'none')
        line = f'{name},{"" if cm is None else cm},{text.replace(chr(10), " ")[:200]}\n'
        notes = LOGS / 'notes.csv'
        try:
            LOGS.mkdir(exist_ok=True)
            if not notes.exists():
                notes.write_text('file,distance_cm,note\n')
            with notes.open('a') as f:
                f.write(line)
        except OSError as e:
            with self.lock:
                self.log.append(f'!! cannot write the note ({e})')
            return None
        with self.lock:
            self.log.append(f'note on {name}: {cm if cm is not None else ""} {text}'.strip())
            del self.log[:-400]
        return name

    def rec_state(self):
        with self.lock:
            if self.rec is not None and time.time() - self.rec_t > 3.0:
                self._rec_close()            # the stream stopped, or the link died mid-walk
            return dict(active=self.rec is not None, rows=self.rec_rows, label=self.rec_label,
                        file=self.rec_path.name if self.rec_path else None)

    def send(self, cmd):
        if cmd != '?':                       # the state poll is four a second, keep it out of the log
            with self.lock:
                self.log.append('> ' + cmd)
                del self.log[:-400]
        ser = self.ser
        if ser is None:
            return                           # mid-reconnect; the poller will ask again
        try:
            ser.write((cmd + '\n').encode())
        except (serial.SerialException, OSError) as e:
            self._drop(f'write failed ({e.__class__.__name__})')

    def close(self):
        self._alive = False
        with self.lock:
            self._rec_close()
        time.sleep(.15)
        if self.ser is not None:
            self.ser.close()


KEYS = [
    ('up/down', 'select joint'),
    ('left/right', 'trim -/+ 1 deg'),
    (', .', 'trim -/+ 0.2 deg'),
    ('[ ]', 'jog -/+ 5 deg'),
    ('0', 'jog to zero'),
    ('i', 'invert sign'),
    ('k', 'sweep both limits'),
    ('n', 'all to neutral'),
    ('g', 'WALK'),
    ('s', 'stop, hold'),
    ('l', 'limp'),
    ('- +', 'gait speed'),
    ('w', 'save to flash'),
    ('r', 'reload saved'),
    ('q', 'quit'),
]


def draw(scr, link, sel, note):
    scr.erase()
    h, w = scr.getmaxyx()
    with link.lock:
        joints, mode, speed, log = link.joints, link.mode, link.speed, link.log[-(h - 16):]

    colour = {'WALK': 2, 'LIMP': 3}.get(mode, 1)
    scr.addnstr(0, 0, ' Duckbert console ', w - 1, curses.A_REVERSE | curses.A_BOLD)
    scr.addnstr(0, 19, f'{link.port} @ {BAUD}', max(0, w - 20), curses.A_DIM)
    scr.addnstr(1, 1, f'mode {mode:<5}', w - 2, curses.color_pair(colour) | curses.A_BOLD)
    scr.addnstr(1, 14, f'gait speed {speed:.2f}', w - 15)

    scr.addnstr(3, 2, 'joint          pin   sign     trim        now      travel', w - 3,
                curses.A_UNDERLINE)
    for i in range(JOINTS):
        y = 4 + i
        if y >= h - 2:
            break
        if i < len(joints):
            j = joints[i]
            row = (f"{j['name']:<13} GPIO{j['pin']:<2} {j['sign']:+d}    "
                   f"{j['trim']:+7.1f}  {j['now']:+7.1f}   "
                   f"[{j['min']:+.0f} .. {j['max']:+.0f}]")
        else:
            row = '...'
        attr = curses.A_REVERSE if i == sel else curses.A_NORMAL
        scr.addnstr(y, 2, f'{">" if i == sel else " "} {row}', w - 3, attr)

    y = 11
    scr.addnstr(y, 2, note or '', w - 3, curses.color_pair(4) | curses.A_BOLD)
    y += 2
    col = 2
    for key, what in KEYS:
        text = f'{key} {what}'
        if col + len(text) + 3 > w - 2:
            y, col = y + 1, 2
        if y < h - 1:
            scr.addnstr(y, col, key, w - col - 1, curses.A_BOLD)
            scr.addnstr(y, col + len(key) + 1, what, max(0, w - col - len(key) - 2), curses.A_DIM)
        col += len(text) + 3
    y += 2

    if y < h - 1:
        scr.addnstr(y, 2, 'serial', w - 3, curses.A_DIM | curses.A_UNDERLINE)
        for k, line in enumerate(log[-(h - y - 2):]):
            if y + 1 + k < h - 1:
                scr.addnstr(y + 1 + k, 2, line, w - 3,
                            curses.A_DIM if line.startswith('>') else curses.A_NORMAL)
    scr.refresh()


def trim_of(link, i):
    with link.lock:
        return link.joints[i]['trim'] if i < len(link.joints) else 0.0


def now_of(link, i):
    with link.lock:
        return link.joints[i]['now'] if i < len(link.joints) else 0.0


def tui(scr, link):
    curses.curs_set(0)
    scr.nodelay(True)
    scr.keypad(True)
    curses.use_default_colors()
    for n, fg in ((1, curses.COLOR_GREEN), (2, curses.COLOR_CYAN),
                  (3, curses.COLOR_RED), (4, curses.COLOR_YELLOW)):
        curses.init_pair(n, fg, -1)

    sel, note, last_poll = 0, '', 0.0
    link.send('?')
    while True:
        if time.time() - last_poll > POLL_S:
            link.send('?')
            last_poll = time.time()
        draw(scr, link, sel, note)

        try:
            ch = scr.getch()
        except curses.error:
            ch = -1
        if ch == -1:
            time.sleep(.03)
            continue
        note = ''

        if ch in (ord('q'), 27):
            return
        elif ch == curses.KEY_UP:
            sel = (sel - 1) % JOINTS
        elif ch == curses.KEY_DOWN:
            sel = (sel + 1) % JOINTS
        elif ch in (curses.KEY_LEFT, curses.KEY_RIGHT, ord(','), ord('.')):
            step = {curses.KEY_LEFT: -1.0, curses.KEY_RIGHT: 1.0,
                    ord(','): -0.2, ord('.'): 0.2}[ch]
            link.send(f't {sel} {trim_of(link, sel) + step:.2f}')
        elif ch in (ord('['), ord(']')):
            link.send(f'a {sel} {now_of(link, sel) + (-5 if ch == ord("[") else 5):.1f}')
        elif ch == ord('0'):
            link.send(f'a {sel} 0')
        elif ch == ord('i'):
            link.send(f'j {sel}')
            link.send('i')
            note = 'sign inverted - the joint eased to zero first'
        elif ch == ord('k'):
            link.send(f'j {sel}')
            link.send('k')
            note = 'sweeping both limits, ~4 s - watch the direction'
        elif ch == ord('g'):
            note = 'WALKING - stand it up first, "s" stops it'
            link.send('g')
        elif ch in (ord('n'), ord('s'), ord('l'), ord('w'), ord('r'), ord('-'), ord('+')):
            link.send(chr(ch))
            if ch == ord('w'):
                note = 'calibration written to flash'
            elif ch == ord('l'):
                note = 'LIMP - the robot will collapse, hold it'
        elif ch == ord('h'):
            link.send('h')


def raw(link):
    """No curses: echo everything, send whatever you type. 'quit' to leave."""
    print(f'connected to {link.port} - type commands, "h" for help, "quit" to exit')
    seen = 0
    q = queue.Queue()
    threading.Thread(target=lambda: [q.put(sys.stdin.readline())
                                     for _ in iter(int, 1)], daemon=True).start()
    while True:
        with link.lock:
            fresh, seen = link.log[seen:], len(link.log)
        for line in fresh:
            print(line)
        try:
            cmd = q.get(timeout=.1).strip()
        except queue.Empty:
            continue
        if cmd in ('quit', 'exit'):
            return
        if cmd:
            link.send(cmd)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default=None)
    ap.add_argument('--raw', action='store_true', help='plain line monitor instead of the TUI')
    a = ap.parse_args()

    link = Link(a.port or find_port())
    time.sleep(2.0)                        # the C3 reboots when the port opens
    try:
        raw(link) if a.raw else curses.wrapper(tui, link)
    except KeyboardInterrupt:
        pass
    finally:
        link.close()


if __name__ == '__main__':
    main()
