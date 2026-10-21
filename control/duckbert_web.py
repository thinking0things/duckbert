#!/usr/bin/env python3
"""Browser control panel for the Duckbert walk firmware.

The board speaks a line protocol over USB serial, and a web page cannot open a serial port
from a sandbox or from https, so this owns the port and puts a small HTTP API in front of it:
the page polls /api/state and posts commands to /api/cmd. Everything stays on localhost, which
is also the only reason the browser is willing to talk to it at all.

    python3 duckbert_web.py                      # opens http://localhost:8080 for you
    python3 duckbert_web.py --port /dev/cu.usbmodem2101 --http 9000 --no-open

Only one program can hold the serial port. Close the Arduino IDE's Serial Monitor and quit
duckbert_console.py before starting this.
"""
import argparse
import json
import re
import pathlib
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from duckbert_console import Link, board_ports, JOINTS, DEFAULT_TCP

HERE = pathlib.Path(__file__).resolve().parent
PAGE = HERE / 'drive.html'        # hold an arrow to walk, let go to stop; the moves
CALIB = HERE / 'calibrate.html'   # guided calibration, one joint pair at a time, at /calibrate

NUM = r'-?\d+(?:\.\d+)?'
ALLOWED = re.compile(rf'''(?x)
      [gsnlkiwrphdeym?+\-\[\]]      # the no-argument commands ('m': one step, since 8 Oct)
    | q(?:\ \d\d?)?                  # crouch, optional depth in degrees
    | T\ -?[01]                      # head tilt
    | M(?:\ [01])?                   # IMU status / raw stream on-off
    | N(?:\ p\ \d\d?(?:\.\d)?|\ s)?   # Wi-Fi status, transmit power, or scan - credentials go over USB, never through here
    | Y(?:\ 1)?                      # face status / display test
    | I\ [01]                        # the gait's own gyro feedback
    | F\ \d\d?\d?(?:\.\d)?             # how much of that feedback, percent
    | J(?:\ -?\d*\.?\d+\ -?\d*\.?\d+)?   # the walks' pitch feedback on gains of its own; bare J = searched gains
    | C\ \d\d?(?:\.\d)?\ -?\d\d?(?:\.\d)?\ \d(?:\.\d\d?)?\ -?\d\d?(?:\.\d)?   # sequence walks: crouch, rock, lift sharpness, hip opening
    | X\ -?\d\d?(?:\.\d)?            # sideways trunk trim
    | R\ [01]                        # start or stop the walk recording
    | L\ -?\d\d?(?:\.\d)?            # trunk lean while walking
    | W\ -?\d\d?\d?(?:\.\d)?         # steering: hip swing asymmetry, percent (to 100 since 8 Oct)
    | o(?:(?:\ -?\d*\.?\d+){{11,12}})?    # one-step parameters, eleven or twelve numbers (bare o reports them)
    | v(?:\ \d*\.?\d+\ \d*\.?\d+\ [012](?:\ \d*\.?\d+(?:\ -?\d*\.?\d+(?:\ \d*\.?\d+(?:\ \d*\.?\d+(?:\ [012](?:\ -?\d*\.?\d+(?:\ [012](?:\ -?\d*\.?\d+)?)?)?)?)?)?)?)?)?   # steps on the rock: threshold, rock size, mode [, lead s [, pitch damping [, splay [, crouch [, style 0/1/2 [, pitch ref [, 0 steps / 1 in place / 2 slide [, legs pump deg]]]]]]]]
    | f\ \d{{1,3}}                       # slide: share of stride and lift to use, 0..100 (the drive page)
    | b(?:\ \d*\.?\d+){{1,6}}               # slide lift recipe: lead ms, toe-up deg, leg push deg, pump cap deg, lift filter ms, leg level 0/1
    | Z(?:\ \d\d?\d?)?                # forward share of the stride; Z 0 + W turns on the spot
    | \*\ [0-4]                      # the video moves: 1 squats, 2 dance, 3 lean, 4 bow; 0 stops
    | E\ \d\d\d?(?:\.\d)?            # knee swing, percent: toe clearance
    | Q\ \d\d?(?:\.\d)?(?:\ \d(?:\.\d\d?)?)?(?:\ \d(?:\.\d\d?)?)?   # rock the weight side to side
    | H\ [01](?:\ -?\d+(?:\.\d+)?\ \d+(?:\.\d+)?)?   # heading hold [+ its two gains; a negative gain flips it]
    | D\ -?\d\d?\d?(?:\.\d)?         # turn by that many degrees, + to its right
    | U\ -?\d*\.?\d+\ -?\d*\.?\d+\ -?\d*\.?\d+\ -?\d*\.?\d+\ -?\d*\.?\d+\ -?\d*\.?\d+\ -?\d*\.?\d+\ -?\d*\.?\d+\ -?\d*\.?\d+   # IMU mounting, nine numbers
    | B\ [012](?:\ -?\d*\.?\d+\ -?\d*\.?\d+\ -?\d*\.?\d+\ -?\d*\.?\d+(?:\ -?\d*\.?\d+)?)?   # balance mode [+ four gains [+ the lean it answers to, in seconds]]
    | K\ [01](?:\ -?\d*\.?\d+\ -?\d*\.?\d+\ -?\d*\.?\d+\ -?\d*\.?\d+)?    # push reaction [+ four gains]
    | j\ [0-5]                       # select joint
    | c\ [0-2]                       # calibration pose
    | G\ \d\d?                       # choose a gait (G10 and up since 8 Oct)
    | A\ (?:0?\.\d\d?|1(?:\.0+)?)    # step size
    | O\ (?:[01]?\.\d\d?|[12](?:\.\d\d?)?)   # hip sway, now up to 2: the rolls have 20 deg of travel
    | V\ (?:0?\.\d\d?|1(?:\.\d\d?)?) # gait speed
    | S(?:\ 0?\.\d\d?|\ 1)?          # stand up from face down, optionally slowed (dry run)
    | [at]\ [0-5]\ {NUM}             # jog to an angle / set a trim
    | z\ [0-5]\ [+-]?1               # set a sign
    | P\ [0-5]\ (?:\d|1\d|2[01])      # move a joint to another GPIO
    | x\ (?:\d|10|2[01])             # pad test on a bare gpio
    | u\ (?:\d|10|2[01])\ \d\d\d\d?  # raw servo pulse on a bare gpio
''')


class Handler(BaseHTTPRequestHandler):
    link = None

    def _send(self, code, body, ctype):
        body = body.encode() if isinstance(body, str) else body
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split('?')[0]
        if path == '/':
            return self._send(200, PAGE.read_bytes(), 'text/html; charset=utf-8')
        if path == '/calibrate':
            return self._send(200, CALIB.read_bytes(), 'text/html; charset=utf-8')
        # The 3D view: three.js vendored next to this file so it works offline, and the robot's
        # compiled geometry written by export_scene.py. Only these exact files are served.
        static = {'/vendor/three.module.js': ('vendor/three.module.js', 'text/javascript'),
                  '/vendor/OrbitControls.js': ('vendor/OrbitControls.js', 'text/javascript'),
                  '/scene/scene.json': ('scene/scene.json', 'application/json'),
                  '/scene/meshes.bin': ('scene/meshes.bin', 'application/octet-stream'),
                  '/gaits.json': ('gaits.json', 'application/json')}
        if path in static:
            rel, ctype = static[path]
            f = HERE / rel
            if not f.exists():
                return self._send(404, f'{rel} missing - run export_scene.py', 'text/plain')
            return self._send(200, f.read_bytes(), ctype)
        if path == '/api/imu':
            since = 0
            if '?' in self.path:
                for tok in self.path.split('?', 1)[1].split('&'):
                    if tok.startswith('since='):
                        since = int(tok[6:] or 0)
            link = Handler.link
            with link.lock:
                count, buf = link.imu_count, list(link.imu_samples)
                fresh = buf[-min(len(buf), max(0, count - since)):] if count > since else []
                body = dict(count=count, samples=fresh, imu_ok=link.imu_ok, connected=link.connected)
            return self._send(200, json.dumps(body), 'application/json')
        if path == '/api/logs':
            files = sorted(p.name for p in (HERE / 'logs').glob('*.csv')) if (HERE / 'logs').is_dir() else []
            return self._send(200, json.dumps(files), 'application/json')
        if path == '/api/state':
            since = 0
            if '?' in self.path:
                for tok in self.path.split('?', 1)[1].split('&'):
                    if tok.startswith('since='):
                        since = int(tok[6:] or 0)
            link = Handler.link
            with link.lock:
                # The board is asked for its state on a timer by the poller thread; this just
                # serves whatever came back last, so a slow client never stalls the link.
                state = dict(mode=link.mode, speed=link.speed, sel=link.sel,
                             gait=link.gait, stand_ok=link.stand_ok, amp=link.amp, sway=link.sway, kv=dict(link.kv),
                             pwm=link.pwm_ok, port=link.port, connected=link.connected,
                             joints=link.joints, log=link.log[since:], seen=len(link.log))
            state['rec'] = link.rec_state()
            state['feet'] = getattr(link, 'feet', 'rocker')
            state['surface'] = getattr(link, 'surface', 'mat')
            return self._send(200, json.dumps(state), 'application/json')
        self._send(404, 'not found', 'text/plain')

    def do_POST(self):
        if self.path == '/api/note':
            n = int(self.headers.get('Content-Length', 0) or 0)
            body = json.loads(self.rfile.read(n).decode('utf-8', 'replace') or '{}')
            cm = body.get('cm')
            # a note may name the recording it belongs to; only a file that is really in logs/
            file = str(body.get('file') or '')
            if file and (not file.endswith('.csv') or '/' in file or not (HERE / 'logs' / file).exists()):
                return self._send(404, json.dumps({'error': 'no such recording'}), 'application/json')
            name = Handler.link.note(str(body.get('text', ''))[:200],
                                     None if cm in (None, '') else float(cm), file or None)
            return self._send(200, json.dumps({'file': name}), 'application/json')
        if self.path == '/api/surface':
            n = int(self.headers.get('Content-Length', 0) or 0)
            surface = self.rfile.read(n).decode('utf-8', 'replace').strip()
            if surface not in ('mat', 'table'):
                return self._send(400, 'unknown surface', 'text/plain')
            Handler.link.surface = surface
            return self._send(200, 'ok', 'text/plain')
        if self.path == '/api/feet':
            n = int(self.headers.get('Content-Length', 0) or 0)
            feet = self.rfile.read(n).decode('utf-8', 'replace').strip()
            if feet not in ('rocker', 'rocker_in', 'flat_in'):
                return self._send(400, 'unknown feet', 'text/plain')
            Handler.link.feet = feet
            return self._send(200, 'ok', 'text/plain')
        if self.path == '/api/reclabel':
            # Names the next recording after the run it belongs to. Kept to the characters that are
            # safe in a file name; the plan in the page sends one before it starts a run.
            n = int(self.headers.get('Content-Length', 0) or 0)
            label = re.sub(r'[^a-z0-9_-]', '', self.rfile.read(n).decode('utf-8', 'replace').lower())[:24]
            Handler.link.rec_label = label
            return self._send(200, 'ok', 'text/plain')
        if self.path != '/api/cmd':
            return self._send(404, 'not found', 'text/plain')
        n = int(self.headers.get('Content-Length', 0))
        cmd = self.rfile.read(n).decode('utf-8', 'replace').strip()
        # The page is the only client, but it is still a client: a whole command has to match
        # the firmware's grammar, not merely start with a letter the firmware knows.
        if not ALLOWED.fullmatch(cmd):
            return self._send(400, 'rejected', 'text/plain')
        Handler.link.send(cmd)
        self._send(200, 'ok', 'text/plain')

    def log_message(self, *a):
        pass                                    # the page has its own log; keep the console quiet


def poller(link, period=0.25):
    """Ask the board for its state on a fixed cadence, independent of who is watching."""
    while True:
        try:
            link.send('?')
        except Exception:                    # never let the one thread that polls the board die
            pass
        time.sleep(period)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default=None, help='serial port (auto-detected by default)')
    ap.add_argument('--http', type=int, default=8080)
    ap.add_argument('--wifi', nargs='?', const=DEFAULT_TCP, default=None, metavar='tcp://host:port',
                    help=f'talk to the robot over Wi-Fi (default {DEFAULT_TCP}) instead of USB')
    ap.add_argument('--usb', action='store_true', help='USB only: never fall back to Wi-Fi when the cable is out')
    ap.add_argument('--lan', action='store_true',
                    help='serve the page to the whole local network, e.g. a phone - anyone on it can drive the robot')
    ap.add_argument('--no-open', action='store_true')
    a = ap.parse_args()

    # Start without a board if none is plugged in: the link finds it when it appears, and the page
    # says DISCONNECTED until then instead of the server refusing to start at all.
    ports = board_ports()
    if a.wifi:
        link = Link(a.wifi)
    else:
        # USB when a cable is in; with no cable, the robot is looked for on Wi-Fi as well.
        link = Link(a.port or (ports[0] if ports else None), wifi_fallback=not a.usb)
    Handler.link = link
    time.sleep(1.5)
    threading.Thread(target=poller, args=(link,), daemon=True).start()

    url = f'http://localhost:{a.http}/'
    server = ThreadingHTTPServer(('0.0.0.0' if a.lan else '127.0.0.1', a.http), Handler)
    print(f'robot {link.port}   ->   {url}')
    if a.lan:
        import socket as _s
        try:
            probe = _s.socket(_s.AF_INET, _s.SOCK_DGRAM); probe.connect(('192.168.0.1', 9)); lan_ip = probe.getsockname()[0]; probe.close()
        except OSError:
            lan_ip = '<this Mac>'
        print(f'on the local network: http://{lan_ip}:{a.http}/   (anyone on this network can drive the robot)')
    print('one holder per port: close the Arduino Serial Monitor and duckbert_console.py')
    if not a.no_open:
        threading.Timer(0.6, webbrowser.open, (url,)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        link.close()


if __name__ == '__main__':
    main()
