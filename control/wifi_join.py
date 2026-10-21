"""Join Duckbert to a Wi-Fi network over USB, without the password ever touching a file.

The firmware stores ONE network with 'N ssid|password' (USB only - the web bridge refuses it); a new one replaces
the old. This asks for both
on the terminal, the password hidden, and sends that one line. Stop the web bridge first: one holder per port.

    python3 wifi_join.py [--port /dev/cu.usbmodem1101]
"""
import argparse
import getpass
import glob
import time

import serial


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default=None)
    a = ap.parse_args()
    port = a.port or (sorted(glob.glob('/dev/cu.usbmodem*')) or [None])[0]
    if not port:
        raise SystemExit('no board on USB')
    ssid = input('Wi-Fi name (2.4 GHz, WPA2 personal): ').strip()
    pw = getpass.getpass('password (hidden): ')
    if not ssid or '|' in ssid or '\n' in ssid + pw:
        raise SystemExit('bad name or password')
    with serial.Serial(port, 115200, timeout=0.2) as s:
        time.sleep(0.5)
        s.reset_input_buffer()
        s.write(f'N {ssid}|{pw}\n'.encode())
        pw = None
        t0 = time.time()
        while time.time() - t0 < 20:
            line = s.readline().decode(errors='replace').strip()
            if not line:
                continue
            if line.startswith('wifi'):
                print(line)
            if line.startswith('wifi connected to'):
                break
        s.write(b'N\n')
        time.sleep(1.0)
        for line in s.read(4000).decode(errors='replace').splitlines():
            if line.startswith('wifi'):
                print(line)


if __name__ == '__main__':
    main()
