# Calibrate and drive Duckbert from your browser

A small Python program talks to the robot (over USB, or over Wi-Fi) and serves two pages on your computer:

- **`/calibrate`**: the guided calibration, one pair of joints at a time, with a drawing of what to look at.
  Do this once, before the first walk. See [`../docs/calibration.md`](../docs/calibration.md).
- **`/`**: drive him. Hold ▲ and he walks; ⟲ ⟳ turn; let go and he stops. Arrow keys work too.
  The same page plays his moves: push-ups, a little dance, leaning left and right, a bow.

## Run it

```sh
pip install pyserial
python3 control/duckbert_web.py            # opens http://localhost:8080
```

- The robot is found by itself on USB. With no cable it looks for `duckbert.local` on Wi-Fi.
- For walking, keep him on **USB** if you can: over Wi-Fi the radio draws current from the same 5 V as the
  servos, and the rock gets smaller.
- `--lan` serves the page to your local network, so you can drive from your phone.
  Anyone on that network can drive the robot.
- Only one program can hold the serial port: close the Arduino Serial Monitor first.

## Wi-Fi

```sh
python3 control/wifi_join.py               # asks for the network name and the password
```

The password goes over USB straight to the board, which stores it; it is never written to a file or printed.
Use a 2.4 GHz WPA2 network. Stop `duckbert_web.py` first: one program per port.

## Also here

- The **● record** button saves a CSV of every joint and the gyroscope at 50 Hz into `control/logs/`.
- `duckbert_console.py` is the same thing in a terminal: a live table of the six joints, for calibrating with the
  keyboard.
