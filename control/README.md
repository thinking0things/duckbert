# Calibrate and drive Duckbert from your browser

A small Python program talks to the robot (over USB, or over Wi-Fi once he's on your network) and
serves two pages on your computer:

- **`/calibrate`**: the guided calibration, one pair of joints at a time, with a drawing of what to
  look at. Do this once, before the first walk. See [`../docs/calibration.md`](../docs/calibration.md).
- **`/`**: drive him. Hold ▲ and he walks; ⟲ ⟳ turn; let go and he stops. Arrow keys work too.

## Run it

```sh
pip install pyserial
python3 control/duckbert_web.py            # opens http://localhost:8080
```

- The robot is found by itself on USB. With no cable it looks for `duckbert.local` on Wi-Fi.
- `--lan` serves the page to your local network, so you can drive from your phone.
  Anyone on that network can drive the robot.
- Only one program can hold the serial port: close the Arduino Serial Monitor first.

To put the robot on Wi-Fi, send `N yourwifi|yourpassword` once over USB, from
`python3 control/duckbert_console.py --raw` or the Arduino Serial Monitor. The password is stored on the board
and never printed.

The **● record** button saves a CSV of every joint and the gyroscope at 50 Hz into `control/logs/`.

`duckbert_console.py` is the same thing in a terminal: a live table of the six joints, for
calibrating with the keyboard.
