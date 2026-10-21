# Firmware

`duckbert_walk/` is the sketch running on the robot: an ESP32-C3 SuperMini driving the six SG90s
straight from its own pins, the gyroscope (MPU-6050) for the rocking, and the OLED face.

## Flash it

With [arduino-cli](https://arduino.github.io/arduino-cli/) (the Arduino IDE works the same way:
board *ESP32C3 Dev Module*, *USB CDC On Boot: Enabled*):

```sh
arduino-cli core install esp32:esp32                      # tested with 3.3.8
arduino-cli lib install "Adafruit GFX Library" "FluxGarage RoboEyes"
arduino-cli compile --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc firmware/duckbert_walk
arduino-cli upload  --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc -p <your port> firmware/duckbert_walk
```

Then calibrate: [`../docs/calibration.md`](../docs/calibration.md).

## Pins

| Joint / module | GPIO |
| -------------- | ---- |
| Left hip roll, hip pitch, knee | 3, 4, 6 |
| Right hip roll, hip pitch, knee | 0, 1, 2 |
| Gyroscope SDA / SCL | 8 / 9 |
| OLED SDA / SCL | 10 / 7 |

The servos take 5 V from the regulator, never from the board's 3.3 V pin, and every ground is common.
A joint can be moved to another pin over serial with `P <joint> <gpio>`.

## How he walks

He walks by rocking, in **STEP** mode (`step_control()`): the gyroscope feels the side-to-side
rock, and the hips push it along like a swing. When he's leaning onto his front foot, still moving
that way, and will still be there a fraction of a second later, the back foot is free, so it steps.
Then the other foot is in front, and the same rule fires on the other side.

Type `h` over serial for the full command list.
