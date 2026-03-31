# TravelCar Host Control

Desktop host app for the TravelCar balancing robot.

## What it does

- Connects to the STM32 Bluetooth/serial link
- Sends keyboard and button control commands
- Shows an emergency stop button
- Reads and logs feedback from the robot
- Displays the ESP32-CAM MJPEG stream

## Default controls

- `W` / `S`: forward / backward
- `A` / `D`: left / right
- `Space`: stop drive
- `Arrow Left` / `Arrow Right`: pan camera
- `Arrow Up` / `Arrow Down`: tilt camera
- `C`: center camera
- `Esc`: emergency stop

## Command protocol

The app sends plain text lines ending with `\n`.

Default commands:

- `MOVE:FWD`
- `MOVE:BACK`
- `MOVE:LEFT`
- `MOVE:RIGHT`
- `MOVE:STOP`
- `CAM:LEFT`
- `CAM:RIGHT`
- `CAM:UP`
- `CAM:DOWN`
- `CAM:CENTER`
- `SYS:ESTOP`

If the STM32 side uses a different protocol, edit `travelcar_host_app.py` and update `COMMANDS`.

## Run

```bash
python travelcar_host\travelcar_host_app.py
```

## Notes

- The app uses Tkinter, which is built into standard Python on Windows.
- Video uses `requests` + `Pillow`, so OpenCV is not required.
- Bluetooth serial usually appears as a `COM` port after pairing on Windows.
