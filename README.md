# ESP Siri Remote Transceiver

ESP32-S3 firmware that connects to an Apple Siri Remote over BLE and exposes it to a host computer as a native USB HID device.

The current firmware targets the `freenove_esp32_s3_wroom` board and turns the remote into a small desktop/presentation/media controller with:

- Mouse pointer movement from single-finger touch gestures
- Smooth vertical and horizontal scrolling from two-finger gestures
- Left/right click mappings from remote buttons and touchpad clicks
- Keyboard shortcuts for presentation control
- Consumer control events for media playback and volume
- Automatic BLE reconnect handling after disconnects or idle drops
- Status indication on the onboard NeoPixel LED

## Hardware

- ESP32-S3 board with native USB device support
  Current PlatformIO target: `freenove_esp32_s3_wroom`
- Apple Siri Remote
- USB connection from the ESP32-S3 to the host computer

## How It Works

The ESP32-S3 scans for a specific Siri Remote BLE address, connects to it, enables its HID-style input path, and forwards the decoded events to the computer over USB.

USB output interfaces used by the firmware:

- Mouse
- Keyboard
- Consumer control

The firmware also logs connection and input state over serial at `115200` baud.

## Setup

1. Find your Siri Remote BLE MAC address.
2. Edit [platformio.ini](/home/moritz/Documents/PlatformIO/Projects/esp-siri-remote-transceiver/platformio.ini:11) and set:

```ini
-D SIRI_REMOTE_MAC=\"DC:2B:2A:EB:C3:DB\"
```

3. Adjust any optional tuning flags if needed.
4. Build and flash:

```bash
pio run -t upload
```

5. Open a serial monitor for logs:

```bash
pio device monitor -b 115200
```

If the remote is asleep, press a button on it to wake it so the ESP32 can see its advertisement and reconnect.

## Default Input Profiles

The firmware has three built-in profiles:

- `mouse`
- `powerpoint`
- `media`

Profile switching:

- Hold `Play/Pause` to enter profile selection mode
- While still holding `Play/Pause`, press `Volume Up` for the next profile
- While still holding `Play/Pause`, press `Volume Down` for the previous profile
- Release `Play/Pause` to exit profile selection mode
- If no profile was changed, releasing `Play/Pause` sends the normal play action

### LED profile colors

> LED requires a neopixel on pin 48

- `mouse`: green
- `powerpoint`: amber
- `media`: blue

General LED states:

- scanning: warm amber
- connecting: blue
- connected: profile color
- error: red

## Button Mapping

### Mouse Profile

- Touch surface, one finger: pointer movement
- Touch surface, quick tap: left click
- Touch surface, double tap and hold: click-and-drag
- Touch surface, two fingers: vertical and horizontal scroll
- Touch surface physical click: left click
- Two-finger touch click: right click
- `Menu`: left click
- `AirPlay`: right click
- `Volume Up`: volume up
- `Volume Down`: volume down
- `Play/Pause`: play or pause

### PowerPoint Profile

- Touch surface, one finger: pointer movement
- Touch surface, two fingers: scroll
- `Menu`, `Volume Down`, or two-finger touch click: left arrow
- `AirPlay`, `Volume Up`, or touchpad click: right arrow
- `Play/Pause`: `F5`
- `Siri`: `Esc`

### Media Profile

- `Volume Up`: volume up
- `Volume Down`: volume down
- `Play/Pause`: play or pause
- `AirPlay`: next track by default
- `Menu`: previous track by default

You can invert next/previous in the build flags with `SIRI_SWAP_MEDIA_NEXT_PREV=1`.

## Tuning Flags

These build flags are defined in [platformio.ini](/home/moritz/Documents/PlatformIO/Projects/esp-siri-remote-transceiver/platformio.ini:16):

- `SIRI_REMOTE_MAC`: BLE address of the target Siri Remote
- `SIRI_TOUCH_SENSITIVITY`: single-finger pointer sensitivity
- `SIRI_EDGE_FILTER`: ignores touches near the touchpad border when enabled
- `SIRI_SWAP_MEDIA_NEXT_PREV`: swaps `Menu` and `AirPlay` next/previous behavior in media mode
- `SIRI_PRECISION_TOUCHPAD`: currently must stay `0`

## Current Limitation

Windows Precision Touchpad mode is not implemented.
The firmware only decodes up to two concurrent contacts,
while true PTP support would require at least three-contact reporting,
so `SIRI_PRECISION_TOUCHPAD` intentionally fails the build if enabled.

This may be added later (ignoring the three finger touch capability)

## Development

Useful commands:

```bash
pio run
pio run -t upload
pio device monitor -b 115200
```