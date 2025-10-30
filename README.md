# Alpaca Master PLC

An Arduino Opta PLC safety controller for observatory automation that monitors conditions and automatically closes the observatory when unsafe conditions are detected.

## Purpose

This PLC acts as a master safety controller that:
- Monitors a safety sensor (weather conditions, etc.)
- Tracks dome position via open/closed sensors
- Checks telescope park status
- Automatically parks the telescope and closes the dome when conditions become unsafe
- Exposes an ASCOM Alpaca safety monitor API

## How It Works

The PLC continuously monitors three inputs:
1. **Safety Monitor Sensor** (A4): External safety signal (HIGH = safe, LOW = unsafe)
2. **Dome Closed Sensor** (A2): Detects when dome is fully closed
3. **Dome Opened Sensor** (A3): Detects when dome is fully open

### Safety Logic

The system triggers an observatory closure when:
- The safety monitor reports unsafe conditions, OR
- The dome is closed but the telescope is not parked (prevents collision)

When triggered, the system:
1. Aborts any telescope or dome movements
2. Parks the telescope (if dome is open)
3. Closes the dome (once telescope is parked)

### Network Architecture

The PLC communicates with:
- **Telescope** (192.168.1.11:3490): Direct TCP commands for parking/status
- **Dome** (192.168.1.14:80): ASCOM Alpaca API for shutter control

The PLC itself hosts an ASCOM Alpaca safety monitor at `http://192.168.1.10/api/v1/safetymonitor/0/`

## Hardware Setup

- **Arduino Opta PLC** with Ethernet connection
- Connect safety sensor to **A4** (HIGH = safe)
- Connect dome closed sensor to **A2** (HIGH when closed)
- Connect dome opened sensor to **A3** (HIGH when open)
- LEDs D0, D1, D2 indicate dome closed, dome opened, and safety status respectively

## Compilation & Upload

Requires [arduino-cli](https://arduino.github.io/arduino-cli/) installed with the Arduino Mbed Opta core.

```bash
# Compile the sketch
arduino-cli compile --fqbn arduino:mbed_opta:opta ./

# Upload to Opta PLC (replace port as needed)
arduino-cli upload -p /dev/cu.usbmodem1301 --fqbn arduino:mbed_opta:opta ./
```

## Configuration

Edit IP addresses in [alpaca-master-plc.ino](alpaca-master-plc.ino):
- `ip[]`: PLC's IP address (default: 192.168.1.10)
- `telescopeDirectIP`: Your telescope controller IP
- `domeAlpacaIP`: Your dome controller IP

## API Endpoints

The PLC exposes these ASCOM Alpaca endpoints:

- `GET /api/v1/safetymonitor/0/connected`: Check connection status
- `GET /api/v1/safetymonitor/0/issafe`: Get safety status (considers both sensor and error state)
- `GET /api/v1/safetymonitor/0/name`: Device name
- `GET /api/v1/safetymonitor/0/driverversion`: Driver version

## License

MIT License - see [LICENSE](LICENSE)