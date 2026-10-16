# Alpaca Master PLC

Custom Arduino Opta PLC safety controller for observatory automation that monitors conditions and automatically closes the observatory when unsafe conditions are detected.

## Purpose

This PLC acts as a master safety controller that:
- Monitors a 24V safety monitor signal
- Tracks dome position via open/closed sensors (rolling roof)
- Checks telescope park status (10Micron mounts)
- Automatically parks the telescope and closes the dome when conditions become unsafe
- Exposes an ASCOM Alpaca safety monitor API

## How It Works

The PLC continuously monitors three inputs:
1. **Safety Monitor Sensor** (A4): External safety signal (HIGH = safe, LOW = unsafe). The input is sampled once a second. The PLC uses the median (majority vote) of the last 60 samples, so short spurious changes are ignored. A real change takes effect after about 30 seconds.
2. **Dome Closed Sensor** (A2): Detects when dome is fully closed
3. **Dome Opened Sensor** (A3): Detects when dome is fully open
4. **Telescope Park Status**: Queried via direct TCP commands to the 10Micron telescope controller

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

The PLC itself hosts an ASCOM Alpaca safety monitor at `http://192.168.1.10/api/v1/safetymonitor/0/`.

In addition, it provides a tiny web UI and simple control APIs (documented below) to pause/resume the automatic actions and to reset errors.

## Hardware Setup

- **Arduino Opta PLC** with Ethernet connection
- Connect safety sensor to **A4** (HIGH = safe)
- Connect dome closed sensor to **A2** (HIGH when closed)
- Connect dome opened sensor to **A3** (HIGH when open)
- LEDs D0, D1, D2, D3 indicate dome closed, dome opened, safety status (filtered over 1 minute), and telescope parked respectively
- User LED flashes slowly under normal operation, fast when error encountered, or off when disabled
- Holding the **USER button** for 3 seconds toggles the hardlink on/off

## Compilation & Upload

Requires [arduino-cli](https://arduino.github.io/arduino-cli/) installed with the Arduino Mbed Opta core (custom core: https://github.com/ppp-one/ArduinoCore-mbed, or use the default Arduino Mbed core, however HTTP queries will be slower with the default core).

```bash
# Compile (adjust FQBN to your installed core)
arduino-cli compile --fqbn arduino-git:mbed:opta ./

# Upload to Opta PLC (replace port as needed)
arduino-cli upload -p /dev/cu.usbmodem2101 --fqbn arduino-git:mbed:opta ./
```

## Configuration

Edit IP addresses in [alpaca-master-plc.ino](alpaca-master-plc.ino):
- `ip[]`: PLC's IP address (default: 192.168.1.10)
- `telescopeDirectIP`: Your telescope controller IP
- `domeAlpacaIP`: Your dome controller IP

## Web UI

Control page: `http://192.168.1.10/`

- Sensors
	- Live readout of the dome closed, dome opened, and safety monitor inputs (filtered and raw), plus the telescope park state (refreshed every 2 s)

- Hardlink automatic control
	- Shows current "Running" state and a button to toggle it
	- Confirms: “Are you sure you want to turn the hardlink on/off?”

- Error reset
	- Shows current error state (OK / Error) and a "Reset errors" button

Notes
- When Running is off, the background safety loop idles (no automatic actions) until re-enabled.

## APIs

ASCOM Alpaca (ISafetyMonitorV3, device 0):
- `GET /api/v1/safetymonitor/0/issafe` – safety status (filtered sensor AND error state)
- `GET /api/v1/safetymonitor/0/devicestate` – `IsSafe`
- All members common to every device: `connected` (GET/PUT), `connect`, `disconnect`, `connecting`, `name`, `description`, `driverinfo`, `driverversion`, `interfaceversion` (3), `supportedactions` (empty). `action` and `command*` return a not implemented error.
- Management API: `/management/apiversions`, `/management/v1/description`, `/management/v1/configureddevices`
- Alpaca discovery on UDP port 32227

Parameter names are case insensitive. PUT parameters are read from the form body. An unknown member or endpoint returns HTTP 400.

When the PLC is in an error state (`errorFree` is false), `issafe` returns an Alpaca device error: `ErrorNumber` 1280 (0x500, driver specific), a non-empty `ErrorMessage`, and no `Value`. Clients such as alpyca raise an exception on this response. `devicestate` reports `IsSafe` false. The error clears when errors are reset on the web page.

`issafe` is not refused while disconnected, so one client disconnecting does not cause errors for the others.

Simple control APIs:
- Combined state (used by the web UI)
	- `GET /state` → `{ "running": ..., "errorFree": ..., "domeClosedSensor": ..., "domeOpenedSensor": ..., "safetyMonitorState": ..., "safetyMonitorRaw": ..., "telescopeParked": ..., "isSafe": ... }`

- Hardlink (Running)
	- `GET /hardlink/state` → `{ "running": true|false }`
	- `POST /hardlink/toggle` with `state=on|off|toggle` → `{ "ok": true, "running": ... }`

- Error state (errorFree)
	- `GET /errorfree/state` → `{ "errorFree": true|false }`
	- `POST /errorfree/reset` → `{ "ok": true, "errorFree": true }`


## License

MIT License - see [LICENSE](LICENSE)