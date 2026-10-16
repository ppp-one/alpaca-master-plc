#include <Ethernet.h>
#include <EthernetUdp.h>
#include <aWOT.h>
#include "opta_info.h"
#include <drivers/Watchdog.h>
#include <strings.h> // for strcasecmp

OptaBoardInfo *info;
OptaBoardInfo *boardInfo();

using namespace mbed;

// the media access control (ethernet hardware) address
byte mac[6];
// the IP address
byte ip[] = {192, 168, 1, 10}; // TODO: can this set more easily by the user - DHCP first then fixed?

// Target server IP and port
IPAddress telescopeAlpacaIP(192, 168, 1, 2);
int telescopeAlpacaPort = 80;

IPAddress telescopeDirectIP(192, 168, 1, 11);
int telescopeDirectPort = 3490;

IPAddress safetyMonitorAlpacaIP(192, 168, 1, 12);
int safetyMonitorAlpacaPort = 80;

IPAddress domeAlpacaIP(192, 168, 1, 14);
int domeAlpacaPort = 80;

// dome shutter sensors
volatile bool domeClosedSensor;
constexpr int domeClosedSensorPin = A2;

volatile bool domeOpenedSensor;
constexpr int domeOpenedSensorPin = A3;

// safety monitor
// The raw input is sampled once a second. safetyMonitorState is the median
// (majority vote) of the last minute of samples, so short spurious changes on
// the input are ignored. A real change takes effect after ~30 seconds.
volatile bool safetyMonitorState;
volatile bool safetyMonitorRaw;
constexpr int safetyMonitorPin = A4;
constexpr int SAFETY_SAMPLES = 60;                    // 1 minute window
constexpr unsigned long SAFETY_SAMPLE_INTERVAL = 1000; // 1 second
bool safetySamples[SAFETY_SAMPLES];
int safetySampleIndex = 0;
int safetySafeCount = 0;

// telescope park state, polled by readStatesAndAct
volatile bool telescopeParked = false;

constexpr int CLOSED_DOME_LED = LED_D0;
constexpr int OPENED_DOME_LED = LED_D1;
constexpr int SAFETY_SIGNAL_LED = LED_D2;
constexpr int TELESCOPE_PARKED_LED = LED_D3;

// user button (active LOW) long press toggles the hardlink
constexpr unsigned long HARDLINK_BUTTON_HOLD = 3000; // 3 seconds

// isSafe
volatile bool isSafe = false;
volatile bool errorFree = true;
volatile bool running = true;

// Blocking flags
const unsigned long FUNCTION_TIMEOUT = 10000; // 10 seconds total timeout, per attempt
const unsigned long DEVICE_TIMEOUT = 180000;  // 3 minutes total timeout
const unsigned long WATCHDOG_TIMEOUT = 20000; // 20 seconds watchdog timeout

// network attempts before a request is given up on and errorFree is cleared
const int REQUEST_ATTEMPTS = 3;

// Thread for reading states
static rtos::Thread ReadStatesAndActThread;

// Alpaca error numbers. ERROR_DRIVER (0x500, first driver specific number)
// is returned by issafe while errorFree is false.
constexpr int32_t ERROR_NOT_IMPLEMENTED = 0x400;
constexpr int32_t ERROR_ACTION_NOT_IMPLEMENTED = 0x40C;
constexpr int32_t ERROR_DRIVER = 0x500;

const char *DRIVER_VERSION = "0.1.0";
const char *DEVICE_NAME = "Safety monitor from Master PLC";
const unsigned int ALPACA_PORT = 80;
const unsigned int DISCOVERY_PORT = 32227;

// incremented on every Alpaca response, only used by the loop thread
uint32_t serverTransactionID = 0;

// ASCOM connected state. issafe is not refused while disconnected, so one
// client disconnecting does not make the others see an error. It starts
// connected so clients do not see a disconnected device after a watchdog reset.
bool safetyMonitorConnected = true;

// unique ID for the management API, filled from the MAC address in setup
char safetyMonitorUniqueID[40];

// declared early for the prototypes the Arduino builder adds
struct AlpacaReply;

// Initialize the Ethernet server and aWOT application
EthernetServer server(ALPACA_PORT);
EthernetUDP discoveryUdp;
Application app;

// Debug flag - set to true for Serial debugging
const bool DEBUG = true;

void debugPrint(const char *message)
{
    if (DEBUG)
    {
        Serial.println(message);
    }
}

void debugPrint(const char *message, auto value)
{
    if (DEBUG)
    {
        Serial.print(message);
        Serial.println(value);
    }
}

String split(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = {0, -1};
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++)
    {
        if (data.charAt(i) == separator || i == maxIndex)
        {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i + 1 : i;
        }
    }

    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

// Set the telescope parked state and keep its LED in sync
void setTelescopeParked(bool state)
{
    telescopeParked = state;
    digitalWrite(TELESCOPE_PARKED_LED, telescopeParked ? HIGH : LOW);
}

// Toggle the hardlink on a 3 second press of the user button
void checkUserButton()
{
    static unsigned long pressedAt = 0;
    static bool handled = false;

    // the user button pulls the pin low while pressed
    bool pressed = digitalRead(BTN_USER) == LOW;

    if (!pressed)
    {
        pressedAt = 0;
        handled = false;
        return;
    }

    if (pressedAt == 0)
    {
        pressedAt = millis();
        return;
    }

    if (!handled && millis() - pressedAt >= HARDLINK_BUTTON_HOLD)
    {
        running = !running;
        handled = true;
        debugPrint("Hardlink toggled by user button, running: ", running);
    }
}

// ISR for dome closed sensor
void domeClosedSensorISR()
{
    domeClosedSensor = digitalRead(domeClosedSensorPin) == HIGH;

    if (domeClosedSensor)
    {
        // LED on
        digitalWrite(CLOSED_DOME_LED, HIGH);
    }
    else
    {
        // LED off
        digitalWrite(CLOSED_DOME_LED, LOW);
    }
}

// ISR for dome opened sensor
void domeOpenedSensorISR()
{
    domeOpenedSensor = digitalRead(domeOpenedSensorPin) == HIGH;

    if (domeOpenedSensor)
    {
        // LED on
        digitalWrite(OPENED_DOME_LED, HIGH);
    }
    else
    {
        // LED off
        digitalWrite(OPENED_DOME_LED, LOW);
    }
}

// Set the filtered safety monitor state and keep its LED in sync
void setSafetyMonitorState(bool state)
{
    safetyMonitorState = state;
    isSafe = state;
    digitalWrite(SAFETY_SIGNAL_LED, state ? HIGH : LOW);
}

// Fill the whole window with the current input, so the state is valid at boot
void initSafetyMonitor()
{
    safetyMonitorRaw = digitalRead(safetyMonitorPin) == HIGH;

    for (int i = 0; i < SAFETY_SAMPLES; i++)
    {
        safetySamples[i] = safetyMonitorRaw;
    }
    safetySafeCount = safetyMonitorRaw ? SAFETY_SAMPLES : 0;

    setSafetyMonitorState(safetyMonitorRaw);
}

// Sample the safety input once a second into the window, called from loop().
// The state is safe only when more than half of the samples are safe, a tie
// counts as unsafe.
void sampleSafetyMonitor()
{
    static unsigned long lastSample = 0;

    if (millis() - lastSample < SAFETY_SAMPLE_INTERVAL)
    {
        return;
    }
    lastSample = millis();

    safetyMonitorRaw = digitalRead(safetyMonitorPin) == HIGH;

    // replace the oldest sample and keep the running count of safe samples
    safetySafeCount += (int)safetyMonitorRaw - (int)safetySamples[safetySampleIndex];
    safetySamples[safetySampleIndex] = safetyMonitorRaw;
    safetySampleIndex = (safetySampleIndex + 1) % SAFETY_SAMPLES;

    setSafetyMonitorState(safetySafeCount > SAFETY_SAMPLES / 2);
}

// Read all states
void readStatesAndAct()
{
    while (true)
    {
        // If not running, idle briefly and continue so we can be resumed
        if (!running)
        {
            debugPrint("Not running, idling");
            digitalWrite(LED_BUILTIN, LOW);
            rtos::ThisThread::sleep_for(500);
            continue;
        }

        // if not errorFree, blink and continue
        if (!errorFree)
        {
            debugPrint("Error state detected, blinking LED");

            // keep publishing the parked state, otherwise a latched error
            // freezes the LED and the web page on a stale value
            setTelescopeParked(getTelescopeDirectParkedState());

            digitalWrite(LED_BUILTIN, LOW);
            rtos::ThisThread::sleep_for(100);
            digitalWrite(LED_BUILTIN, HIGH);
            rtos::ThisThread::sleep_for(100);

            // hold the same ~1 s cadence as the normal path so the mount is
            // not polled five times a second while in error
            rtos::ThisThread::sleep_for(800);
            continue;
        }

        // TODO: wrap in try catch

        digitalWrite(LED_BUILTIN, HIGH);

        // checks
        setTelescopeParked(getTelescopeDirectParkedState());

        debugPrint("Safety monitor state: ", safetyMonitorState);
        debugPrint("Safety monitor raw: ", safetyMonitorRaw);
        debugPrint("Safety monitor safe samples: ", safetySafeCount);
        debugPrint("Dome closed state: ", domeClosedSensor);
        debugPrint("Dome opened state: ", domeOpenedSensor);
        debugPrint("Telescope parked state: ", telescopeParked);
        debugPrint("isSafe: ", isSafe);
        debugPrint("errorFree: ", errorFree);

        // if dome possibly stuck open and telescope not parked, abort all movements
        if (!telescopeParked && !domeClosedSensor && !domeOpenedSensor && running)
        {
            debugPrint("Dome possibly stuck open and telescope not parked, aborting movements");
            abortDomeMovement();
            abortTelescopeMovement();
            errorFree = false;
        }

        bool action = (!safetyMonitorState || (domeClosedSensor && !telescopeParked)) && errorFree && running;
        // | safetyMonitorState | domeClosedSensor | telescopeParked | errorFree | action |
        // | ------------------ | ---------------- | --------------- | --------- | ------ |
        // | 0                  | 0                | 0               | 0         | 0      |
        // | 0                  | 0                | 1               | 0         | 0      |
        // | 0                  | 1                | 0               | 0         | 0      |
        // | 0                  | 1                | 1               | 0         | 0      |
        // | 1                  | 0                | 0               | 0         | 0      |
        // | 1                  | 0                | 1               | 0         | 0      |
        // | 1                  | 1                | 0               | 0         | 0      |
        // | 1                  | 1                | 1               | 0         | 0      |
        // | 0                  | 0                | 0               | 1         | 1      |
        // | 0                  | 0                | 1               | 1         | 1      |
        // | 0                  | 1                | 0               | 1         | 1      |
        // | 0                  | 1                | 1               | 1         | 1      |
        // | 1                  | 0                | 0               | 1         | 0      |
        // | 1                  | 0                | 1               | 1         | 0      |
        // | 1                  | 1                | 0               | 1         | 1      |
        // | 1                  | 1                | 1               | 1         | 0      |

        if (action)
        {
            debugPrint("Action: closing observatory");
            closeObservatory();
        }

        rtos::ThisThread::sleep_for(100);
        digitalWrite(LED_BUILTIN, LOW);
        rtos::ThisThread::sleep_for(900);
    }
}

bool getTelescopeDirectParkedState()
{
    debugPrint("Checking telescope parked state");
    // TCP/IP connection to the telescope
    String response = simpleNetcat(telescopeDirectIP, telescopeDirectPort, ":Ginfo#", true);

    if (response.length() == 0)
    {
        debugPrint("Telescope Ginfo empty response");
        return false; // conservative default
    }

    // Trim trailing '#'
    if (response.endsWith("#"))
    {
        response.remove(response.length() - 1);
    }

    // #:Ginfo#
    // Get multiple information. Returns a string where multiple data are encoded, separated
    // by commas ',', and terminated by '#'. Data are recognized by their position in the string.
    // The data are the following:
    // Position Datum
    // 1        The telescope right ascension in hours and decimals (from 000.00000 to 23.99999),
    //          true equinox and equator of date of observation (i.e. Jnow).
    // 2        The telescope declination in degrees and decimals (from –90.0000 to +90.0000),
    //          true equinox and equator of date of observation (i.e. Jnow).
    // 3        A flag indicating the side of the pier on which the telescope is currently positioned
    //          ("E" or "W").
    // 4        The telescope azimuth in degrees and decimals (from 000.0000 to 359.9999).
    // 5        The telescope altitude in degrees and decimals (from -90.0000 to +90.0000).
    // 6        The julian date (JJJJJJJ.JJJJJJJJ), UTC, with leap second flag (see command
    //          :GJD2# for the description of this datum).
    // 7        A number encoding the status of the mount as in the :Gstat command.
    // 8        A number returning the slew status (0 if :D# would return no slew, 1 otherwise).
    // The string is terminated by '#'. Other parameters may be added in future at the end of
    // the string: do not assume that the number of parameters will stay the same.
    // Available from version 2.14.9 (previous versions may have this command but it was
    // experimental and possibly with a different format).
    // 09.852348,-08.20370,E,257.29757,+00.96283,2460935.12144972,5,0#

    // Extract the status
    String status = split(response, ',', 6);
    float azimuth = split(response, ',', 3).toFloat();
    float altitude = split(response, ',', 4).toFloat();

    // altitude and azimuth must be with 2 degrees of 257.29757,+00.96283
    float parkAzimuth = 257.29757;
    float parkAltitude = 0.96283;
    bool isParked = (status == "5") && (altitude < (parkAltitude + 2)) && (altitude > (parkAltitude - 2)) && (azimuth < (parkAzimuth + 2)) && (azimuth > (parkAzimuth - 2));

    debugPrint("Telescope status: ", status);
    debugPrint("Telescope azimuth: ", azimuth);
    debugPrint("Telescope altitude: ", altitude);
    debugPrint("Telescope isParked: ", isParked);

    // Check if the mount is parked
    return isParked;
}

void closeObservatory()
{
    // Check if the telescope is parked. This must publish to the global so the
    // LED and the web page follow the close sequence, which can block this
    // thread for minutes.
    setTelescopeParked(getTelescopeDirectParkedState());

    // Park the telescope if it is not already parked
    if (!telescopeParked)
    {
        // Abort any current movements
        abortDomeMovement();
        rtos::ThisThread::sleep_for(100);

        abortTelescopeMovement();
        rtos::ThisThread::sleep_for(100);

        // Only park telescope if dome open
        if (domeOpenedSensor)
        {
            parkTelescope();
            rtos::ThisThread::sleep_for(100);
        }

        // Check if the telescope is parked again
        setTelescopeParked(getTelescopeDirectParkedState());
    }

    if (telescopeParked && domeClosedSensor != 1)
    {
        // Close the dome
        closeDome();
    }
}

void parkTelescope()
{
    debugPrint("Parking telescope");
    // Park the telescope
    simpleNetcat(telescopeDirectIP, telescopeDirectPort, ":KA#", false);
    rtos::ThisThread::sleep_for(100);

    // Check if the telescope is parked in while loop, timeout after 180 seconds
    unsigned long startTime = millis();
    while (true)
    {
        // publish every poll so the LED and web page follow the park
        setTelescopeParked(getTelescopeDirectParkedState());

        if (telescopeParked)
        {
            break;
        }

        if (millis() - startTime > DEVICE_TIMEOUT)
        {
            errorFree = false;
            break;
        }
        rtos::ThisThread::sleep_for(1000);
    }
}

void closeDome()
{
    debugPrint("Closing dome");
    // Close the dome
    String payload = sendRequest("PUT", "/api/v1/dome/0/closeshutter", "", domeAlpacaIP, domeAlpacaPort);

    // Check if the dome is closed in while loop, timeout after 180 seconds
    unsigned long startTime = millis();
    while (domeClosedSensor != 1)
    {
        if (millis() - startTime > DEVICE_TIMEOUT)
        {
            errorFree = false;
            break;
        }

        // closing can take minutes, keep the parked readout live meanwhile
        setTelescopeParked(getTelescopeDirectParkedState());

        rtos::ThisThread::sleep_for(1000);
    }
}

void abortDomeMovement()
{
    debugPrint("Aborting dome movement");
    // Abort the dome movement
    sendRequest("PUT", "/api/v1/dome/0/abortslew", "", domeAlpacaIP, domeAlpacaPort);
}

void abortTelescopeMovement()
{
    debugPrint("Aborting telescope movement");
    // Abort the telescope movement
    simpleNetcat(telescopeDirectIP, telescopeDirectPort, ":STOP#", false);
}

// Function to forward a request to a server and receive a response
String sendRequest(const char *methodStr, const char *path, String requestBuffer, IPAddress targetServerIP, int targetServerPort)
{
    for (int attempt = 1; attempt <= REQUEST_ATTEMPTS; attempt++)
    {
        // Per attempt timeout
        unsigned long functionStartTime = millis();

        debugPrint("Request attempt: ", attempt);
        debugPrint("Request method: ", methodStr);
        debugPrint("Request path: ", path);
        debugPrint("Request body: ", requestBuffer.c_str());

        // Create Ethernet client
        EthernetClient ethClient;

        // Set timeout for the client operations
        ethClient.setSocketTimeout(FUNCTION_TIMEOUT); // timeout for read operations

        // Attempt to connect to the target server
        bool connectSuccess = false;
        while (!connectSuccess)
        {
            if (ethClient.connect(targetServerIP, targetServerPort))
            {
                connectSuccess = true;
                break;
            }

            // Check per attempt timeout
            if (millis() - functionStartTime > FUNCTION_TIMEOUT)
            {
                Serial.println("Function execution timeout during connection");
                break;
            }

            rtos::ThisThread::sleep_for(100);
        }

        if (!connectSuccess)
        {
            Serial.println("Connection to target server failed");
            ethClient.stop();
            rtos::ThisThread::sleep_for(100);
            continue;
        }

        debugPrint("Connected to target server");

        // Send the request line
        ethClient.print(methodStr);
        ethClient.print(" ");
        ethClient.print(path);

        // For GET requests, append parameters to the URL if they exist
        if (strcmp(methodStr, "GET") == 0 && requestBuffer.length() > 0)
        {
            ethClient.print("?");
            ethClient.print(requestBuffer);
        }

        ethClient.println(" HTTP/1.0");

        // Add required headers
        ethClient.print("Host: ");
        ethClient.print(targetServerIP[0]);
        for (int i = 1; i < 4; i++)
        {
            ethClient.print(".");
            ethClient.print(targetServerIP[i]);
        }
        ethClient.print(":");
        ethClient.println(targetServerPort);

        ethClient.println("Connection: close");

        // Add Content-Length and Content-Type for PUT requests with body
        if ((strcmp(methodStr, "PUT") == 0) &&
            requestBuffer.length() > 0)
        {
            ethClient.print("Content-Length: ");
            ethClient.println(requestBuffer.length());
            ethClient.println("Content-Type: application/x-www-form-urlencoded");

            // End headers with blank line
            ethClient.println();

            // Send the body
            ethClient.print(requestBuffer);
        }
        else
        {
            // End headers with blank line
            ethClient.println();
        }

        debugPrint("Request sent to target server");

        String header = "";
        String body = "";
        bool isBody = false;
        bool readTimeout = false;

        // keep reading while bytes are still buffered, the peer closes the
        // connection as soon as it has sent the response
        while (ethClient.connected() || ethClient.available())
        {
            // Check response timeout
            if (millis() - functionStartTime > FUNCTION_TIMEOUT)
            {
                Serial.println("Response read timeout");
                readTimeout = true;
                break;
            }

            if (ethClient.available())
            {
                char c = ethClient.read();

                if (!isBody)
                {
                    header += c;
                    if (header.endsWith("\r\n\r\n"))
                    {
                        isBody = true; // Switch to body parsing
                    }
                }
                else
                {
                    body += c;
                }
            }
        }

        ethClient.stop();

        // debugPrint("Header:", header);
        debugPrint("Body:", body);
        debugPrint(" ");

        // a timed out or empty response is a failed attempt, retry rather than
        // handing the caller a body it cannot act on
        if (readTimeout || body.length() == 0)
        {
            debugPrint("No usable response, retrying");
            rtos::ThisThread::sleep_for(100);
            continue;
        }

        return body;
    }

    errorFree = false;
    Serial.print("Failed to send request after ");
    Serial.print(REQUEST_ATTEMPTS);
    Serial.println(" attempts");
    return "";
}

// Function to send data to a server and receive response
String simpleNetcat(IPAddress host, int port, String content, bool expectResponse)
{
    for (int attempt = 1; attempt <= REQUEST_ATTEMPTS; attempt++)
    {
        // Per attempt timeout
        unsigned long functionStartTime = millis();

        debugPrint("Netcat attempt: ", attempt);

        String response = "";
        EthernetClient netCatClient;

        // Set timeout for client operations
        netCatClient.setSocketTimeout(FUNCTION_TIMEOUT); // FUNCTION_TIMEOUT seconds timeout for read operations

        // Attempt to connect to the target server
        bool connectSuccess = false;
        while (!connectSuccess)
        {
            if (netCatClient.connect(host, port))
            {
                connectSuccess = true;
                break;
            }

            // Check per attempt timeout
            if (millis() - functionStartTime > FUNCTION_TIMEOUT)
            {
                Serial.println("Function execution timeout during connection");
                break;
            }

            rtos::ThisThread::sleep_for(100);
        }

        if (!connectSuccess)
        {
            Serial.println("Connection to target server failed");
            netCatClient.stop();
            rtos::ThisThread::sleep_for(100);
            continue;
        }

        // Send the content
        netCatClient.print(content);

        // nothing to wait for on fire and forget commands
        if (!expectResponse)
        {
            netCatClient.stop();
            return "";
        }

        // keep reading while bytes are still buffered, the mount closes the
        // connection as soon as it has sent the reply
        while (netCatClient.connected() || netCatClient.available())
        {
            // Check response timeout
            if (millis() - functionStartTime > FUNCTION_TIMEOUT)
            {
                Serial.println("Response read timeout");
                break;
            }

            if (netCatClient.available())
            {
                char c = netCatClient.read();
                response += c;
            }

            if (response.endsWith("#"))
            {
                break;
            }
        }

        // Close the connection
        netCatClient.stop();

        debugPrint("Response:", response);

        // a complete reply is terminated by '#', anything shorter is a failed
        // attempt. Retry rather than returning a truncated string, which the
        // caller would parse as "not parked"
        if (!response.endsWith("#"))
        {
            debugPrint("Incomplete response, retrying");
            rtos::ThisThread::sleep_for(100);
            continue;
        }

        return response;
    }

    errorFree = false;
    Serial.print("Failed to send request after ");
    Serial.print(REQUEST_ATTEMPTS);
    Serial.println(" attempts");
    return "";
}

// Alpaca parameter names are case insensitive. GET parameters are in the
// query string, PUT parameters are in the form encoded body.
void readParams(Request &req, char *params, int length)
{
    params[0] = '\0';

    if (req.method() == Request::GET)
    {
        strncpy(params, req.query(), length - 1);
        params[length - 1] = '\0';
    }
    else
    {
        int toRead = min(req.left(), length - 1);
        int count = req.readBytes(params, toRead);
        params[count] = '\0';
    }
}

bool findParam(const char *params, const char *name, char *value, int valueLength)
{
    size_t nameLength = strlen(name);
    const char *position = params;

    while (*position)
    {
        const char *end = strchr(position, '&');
        if (end == NULL)
        {
            end = position + strlen(position);
        }

        const char *equals = (const char *)memchr(position, '=', end - position);
        if (equals != NULL && (size_t)(equals - position) == nameLength && strncasecmp(position, name, nameLength) == 0)
        {
            int count = min((int)(end - equals - 1), valueLength - 1);
            memcpy(value, equals + 1, count);
            value[count] = '\0';
            return true;
        }

        position = *end ? end + 1 : end;
    }

    return false;
}

// Returns true if the parameter is "true" or "false", and sets result
bool findBoolParam(const char *params, const char *name, bool &result)
{
    char value[8];
    if (!findParam(params, name, value, sizeof(value)))
    {
        return false;
    }
    if (strcasecmp(value, "true") == 0)
    {
        result = true;
        return true;
    }
    if (strcasecmp(value, "false") == 0)
    {
        result = false;
        return true;
    }
    return false;
}

// Reply to one Alpaca request
struct AlpacaReply
{
    enum Type
    {
        NONE, // method without a return value
        BOOL,
        INT,
        STRING,
        JSON // value printed as is, for arrays and objects
    };

    Type type = NONE;
    bool boolValue = false;
    int32_t intValue = 0;
    const char *text = "";
    int32_t errorNumber = 0;
    const char *errorMessage = "";
    bool badRequest = false;

    void setBool(bool value)
    {
        type = BOOL;
        boolValue = value;
    }

    void setInt(int32_t value)
    {
        type = INT;
        intValue = value;
    }

    void setString(const char *value)
    {
        type = STRING;
        text = value;
    }

    void setJson(const char *value)
    {
        type = JSON;
        text = value;
    }

    void setError(int32_t number, const char *message)
    {
        errorNumber = number;
        errorMessage = message;
    }

    void setBadRequest(const char *message)
    {
        badRequest = true;
        errorMessage = message;
    }
};

void sendReply(Response &res, const char *params, AlpacaReply &reply)
{
    if (reply.badRequest)
    {
        res.status(400);
        res.set("Content-Type", "text/plain");
        res.print(reply.errorMessage);
        res.end();
        return;
    }

    char buffer[16];
    uint32_t clientTransactionID = 0;
    if (findParam(params, "ClientTransactionID", buffer, sizeof(buffer)))
    {
        clientTransactionID = strtoul(buffer, NULL, 10);
    }

    // Alpaca: 1 to 4294967295, unique for each transaction
    if (++serverTransactionID == 0)
    {
        serverTransactionID = 1;
    }

    res.set("Content-Type", "application/json");
    res.print("{\"ClientTransactionID\":");
    res.print(clientTransactionID);
    res.print(",\"ServerTransactionID\":");
    res.print(serverTransactionID);
    res.print(",\"ErrorNumber\":");
    res.print(reply.errorNumber);
    res.print(",\"ErrorMessage\":\"");
    res.print(reply.errorMessage);
    res.print("\"");

    // a value is only sent when there is no error
    if (reply.errorNumber == 0 && reply.type != AlpacaReply::NONE)
    {
        res.print(",\"Value\":");
        switch (reply.type)
        {
        case AlpacaReply::BOOL:
            res.print(reply.boolValue ? "true" : "false");
            break;
        case AlpacaReply::INT:
            res.print(reply.intValue);
            break;
        case AlpacaReply::STRING:
            res.print("\"");
            res.print(reply.text);
            res.print("\"");
            break;
        default:
            res.print(reply.text);
            break;
        }
    }

    res.print("}");
    res.end();
}

// Members common to all device types. Returns false if the member is unknown.
bool commonMember(const char *member, bool isPut, const char *params, bool &connected,
                  const char *description, const char *deviceState, AlpacaReply &reply)
{
    if (isPut)
    {
        if (strcmp(member, "connected") == 0)
        {
            bool value;
            if (!findBoolParam(params, "Connected", value))
            {
                reply.setBadRequest("Connected parameter missing or invalid");
            }
            else
            {
                connected = value;
            }
        }
        else if (strcmp(member, "connect") == 0)
        {
            connected = true;
        }
        else if (strcmp(member, "disconnect") == 0)
        {
            connected = false;
        }
        else if (strcmp(member, "action") == 0)
        {
            reply.setError(ERROR_ACTION_NOT_IMPLEMENTED, "No actions are supported");
        }
        else if (strcmp(member, "commandblind") == 0 || strcmp(member, "commandbool") == 0 || strcmp(member, "commandstring") == 0)
        {
            reply.setError(ERROR_NOT_IMPLEMENTED, "Commands are not implemented");
        }
        else
        {
            return false;
        }
        return true;
    }

    if (strcmp(member, "connected") == 0)
    {
        reply.setBool(connected);
    }
    else if (strcmp(member, "connecting") == 0)
    {
        reply.setBool(false);
    }
    else if (strcmp(member, "name") == 0)
    {
        reply.setString(DEVICE_NAME);
    }
    else if (strcmp(member, "description") == 0)
    {
        reply.setString(description);
    }
    else if (strcmp(member, "driverinfo") == 0)
    {
        reply.setString("Arduino Opta master safety PLC");
    }
    else if (strcmp(member, "driverversion") == 0)
    {
        reply.setString(DRIVER_VERSION);
    }
    else if (strcmp(member, "interfaceversion") == 0)
    {
        reply.setInt(3);
    }
    else if (strcmp(member, "supportedactions") == 0)
    {
        reply.setJson("[]");
    }
    else if (strcmp(member, "devicestate") == 0)
    {
        reply.setJson(deviceState);
    }
    else
    {
        return false;
    }
    return true;
}

// Handle the safety monitor endpoint requests
void safetyMonitorEndPoint(Request &req, Response &res)
{
    char member[32];
    char params[256];
    req.route("member", member, sizeof(member));
    readParams(req, params, sizeof(params));
    bool isPut = req.method() == Request::PUT;

    debugPrint("Safety monitor request: ", member);

    // A latched PLC error is reported as an Alpaca device error on issafe and
    // as unsafe in devicestate, so the client never trusts a stale state
    bool safe = isSafe && errorFree;

    char deviceState[48];
    snprintf(deviceState, sizeof(deviceState),
             "[{\"Name\":\"IsSafe\",\"Value\":%s}]",
             safe ? "true" : "false");

    AlpacaReply reply;

    if (commonMember(member, isPut, params, safetyMonitorConnected, "Observatory safety monitor from the master PLC", deviceState, reply))
    {
        // handled
    }
    else if (!isPut && strcmp(member, "issafe") == 0)
    {
        if (!errorFree)
        {
            reply.setError(ERROR_DRIVER, "Master PLC is in an error state, reset errors on the PLC web page");
        }
        else
        {
            reply.setBool(safe);
        }
    }
    else
    {
        reply.setBadRequest("Unknown safety monitor member");
    }

    sendReply(res, params, reply);
}

// Management API

void apiVersions(Request &req, Response &res)
{
    char params[256];
    readParams(req, params, sizeof(params));
    AlpacaReply reply;
    reply.setJson("[1]");
    sendReply(res, params, reply);
}

void managementDescription(Request &req, Response &res)
{
    char params[256];
    readParams(req, params, sizeof(params));
    char description[160];
    snprintf(description, sizeof(description),
             "{\"ServerName\":\"Alpaca master PLC\",\"Manufacturer\":\"PPP\",\"ManufacturerVersion\":\"%s\",\"Location\":\"Observatory\"}",
             DRIVER_VERSION);
    AlpacaReply reply;
    reply.setJson(description);
    sendReply(res, params, reply);
}

void configuredDevices(Request &req, Response &res)
{
    char params[256];
    readParams(req, params, sizeof(params));
    char devices[160];
    snprintf(devices, sizeof(devices),
             "[{\"DeviceName\":\"%s\",\"DeviceType\":\"SafetyMonitor\",\"DeviceNumber\":0,\"UniqueID\":\"%s\"}]",
             DEVICE_NAME, safetyMonitorUniqueID);
    AlpacaReply reply;
    reply.setJson(devices);
    sendReply(res, params, reply);
}

// Runs last, for any request no route has answered
void badRequest(Request &req, Response &res)
{
    res.status(400);
    res.set("Content-Type", "text/plain");
    res.print("Unknown endpoint");
    res.end();
}

// Answer Alpaca discovery broadcasts with the HTTP port
void handleDiscovery()
{
    int size = discoveryUdp.parsePacket();
    if (size <= 0)
    {
        return;
    }

    char packet[32] = {0};
    discoveryUdp.read(packet, min(size, (int)sizeof(packet) - 1));

    if (strncmp(packet, "alpacadiscovery", 15) == 0)
    {
        char reply[24];
        snprintf(reply, sizeof(reply), "{\"AlpacaPort\":%u}", ALPACA_PORT);
        discoveryUdp.beginPacket(discoveryUdp.remoteIP(), discoveryUdp.remotePort());
        discoveryUdp.write((const uint8_t *)reply, strlen(reply));
        discoveryUdp.endPacket();
    }
}

// Simple API: get running state
void hardlinkState(Request &req, Response &res)
{
        res.set("Content-Type", "application/json");
        res.print("{");
        res.print("\"running\": ");
        res.print(running ? "true" : "false");
        res.print("}");
        res.end();
}

// Simple API: toggle/set running state via form or query param `state` = on|off|toggle
void hardlinkToggle(Request &req, Response &res)
{
        char stateBuf[16];
        bool hasState = false;

        // Try query string first
        if (req.query("state", stateBuf, sizeof(stateBuf)))
        {
                hasState = true;
        }

        // If no query param, try reading form data
        if (!hasState)
        {
                char name[32];
                char value[32];
                while (req.left())
                {
                        if (!req.form(name, sizeof(name), value, sizeof(value)))
                        {
                                break;
                        }
                        if (strcmp(name, "state") == 0)
                        {
                                strncpy(stateBuf, value, sizeof(stateBuf) - 1);
                                stateBuf[sizeof(stateBuf) - 1] = '\0';
                                hasState = true;
                        }
                }
        }

        if (hasState)
        {
                if (strcasecmp(stateBuf, "on") == 0 || strcmp(stateBuf, "1") == 0 || strcasecmp(stateBuf, "true") == 0)
                {
                        running = true;
                }
                else if (strcasecmp(stateBuf, "off") == 0 || strcmp(stateBuf, "0") == 0 || strcasecmp(stateBuf, "false") == 0)
                {
                        running = false;
                }
                else if (strcasecmp(stateBuf, "toggle") == 0)
                {
                        running = !running;
                }
                else
                {
                        res.sendStatus(400);
                        res.end();
                        return;
                }
        }
        else
        {
                // No param provided, default to toggle
                running = !running;
        }

        res.set("Content-Type", "application/json");
        res.print("{");
        res.print("\"ok\": true, \"running\": ");
        res.print(running ? "true" : "false");
        res.print("}");
        res.end();
}

// API: get current errorFree state
void errorFreeState(Request &req, Response &res)
{
    res.set("Content-Type", "application/json");
    res.print("{");
    res.print("\"errorFree\": ");
    res.print(errorFree ? "true" : "false");
    res.print("}");
    res.end();
}

// API: reset errorFree to true
void errorFreeReset(Request &req, Response &res)
{
    errorFree = true;
    res.set("Content-Type", "application/json");
    res.print("{\"ok\": true, \"errorFree\": true}");
    res.end();
}

// API: all states in one response, used by the page to refresh itself
void statusState(Request &req, Response &res)
{
    res.set("Content-Type", "application/json");
    res.print("{");
    res.print("\"running\": ");
    res.print(running ? "true" : "false");
    res.print(", \"errorFree\": ");
    res.print(errorFree ? "true" : "false");
    res.print(", \"domeClosedSensor\": ");
    res.print(domeClosedSensor ? "true" : "false");
    res.print(", \"domeOpenedSensor\": ");
    res.print(domeOpenedSensor ? "true" : "false");
    res.print(", \"safetyMonitorState\": ");
    res.print(safetyMonitorState ? "true" : "false");
    res.print(", \"safetyMonitorRaw\": ");
    res.print(safetyMonitorRaw ? "true" : "false");
    res.print(", \"telescopeParked\": ");
    res.print(telescopeParked ? "true" : "false");
    res.print(", \"isSafe\": ");
    res.print(isSafe ? "true" : "false");
    res.print("}");
    res.end();
}

// Single UI page at root with sensor readout, hardlink toggle and error reset
// All values are filled in and kept up to date by polling /state, so the page
// also follows changes made from the user button on the PLC.
void rootPage(Request &req, Response &res)
{
        res.set("Content-Type", "text/html; charset=utf-8");

        res.print(R"~(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Alpaca master PLC</title>
    <style>
        body { font-family: system-ui; margin: 0; padding: 0; display: flex; justify-content: center; align-items: center; min-height: 100vh; background-color: #121212; color: #fff; }
        .container { padding: 20px; border-radius: 8px; width: 320px; background-color: #1e1e1e; box-shadow: 0 0 10px rgba(0,0,0,0.3); }
        h2 { margin-top: 0; font-size: large; font-weight: 500; color: #ccc; }
        h3 { font-size: medium; font-weight: 500; color: #ccc; margin-bottom: 4px; }
        .row { display: flex; justify-content: space-between; align-items: baseline; padding: 6px 0; border-bottom: 1px solid #2a2a2a; font-size: 15px; }
        .row:last-child { border-bottom: none; }
        .row span { color: #999; }
        .state { margin: 8px 0 14px; font-size: 18px; }
        .ok { color: #2ecc71; }
        .warn { color: #e67e22; }
        .err { color: #e74c3c; }
        .unknown { color: #999; }
        .btn { display: inline-block; width: 100%; padding: 12px; border-radius: 6px; border: none; cursor: pointer; font-weight: 600; }
        .btn:disabled { opacity: 0.5; cursor: default; }
        .on { background: #2ecc71; color: #000; }
        .off { background: #e67e22; color: #000; }
        .reset { background: #93d4ff; color: #000; }
        .stale { color: #e74c3c; font-size: 13px; margin-top: 12px; display: none; }
        hr { border: none; border-top: 1px solid #333; margin: 22px 0; }
    </style>
</head>
<body>
    <div class="container">
        <h2>Alpaca master PLC</h2>

        <h3>Sensors</h3>
        <div class="row"><span>Dome closed</span><strong id="domeClosedSensor" class="unknown">&hellip;</strong></div>
        <div class="row"><span>Dome opened</span><strong id="domeOpenedSensor" class="unknown">&hellip;</strong></div>
        <div class="row"><span>Safety monitor (1 min)</span><strong id="safetyMonitorState" class="unknown">&hellip;</strong></div>
        <div class="row"><span>Safety monitor (raw)</span><strong id="safetyMonitorRaw" class="unknown">&hellip;</strong></div>
        <div class="row"><span>Telescope</span><strong id="telescopeParked" class="unknown">&hellip;</strong></div>

        <hr>

        <h3>Hardlink</h3>
        <div class="state">Current state: <strong id="running" class="unknown">&hellip;</strong></div>
        <button id="toggleBtn" class="btn" disabled>&hellip;</button>

        <hr>

        <h3>Error state</h3>
        <div class="state">Current: <strong id="errorFree" class="unknown">&hellip;</strong></div>
        <button id="resetBtn" class="btn reset">Reset errors</button>

        <div id="stale" class="stale">Connection lost, values may be out of date.</div>
    </div>
    <script>
        const toggleBtn = document.getElementById('toggleBtn');
        const resetBtn = document.getElementById('resetBtn');
        const stale = document.getElementById('stale');
        let running = null;

        function set(id, text, cls) {
            const el = document.getElementById(id);
            el.textContent = text;
            el.className = cls;
        }

        function render(s) {
            set('domeClosedSensor', s.domeClosedSensor ? 'Closed' : 'Not closed', s.domeClosedSensor ? 'ok' : 'unknown');
            set('domeOpenedSensor', s.domeOpenedSensor ? 'Opened' : 'Not opened', s.domeOpenedSensor ? 'ok' : 'unknown');
            set('safetyMonitorState', s.safetyMonitorState ? 'Safe' : 'Unsafe', s.safetyMonitorState ? 'ok' : 'err');
            set('safetyMonitorRaw', s.safetyMonitorRaw ? 'Safe' : 'Unsafe', s.safetyMonitorRaw ? 'ok' : 'err');
            set('telescopeParked', s.telescopeParked ? 'Parked' : 'Not parked', s.telescopeParked ? 'ok' : 'unknown');
            set('running', s.running ? 'On' : 'Off', s.running ? 'ok' : 'warn');
            set('errorFree', s.errorFree ? 'OK' : 'Error', s.errorFree ? 'ok' : 'err');

            running = s.running;
            toggleBtn.textContent = running ? 'Turn Off' : 'Turn On';
            toggleBtn.className = 'btn ' + (running ? 'off' : 'on');
            toggleBtn.disabled = false;
        }

        async function refresh() {
            try {
                const resp = await fetch('/state');
                if (!resp.ok) throw new Error('Request failed');
                render(await resp.json());
                stale.style.display = 'none';
            } catch (e) {
                stale.style.display = 'block';
            }
        }

        toggleBtn.addEventListener('click', async () => {
            const nextState = running ? 'off' : 'on';
            if (!confirm(`Are you sure you want to turn the hardlink ${nextState}?`)) return;
            try {
                const resp = await fetch('/hardlink/toggle', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                    body: `state=${nextState}`
                });
                if (!resp.ok) throw new Error('Request failed');
                await refresh();
            } catch (e) {
                alert('Failed to toggle: ' + e.message);
            }
        });

        resetBtn.addEventListener('click', async () => {
            if (!confirm('Are you sure you want to reset errors?')) return;
            try {
                const resp = await fetch('/errorfree/reset', { method: 'POST' });
                if (!resp.ok) throw new Error('Request failed');
                await refresh();
            } catch (e) {
                alert('Failed to reset: ' + e.message);
            }
        });

        refresh();
        setInterval(refresh, 2000);
    </script>
</body>
</html>
)~");
        res.end();
}

void setup()
{
    Serial.begin(9600);

    // wait 5s for terminal to come up
    // and for OCS to pick up stale safety state 
    // if watchdog reset occurred
    delay(5000);

    // start the watchdog
    mbed::Watchdog::get_instance().start(WATCHDOG_TIMEOUT);

    info = boardInfo();
    if (info->magic == 0xB5)
    {
        for (int i = 0; i < 6; i++)
        {
            mac[i] = info->mac_address[i];
        }
    }

    if (Ethernet.begin(mac, ip))
    {
        Serial.println(Ethernet.localIP());
    }
    else
    {
        Serial.println("Ethernet failed");
    }

    // set pins for LEDs
    pinMode(CLOSED_DOME_LED, OUTPUT);
    pinMode(OPENED_DOME_LED, OUTPUT);
    pinMode(SAFETY_SIGNAL_LED, OUTPUT);
    pinMode(TELESCOPE_PARKED_LED, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    // set pins for sensors
    pinMode(domeClosedSensorPin, INPUT);
    pinMode(domeOpenedSensorPin, INPUT);
    pinMode(safetyMonitorPin, INPUT);

    // user button toggles the hardlink on a long press
    pinMode(BTN_USER, INPUT);

    // call all ISRs
    domeClosedSensorISR();
    domeOpenedSensorISR();

    // the safety monitor is polled from loop(), not by an interrupt
    initSafetyMonitor();

    // parked state is unknown until the first poll
    setTelescopeParked(false);

    // Attach interrupts to the sensors
    attachInterrupt(digitalPinToInterrupt(domeClosedSensorPin), domeClosedSensorISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(domeOpenedSensorPin), domeOpenedSensorISR, CHANGE);

    snprintf(safetyMonitorUniqueID, sizeof(safetyMonitorUniqueID), "%02x%02x%02x%02x-%02x%02x-4000-8000-000000000001",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    // Alpaca endpoints
    app.get("/api/v1/safetymonitor/0/:member", &safetyMonitorEndPoint);
    app.put("/api/v1/safetymonitor/0/:member", &safetyMonitorEndPoint);

    app.get("/management/apiversions", &apiVersions);
    app.get("/management/v1/description", &managementDescription);
    app.get("/management/v1/configureddevices", &configuredDevices);

    // Control page with sensor readout, hardlink toggle and error reset
    app.get("/", &rootPage);
    app.get("/state", &statusState);

    // Simple hardlink control endpoints
    app.get("/hardlink/state", &hardlinkState);
    app.post("/hardlink/toggle", &hardlinkToggle);

    // Error state endpoints
    app.get("/errorfree/state", &errorFreeState);
    app.post("/errorfree/reset", &errorFreeReset);

    // must be registered last, every handler above must call res.end()
    app.use(&badRequest);

    // start ReadStatesAndActThread thread
    ReadStatesAndActThread.start(readStatesAndAct);

    // begin the server
    server.begin();
    discoveryUdp.begin(DISCOVERY_PORT);
    debugPrint("Server started");
}

void loop()
{
    checkUserButton();
    sampleSafetyMonitor();
    handleDiscovery();

    EthernetClient client = server.available();

    if (client.connected())
    {
        app.process(&client);
        client.stop();
    }
    mbed::Watchdog::get_instance().kick();
}

// arduino-cli compile --fqbn arduino:mbed_opta:opta ./
// arduino-cli upload -p /dev/cu.usbmodem2101 --fqbn arduino:mbed_opta:opta ./