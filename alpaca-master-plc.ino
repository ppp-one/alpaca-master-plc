#include <Ethernet.h>
#include <aWOT.h>
#include <ArduinoJson.h>
#include "opta_info.h"
#include <drivers/Watchdog.h>

OptaBoardInfo *info;
OptaBoardInfo *boardInfo();

using namespace mbed;

// Network configuration for your Arduino
// byte mac[] = {0xA8, 0x61, 0x0A, 0x50, 0x9D, 0x95};
// IPAddress ip(192, 168, 1, 10);
// IPAddress gateway(192, 168, 1, 1);
// IPAddress subnet(255, 255, 255, 0);
// IPAddress dns(8, 8, 8, 8);

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
volatile bool safetyMonitorState;
constexpr int safetyMonitorPin = A4;

constexpr int CLOSED_DOME_LED = LED_D0;
constexpr int OPENED_DOME_LED = LED_D1;
constexpr int SAFETY_SIGNAL_LED = LED_D2;

// isSafe
bool isSafe = false;
bool errorFree = true;

// Blocking flags
const unsigned long FUNCTION_TIMEOUT = 10000; // 10 seconds total timeout
const unsigned long DEVICE_TIMEOUT = 180000;  // 3 minutes total timeout

// Thread for reading states
static rtos::Thread ReadStatesAndActThread;

// Initialize the Ethernet server and aWOT application
EthernetServer server(80);
Application app;

// Alpaca error handling
int32_t ErrorNumber = 0;
String ErrorMessage = "\"\"";

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

// ISR for safety monitor
void safetyMonitorISR()
{
    safetyMonitorState = digitalRead(safetyMonitorPin) == HIGH;

    if (safetyMonitorState)
    {
        // LED on
        digitalWrite(SAFETY_SIGNAL_LED, HIGH);
        isSafe = true;
    }
    else
    {
        // LED off
        digitalWrite(SAFETY_SIGNAL_LED, LOW);
        isSafe = false;
    }
}

// Read all states
void readStatesAndAct()
{
    while (true)
    {
        // TODO: wrap in try catch

        digitalWrite(LED_BUILTIN, HIGH);

        // checks
        bool telescopeParked = getTelescopeDirectParkedState();

        debugPrint("Safety monitor state: ", safetyMonitorState);
        debugPrint("Dome closed state: ", domeClosedSensor);
        debugPrint("Dome opened state: ", domeOpenedSensor);
        debugPrint("Telescope parked state: ", telescopeParked);
        debugPrint("isSafe: ", isSafe);
        debugPrint("errorFree: ", errorFree);

        // if dome possibly stuck open and telescope not parked, abort all movements
        if (!telescopeParked && !domeClosedSensor && !domeOpenedSensor)
        {
            abortDomeMovement();
            abortTelescopeMovement();
            errorFree = false;
        }

        bool action = (!safetyMonitorState || (domeClosedSensor && !telescopeParked)) && errorFree;
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
            closeObservatory();
        }

        rtos::ThisThread::sleep_for(100);
        digitalWrite(LED_BUILTIN, LOW);
        rtos::ThisThread::sleep_for(900);
        mbed::Watchdog::get_instance().kick();
    }
}

bool getTelescopeDirectParkedState()
{
    // TCP/IP connection to the telescope
    String response = simpleNetcat(telescopeDirectIP, telescopeDirectPort, ":Ginfo#", true);

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

    // Check if the mount is parked
    return isParked;
}

void closeObservatory()
{
    // Park the telescope if it is not already parked
    if (getTelescopeDirectParkedState() == false)
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
    }

    rtos::ThisThread::sleep_for(1000);

    if (getTelescopeDirectParkedState() == true && domeClosedSensor != 1)
    {
        // Close the dome
        closeDome();
    }
}

void parkTelescope()
{
    // Park the telescope
    simpleNetcat(telescopeDirectIP, telescopeDirectPort, ":KA#", false);
    rtos::ThisThread::sleep_for(100);

    // Check if the telescope is parked in while loop, timeout after 180 seconds
    unsigned long startTime = millis();
    while (getTelescopeDirectParkedState() == false)
    {
        if (millis() - startTime > DEVICE_TIMEOUT)
        {
            errorFree = false;
            break;
        }
        rtos::ThisThread::sleep_for(1000);
        mbed::Watchdog::get_instance().kick();
    }
}

void closeDome()
{
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
        rtos::ThisThread::sleep_for(1000);
        mbed::Watchdog::get_instance().kick();
    }
}

void abortDomeMovement()
{
    // Abort the dome movement
    sendRequest("PUT", "/api/v1/dome/0/abortslew", "", domeAlpacaIP, domeAlpacaPort);
}

void abortTelescopeMovement()
{
    // Abort the telescope movement
    simpleNetcat(telescopeDirectIP, telescopeDirectPort, ":STOP#", false);
}

// Function to forward a request to a server and receive a response
String sendRequest(const char *methodStr, const char *path, String requestBuffer, IPAddress targetServerIP, int targetServerPort)
{
    int attempts = 0;
    while (attempts < 3)
    {
        // Overall function timeout
        unsigned long functionStartTime = millis();

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

            // Check overall function timeout
            if (millis() - functionStartTime > FUNCTION_TIMEOUT)
            {
                Serial.println("Function execution timeout during connection");
                attempts++;
                break; // Or return an error message
            }

            rtos::ThisThread::sleep_for(100);
        }

        if (connectSuccess)
        {
            debugPrint("Connected to target server");

            // Send the request line
            ethClient.print(methodStr);
            ethClient.print(" ");
            ethClient.print(path);

            // For GET requests, append parameters to the URL if they exist
            if (methodStr == "GET" && requestBuffer.length() > 0)
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
            if ((methodStr == "PUT") &&
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
        }
        else
        {
            Serial.println("Connection to target server failed");
            ethClient.stop();
            // res.status(502); // Bad Gateway
            // res.print("{\"error\":\"Unable to connect to target server\"}");
            attempts++;
            break;
        }

        String header = "";
        String body = "";
        bool isBody = false;

        while (ethClient.connected())
        {
            // Check response timeout
            if (millis() - functionStartTime > FUNCTION_TIMEOUT)
            {
                Serial.println("Response read timeout");
                attempts++;
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

        return body;
    }

    errorFree = false;
    Serial.println("Failed to send request after 3 attempts");
    return "";
}

// Function to send data to a server and receive response
String simpleNetcat(IPAddress host, int port, String content, bool expectResponse)
{
    int attempts = 0;
    while (attempts < 3)
    {
        // Overall function timeout
        unsigned long functionStartTime = millis();

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

            // Check overall function timeout
            if (millis() - functionStartTime > FUNCTION_TIMEOUT)
            {
                Serial.println("Function execution timeout during connection");
                break;
            }

            rtos::ThisThread::sleep_for(100);
        }

        // Try to connect to the server
        if (connectSuccess)
        {
            // Send the content
            netCatClient.print(content);
        }
        else
        {
            Serial.println("Connection to target server failed");
            netCatClient.stop();
            attempts++;
            break;
        }

        while (netCatClient.connected() && expectResponse)
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

        return response;
    }

    errorFree = false;
    Serial.println("Failed to send request after 3 attempts");
    return "";
}

// Handle the endpoint requests
void endPoint(Request &req, Response &res)
{

    res.set("Content-Type", "application/json");
    auto url = split(req.path(), '/', 5);

    uint32_t ClientID = 0;
    uint32_t ClientTransactionID = 0;

    uint32_t ServerID = 0;            // TODO: what is this?
    uint32_t ServerTransactionID = 0; // TODO: what is this? increment?

    float Value;
    String ValueString;
    bool ValueBool;

    char buffer[16]; // Adjust the buffer size as needed

    if (req.query("ClientID", buffer, sizeof(buffer)))
    {
        ClientID = strtoul(buffer, NULL, 10);
    }

    if (req.query("ClientTransactionID", buffer, sizeof(buffer)))
    {
        ClientTransactionID = strtoul(buffer, NULL, 10);
    }

    if (url == "connected")
    {
        ValueBool = true;
    }
    else if (url == "name")
    {
        ValueString = "Safety monitor from Master PLC";
    }
    else if (url == "driverversion")
    {
        ValueString = "0.0.2";
    }
    else if (url == "issafe")
    {
        ValueBool = isSafe && errorFree;
    }
    else
    {
        ErrorNumber = 1;
        ErrorMessage = "\"Invalid path\"";
    }

    res.print("{");
    res.print("\"ClientID\": ");
    res.print(ClientID);
    res.print(", ");
    res.print("\"ClientTransactionID\": ");
    res.print(ClientTransactionID);
    res.print(", ");
    res.print("\"ServerID\": ");
    res.print(ServerID);
    res.print(", ");
    res.print("\"ErrorNumber\": ");
    res.print(ErrorNumber);
    res.print(", ");
    res.print("\"ErrorMessage\": ");
    res.print(ErrorMessage);
    res.print(", ");
    res.print("\"Value\": ");
    if (url == "name" || url == "driverversion")
    {
        res.print("\"");
        res.print(ValueString);
        res.print("\"");
    }
    else if (url == "issafe" || url == "connected")
    {
        res.print(ValueBool ? "true" : "false");
    }
    else
    {
        res.print(Value);
    }
    res.print("}");

    // reset error
    ErrorNumber = 0;
    ErrorMessage = "\"\"";
}

void setup()
{
    Serial.begin(9600);

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
    pinMode(LED_BUILTIN, OUTPUT);

    // set pins for sensors
    pinMode(domeClosedSensorPin, INPUT);
    pinMode(domeOpenedSensorPin, INPUT);
    pinMode(safetyMonitorPin, INPUT);

    // call all ISRs
    domeClosedSensorISR();
    domeOpenedSensorISR();
    safetyMonitorISR();

    // Attach interrupts to the sensors
    attachInterrupt(digitalPinToInterrupt(domeClosedSensorPin), domeClosedSensorISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(domeOpenedSensorPin), domeOpenedSensorISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(safetyMonitorPin), safetyMonitorISR, CHANGE);

    // Define endpoints
    app.get("/api/v1/safetymonitor/0/connected", &endPoint);
    app.put("/api/v1/safetymonitor/0/connected", &endPoint);
    app.get("/api/v1/safetymonitor/0/issafe", &endPoint);
    app.get("/api/v1/safetymonitor/0/name", &endPoint);
    app.get("/api/v1/safetymonitor/0/driverversion", &endPoint);

    // start ReadStatesAndActThread thread
    ReadStatesAndActThread.start(readStatesAndAct);

    // begin the server
    server.begin();
    debugPrint("Server started");

    // start the watchdog
    mbed::Watchdog::get_instance().start(20000); // 20 seconds timeout
}

void loop()
{
    EthernetClient client = server.available();

    if (client.connected())
    {
        app.process(&client);
        client.stop();
    }
}

// arduino-cli compile --fqbn arduino:mbed_opta:opta ./
// arduino-cli upload -p /dev/cu.usbmodem2101 --fqbn arduino:mbed_opta:opta ./