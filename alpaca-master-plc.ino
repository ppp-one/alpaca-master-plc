#include <Ethernet.h>
#include <aWOT.h>
#include <ArduinoJson.h>

using namespace mbed;

// Network configuration for your Arduino
byte mac[] = {0xA8, 0x61, 0x0A, 0x50, 0x9D, 0x95};
IPAddress ip(192, 168, 1, 177);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(8, 8, 8, 8);

// Target server IP and port
IPAddress domeAlpacaIP(192, 168, 1, 14);
int domeAlpacaPort = 80;

IPAddress telescopeAlpacaIP(192, 168, 1, 7);
int telescopeAlpacaPort = 80;

IPAddress telescopeDirectIP(192, 168, 1, 7);
int telescopeDirectPort = 3490;

IPAddress safetyMonitorAlpacaIP(192, 168, 1, 12);
int safetyMonitorAlpacaPort = 80;

// Blocking flags
volatile bool blockDome = false;
volatile bool blockTelescope = false;
bool requestHappening = false;

// Thread for reading states
static rtos::Thread ReadStatesThread;

// Initialize the Ethernet server and aWOT application
EthernetServer server(80);
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

// Read all states
void readStates()
{
  while (true)
  {
    // TODO: wrap in try catch

    digitalWrite(LED_BUILTIN, HIGH);

    // checks
    bool safetyMonitorState = getSafetyMonitorState();
    debugPrint("Safety monitor state: ", safetyMonitorState);
    int domeShutterState = getDomeShutterState();
    debugPrint("Dome shutter state: ", domeShutterState);
    bool telescopeParked = getTelescopeDirectParkedState();
    debugPrint("Telescope parked state: ", telescopeParked);

    if (domeShutterState != 0)
    {
      blockTelescope = true;
    }
    else
    {
      blockTelescope = false;
    }

    if (telescopeParked == false)
    {
      blockDome = true;
    }
    else
    {
      blockDome = false;
    }

    if (safetyMonitorState == false)
    {
      closeObservatory();
    }

    rtos::ThisThread::sleep_for(2500);
    digitalWrite(LED_BUILTIN, LOW);
    rtos::ThisThread::sleep_for(2500);
  }
}

int getDomeShutterState()
{

  // shutterOpen = 0
  // shutterClosed = 1
  // shutterOpening = 2
  // shutterClosing = 3
  // shutterError = 4

  String payload = sendRequest("GET", "/api/v1/dome/0/shutterstatus", "", domeAlpacaIP, domeAlpacaPort);

  DynamicJsonDocument doc(1024); // Heap-allocated for variable sizes

  if (deserializeJson(doc, payload.c_str()) != DeserializationError::Ok)
  {
    debugPrint("Parsing failed for getDomeShutterState");
    return 4;
  }

  return doc["Value"];
}

bool getTelescopeAlpacaParkedState()
{
  String payload = sendRequest("GET", "/api/v1/telescope/0/atpark", "", telescopeAlpacaIP, telescopeAlpacaPort);

  DynamicJsonDocument doc(1024); // Heap-allocated for variable sizes

  if (deserializeJson(doc, payload.c_str()) != DeserializationError::Ok)
  {
    debugPrint("Parsing failed for getTelescopeAlpacaParkedState");
    return false;
  }

  return doc["Value"];
}

bool getTelescopeDirectParkedState()
{
  // TCP/IP connection to the telescope
  String response = simpleNetcat(telescopeDirectIP, telescopeDirectPort, ":Gstat#", 500);

  // Parse the response
  // Returns: a number depending on the current status of the mount, terminated by a '#',
  // with the following meaning:
  // 0# The mount is tracking.
  // 1# The mount is stopped after the pressing of the STOP key,
  // receiving the :STOP# command or completing an homing
  // sequence. Note that up to firmware version 2.12.21 included, the
  // status will be returned as 1# just after the STOP key is pressed or
  // the :STOP# command is received, even if the mount hasn't
  // stopped yet. Since firmware version 2.12.22, this status will be
  // returned only if the motors are actually stopped.
  // 2# The mount is slewing to the park position.
  // 3# The mount is unparking.
  // 4# The mount is slewing to the home position.
  // 5# The mount is parked.
  // 6# The mount is slewing, or the mount is going to stop (but still
  // moving) after the STOP key has been pressed or the :STOP#
  // command has been received.
  // 7# Tracking is off and the mount is not moving.
  // 8# The motors are inhibited because of low temperature (only for
  // special-purpose mounts with temperature sensors) , or the motors
  // are subject to pre-heating before startup.
  // 9# Tracking is on but the mount is outside tracking limits.
  // 10# The mount is following a precalulated satellite trajectory.
  // 11# The mount needs an user intervention to authorize movement, due
  // to a suspected inconsistency in data (see :USEROK# command);
  // if this occurs, DO NOT assume anything about the correctness of
  // the mount position or alignment data.
  // 98# Unknown status.
  // 99# Error.

  // Extract the status
  int status = response.substring(0, response.indexOf("#")).toInt();

  // Check if the mount is parked
  return status == 5;
}

bool getSafetyMonitorState()
{
  String payload = sendRequest("GET", "/api/v1/safetymonitor/0/issafe", "", safetyMonitorAlpacaIP, safetyMonitorAlpacaPort);

  DynamicJsonDocument doc(1024); // Heap-allocated for variable sizes

  if (deserializeJson(doc, payload.c_str()) != DeserializationError::Ok)
  {
    debugPrint("Parsing failed for getSafetyMonitorState");
    return false;
  }

  return doc["Value"];
}

void closeObservatory()
{
  // Park the telescope if it is not already parked
  if (getTelescopeDirectParkedState() == false)
  {
    abortTelescopeMovement();
    parkTelescope();
  }

  if (getTelescopeDirectParkedState() == true && getDomeShutterState() != 1)
  {
    // Close the dome
    closeDome();
  }
}

void parkTelescope()
{
  // Park the telescope
  String response = simpleNetcat(telescopeDirectIP, telescopeDirectPort, ":KA#", 500);

  // Check if the telescope is parked in while loop, timeout after 180 seconds
  unsigned long startTime = millis();
  while (getTelescopeDirectParkedState() == false)
  {
    if (millis() - startTime > 180000)
    {
      break;
    }
    rtos::ThisThread::sleep_for(1000);
  }
}

void closeDome()
{
  // Close the dome
  String payload = sendRequest("PUT", "/api/v1/dome/0/closeshutter", "", domeAlpacaIP, domeAlpacaPort);

  // Check if the dome is closed in while loop, timeout after 180 seconds
  unsigned long startTime = millis();
  while (getDomeShutterState() != 1)
  {
    if (millis() - startTime > 180000)
    {
      break;
    }
    rtos::ThisThread::sleep_for(1000);
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
  String response = simpleNetcat(telescopeDirectIP, telescopeDirectPort, ":STOP#", 500);
}

// Function to forward a request to the target device
void forwardRequest(Request &req, Response &res)
{
  // Extract path and method
  const char *path = req.path();

  // Get HTTP method as string
  const char *methodStr;
  switch (req.method())
  {
  case Request::GET:
    methodStr = "GET";
    break;
  case Request::PUT:
    methodStr = "PUT";
    break;
  default:
    methodStr = "GET";
  }

  res.set("Content-Type", "application/json");

  if (strstr(path, "connected") == NULL)
  {
    if (((blockDome && strstr(path, "dome") != NULL) ||
         (blockTelescope && strstr(path, "telescope") != NULL)) &&
        (methodStr == "PUT"))
    {
      res.status(403); // Forbidden
      res.print("{\"error\":\"Device is currently in use\"}");
      return;
    }
  }

  IPAddress targetIP;
  int targetPort;

  if (strstr(path, "dome") != NULL)
  {
    targetIP = domeAlpacaIP;
    targetPort = domeAlpacaPort;
  }
  else if (strstr(path, "telescope") != NULL)
  {
    targetIP = telescopeAlpacaIP;
    targetPort = telescopeAlpacaPort;
  }
  else if (strstr(path, "safetymonitor") != NULL)
  {
    targetIP = safetyMonitorAlpacaIP;
    targetPort = safetyMonitorAlpacaPort;
  }
  else
  {
    res.status(404); // Not Found
    res.print("{\"error\":\"Device not found\"}");
    return;
  }

  // Read the request body
  String requestBuffer = req.readString();

  String body = sendRequest(methodStr, path, requestBuffer, targetIP, targetPort);

  res.print(body.c_str());
}

// Function to send a request to a server and receive a response
String sendRequest(const char *methodStr, const char *path, String requestBuffer, IPAddress targetServerIP, int targetServerPort)
{
  if (requestHappening)
  {
    debugPrint("Request already happening");
    while (requestHappening)
    {
      rtos::ThisThread::sleep_for(100);
    }
  }
  else
  {
    requestHappening = true;
  }
  debugPrint("Request method: ", methodStr);
  debugPrint("Request path: ", path);
  debugPrint("Request body: ", requestBuffer.c_str());

  // Create Ethernet client
  EthernetClient ethClient;

  // Attempt to connect to the target server
  if (ethClient.connect(targetServerIP, targetServerPort))
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
    debugPrint("Connection to target server failed");
    requestHappening = false;
    // res.status(502); // Bad Gateway
    // res.print("{\"error\":\"Unable to connect to target server\"}");
    return "";
  }

  String header = "";
  String body = "";
  bool isBody = false;

  while (ethClient.connected())
  {
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

  requestHappening = false;

  return body;
}

// Function to send data to a server and receive response
String simpleNetcat(IPAddress host, int port, String content, int sleep_ms)
{
  String response = "";
  EthernetClient netCatClient;

  // Try to connect to the server
  if (netCatClient.connect(host, port))
  {
    // Send the content
    netCatClient.print(content);

    // Wait before closing the connection
    delay(sleep_ms);

    // Read the response
    while (netCatClient.available())
    {
      char c = netCatClient.read();
      response += c;

      // You can uncomment for debugging
      // Serial.print(c);
    }

    // Close the connection
    netCatClient.stop();
  }
  else
  {
    // Connection failed
    Serial.println("Connection failed");
  }

  return response;
}

void setup()
{
  if (DEBUG)
  {
    Serial.begin(9600);
    while (!Serial)
    {
      ; // Wait for serial port to connect
    }
  }

  // set pins for LEDs
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize Ethernet connection
  Ethernet.begin(mac, ip, dns, gateway, subnet);
  ;

  debugPrint("Server started at ", Ethernet.localIP());

  // start ReadStatesThread thread
  ReadStatesThread.start(readStates);

  // Set up aWOT routes
  app.use(&forwardRequest);

  // Start the server
  server.begin();
  debugPrint("Proxy server ready!");
}

void loop()
{
  // Handle incoming client connections
  EthernetClient client = server.available();

  if (client.connected())
  {
    debugPrint("Client connected");
    app.process(&client);
    client.stop();
  }
}