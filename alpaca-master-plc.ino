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
IPAddress domeAlpacaIP(192, 168, 1, 7);
int domeAlpacaPort = 80;

IPAddress telescopeAlpacaIP(192, 168, 1, 7);
int telescopeAlpacaPort = 80;

IPAddress telescopeDirectIP(192, 168, 1, 7);
int telescopeDirectPort = 80;

IPAddress safetyMonitorAlpacaIP(192, 168, 1, 7);
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
    int domeShutterState = getDomeShutterState();
    Serial.print("Dome shutter state: ");
    Serial.println(domeShutterState);
    bool telescopeParked = getTelescopeParkedState();
    Serial.print("Telescope parked: ");
    Serial.println(telescopeParked);
    bool safetyMonitorState = getSafetyMonitorState();
    Serial.print("Safety monitor state: ");
    Serial.println(safetyMonitorState);

    rtos::ThisThread::sleep_for(500);
    digitalWrite(LED_BUILTIN, LOW);
    rtos::ThisThread::sleep_for(500);
  }
}

int getDomeShutterState()
{
  String payload = sendRequest("GET", "/api/v1/dome/0/shutterstatus", "", domeAlpacaIP, domeAlpacaPort);

  DynamicJsonDocument doc(1024); // Heap-allocated for variable sizes

  if (deserializeJson(doc, payload.c_str()) != DeserializationError::Ok)
  {
    debugPrint("Parsing failed for getDomeShutterState");
    return 4;
  }

  return doc["Value"];
}

bool getTelescopeParkedState()
{
  String payload = sendRequest("GET", "/api/v1/telescope/0/atpark", "", telescopeAlpacaIP, telescopeAlpacaPort);

  DynamicJsonDocument doc(1024); // Heap-allocated for variable sizes

  if (deserializeJson(doc, payload.c_str()) != DeserializationError::Ok)
  {
    debugPrint("Parsing failed for getTelescopeParkedState");
    return false;
  }

  return doc["Value"];
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
  // Close the dome
  blockDome = true;
  // Close the telescope
  blockTelescope = true;
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

  // Read the request body
  String requestBuffer = req.readString();

  String body = sendRequest(methodStr, path, requestBuffer, domeAlpacaIP, domeAlpacaPort);

  res.set("Content-Type", "application/json");
  res.print(body.c_str());
}

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