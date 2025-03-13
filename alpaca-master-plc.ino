#include <Ethernet.h>
#include <aWOT.h>

// Network configuration for your Arduino
byte mac[] = {0xA8, 0x61, 0x0A, 0x50, 0x9D, 0x95};
IPAddress ip(192, 168, 1, 177);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(8, 8, 8, 8);

// Target server configuration
IPAddress targetServerIP(192, 168, 1, 14); // Change to your target device's IP
int targetServerPort = 80;                 // Change to your target device's port

// Initialize the Ethernet server and aWOT application
EthernetServer server(80);
Application app;

// Create Ethernet client for HttpClient to use
EthernetClient ethClient;
// Buffer for payload data
String requestBuffer;

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
  requestBuffer = req.readString();

  debugPrint("Request method: ", methodStr);
  debugPrint("Request path: ", path);
  debugPrint("Request body: ", requestBuffer.c_str());

  if (ethClient.connect(targetServerIP, targetServerPort))
  {
    debugPrint("Connected to target server");

    // Send the request line
    ethClient.print(methodStr);
    ethClient.print(" ");
    ethClient.print(path);

    // For GET requests, append parameters to the URL if they exist
    if (req.method() == Request::GET && requestBuffer.length() > 0)
    {
      ethClient.print("?");
      ethClient.print(requestBuffer);
    }

    ethClient.println(" HTTP/1.0"); // Use HTTP/1.1 instead of 1.0

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
    if ((req.method() == Request::PUT) &&
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
    res.status(502); // Bad Gateway
    res.print("{\"error\":\"Unable to connect to target server\"}");
    return;
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
  res.set("Content-Type", "application/json");
  res.print(body.c_str());

  debugPrint("Header:", header);
  debugPrint("Body:", body);

  ethClient.stop();
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

  // Initialize Ethernet connection
  Ethernet.begin(mac, ip, dns, gateway, subnet);
  ;

  if (DEBUG)
  {
    Serial.print("Server started at ");
    Serial.println(Ethernet.localIP());
  }

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
    app.process(&client);
    client.stop();
  }
}