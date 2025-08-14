/*
 * ESP32 receive the CAM video and control the motor 
*/
#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "****";
const char* password = "******";

// ESP32-CAM stream URL (must be /stream)
const char* cam_url = "http://192.168.1.200/stream";

// Motor control pins
#define IN1 14
#define IN2 27
#define IN3 26
#define IN4 25

WebServer server(80);

// HTML UI (with auto-reload video)
const char MAIN_page[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<title>ESP32 Rover Control</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  body { font-family: Arial; text-align: center; margin:0; background-color: #222; color:white;}
  #videoStream { width: 100%; height: 50vh; object-fit: cover; background:black; }
  .btn {
    width: 100px; height: 50px; margin: 10px; font-size: 16px;
    background-color: #4CAF50; color: white; border: none; border-radius: 8px;
  }
  .btn:active { background-color: #45a049; }
</style>
</head>
<body>
<h2>ESP32 Rover Control</h2>
<img id="videoStream" src="%CAM_URL%" onerror="reloadStream()">
<div>
  <button class="btn" onclick="fetch('/forward')">Forward</button><br>
  <button class="btn" onclick="fetch('/left')">Left</button>
  <button class="btn" onclick="fetch('/stop')">Stop</button>
  <button class="btn" onclick="fetch('/right')">Right</button><br>
  <button class="btn" onclick="fetch('/backward')">Backward</button>
</div>
<script>
function reloadStream(){
  const img = document.getElementById('videoStream');
  img.src = '%CAM_URL%' + '?t=' + new Date().getTime();
}
setInterval(reloadStream, 5000);
</script>
</body>
</html>
)rawliteral";

// ===== Motor control functions =====
void forward() {
  Serial.println("Forward");
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}
void backward() {
  Serial.println("Backward");
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}
void left() {
  Serial.println("Left");
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}
void right() {
  Serial.println("Right");
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}
void stopMotor() {
  Serial.println("Stop");
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void setup() {
  Serial.begin(115200);

  // Motor pin setup
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopMotor();

  // Wi-Fi connection
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println(WiFi.localIP());

  // Prepare HTML page
  String page = MAIN_page;
  page.replace("%CAM_URL%", cam_url);

  // Routes
  server.on("/", [page]() { server.send(200, "text/html", page); });
  server.on("/forward", [](){ forward(); server.send(200,"text/plain","Forward"); });
  server.on("/backward", [](){ backward(); server.send(200,"text/plain","Backward"); });
  server.on("/left", [](){ left(); server.send(200,"text/plain","Left"); });
  server.on("/right", [](){ right(); server.send(200,"text/plain","Right"); });
  server.on("/stop", [](){ stopMotor(); server.send(200,"text/plain","Stop"); });

  server.begin();
}

void loop() {
  server.handleClient();
}
