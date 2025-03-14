#include <WiFi.h>
#include <WebServer.h>
#include "OLED-Display-SOLDERED.h"

const char index_html[] PROGMEM = R"rawliteral(
<html>
<head>
    <script>
        let depth = 256;
        let distance = 1000;
        let start = 80;
        let maxx = 64;
        let ctx;
        let data;
        let p;
        let xs;
        let ys;

        function generateMockData(objectDistance) {
            let arrayData = new Array(maxx);
            let normalizedDepth = Math.floor((objectDistance - 2) / (400 - 2) * (depth - 1));

            for (let x = 0; x < maxx; x++) {
                let a = new Array(depth).fill(0);
                for (let i = 0; i < depth; i++) {
                    let noise = Math.random() * 50;
                    let fading = Math.exp(-i / depth * 3) * 80;
                    let intensity = Math.floor(noise + fading);

                    if (Math.abs(i - normalizedDepth) <= 3 && Math.random() < 0.3) {
                        intensity = 255 - Math.random() * 30;
                    } else if (Math.abs(i - normalizedDepth) <= 7 && Math.random() < 0.1) {
                        intensity = 150 + Math.random() * 50;
                    }

                    a[i] = Math.max(0, Math.min(255, intensity));
                }
                arrayData[x] = a;
            }
            return arrayData;
        }

        function renderData(arrayData) {
            const distanceR = distance / 2;
            const startR = start / 2;
            const totalR = startR + distanceR;
            const mm2pix = ys / totalR;
            const data2mm = distanceR / depth;
            const data2pix = data2mm * mm2pix;
            const startPix = startR * mm2pix;

            let x0 = xs * 0.5;
            let y0 = ys - 1;

            ctx.clearRect(0, 0, xs, ys);

            for (let y = 0; y < ys; y++) {
                for (let x = 0; x < xs; x++) {
                    let dx = x - x0;
                    let dy = y - y0;
                    let d = Math.sqrt(dx * dx + dy * dy);
                    let angleIndex = ((Math.atan2(dy, dx) + Math.PI * 0.5) / (Math.PI / 180 * 60) + 0.5) * maxx;
                    let angleLow = Math.floor(angleIndex);
                    let angleHigh = Math.min(angleLow + 1, maxx - 1);
                    let angleWeight = angleIndex - angleLow;

                    let depthIndex = (d - startPix) / data2pix;
                    let depthLow = Math.floor(depthIndex);
                    let depthHigh = Math.min(depthLow + 1, depth - 1);
                    let depthWeight = depthIndex - depthLow;

                    let v = 0;
                    let alpha = 0;

                    if (angleLow >= 0 && angleLow < maxx && depthLow >= 0 && depthLow < depth) {
                        let v00 = arrayData[angleLow][depthLow];
                        let v10 = arrayData[angleHigh][depthLow];
                        let v01 = arrayData[angleLow][depthHigh];
                        let v11 = arrayData[angleHigh][depthHigh];

                        v = (v00 * (1 - angleWeight) + v10 * angleWeight) * (1 - depthWeight) +
                            (v01 * (1 - angleWeight) + v11 * angleWeight) * depthWeight;

                        alpha = Math.max(0, 255 - (depthIndex / depth) * 255);
                    }

                    let pixelIndex = (xs * y + x) * 4;
                    p[pixelIndex + 0] = v;
                    p[pixelIndex + 1] = v;
                    p[pixelIndex + 2] = v;
                    p[pixelIndex + 3] = alpha;
                }
            }
            ctx.putImageData(data, 0, 0);
        }

        function startRendering() {
            fetch('/sensor_data')
                .then(response => response.text())
                .then(distance => {
                    let arrayData = generateMockData(parseInt(distance));
                    renderData(arrayData);
                    requestAnimationFrame(startRendering);
                })
                .catch(error => {
                    console.error('Error fetching sensor data:', error);
                    // Optionally, use a default value or handle the error gracefully
                    let arrayData = generateMockData(200);
                    renderData(arrayData);
                    requestAnimationFrame(startRendering);
                });
        }

        function saveImage() {
            const canvas = document.getElementById("scan");
            const image = canvas.toDataURL("image/png").replace("image/png", "image/octet-stream");
            const link = document.createElement('a');
            link.download = "sonar_img.png";
            link.href = image;
            link.click();
        }

        window.onload = function() {
            const canvas = document.getElementById("scan");
            ctx = canvas.getContext("2d");
            xs = canvas.width;
            ys = canvas.height;
            data = ctx.getImageData(0, 0, xs, ys);
            p = data.data;
            startRendering();
        };
    </script>
</head>
<body style="font-family: Arial, Helvetica, sans-serif; background-color: #001;">
    <center>
        <canvas id="scan" width="512" height="512" style="zoom: 1.5; border-style: solid; border-color: #111;"></canvas>
        <br><br>
        <button onclick="startRendering()">Restart Scan</button>
        <button onclick="saveImage()">Save Image</button>
    </center>
</body>
</html>
)rawliteral";

OLED_Display display;

const char* ssid = "Sonogrid";
const char* password = "DavidNwafa";

// Ultrasonic sensor pins
const int trigPin = 14;
const int echoPin = 27;

WebServer server(80);

void setup() {
  // Serial.begin(115200);
  if (!display.begin())
  {
    // Serial.println("Display init failed!");
    ;
  }
  display.display();
  delay(3000);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  delay(10);

  // We start by connecting to a WiFi network
  if (!WiFi.softAP(ssid, password)){
      ;
    }
  IPAddress IP = WiFi.softAPIP();
  String apURL = "http://" + IP.toString() + "/";

  // Serial.println();
  // Serial.println();
  // Serial.print("Device Url: ");
  // Serial.println(apURL);

  // Serve the HTML page
  server.on("/", []() {
    server.send(200, "text/html", index_html);
  });

  // Serve sensor data
  server.on("/sensor_data", []() {
    long duration, distance;
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance = (duration * 0.034 / 2); // Calculate distance in cm
    server.send(200, "text/plain", String(distance));
  });

  server.begin();
  display.clearDisplay();

  display.setTextSize(1);              // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);             // Start at top-left corner
  display.println(F(apURL.c_str()));
  display.setCursor(0, 20);
  display.print(F("SSID: Sonogrid"));
  display.setCursor(0, 40);
  display.println(F("Password: DavidNwafa"));
  display.display();
}

void loop() {
  server.handleClient();
}