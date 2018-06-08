// This code will send all available data streams from the the BNO055 sensor
// through a wifi network via the Adafruit Feather Huzzah with ESP8266 wifi chip.
// The data will be recieved by Max/MSP on any computer connected to the wifi network.
//

// This code tests sending data streams from the BNO055 chip
// via wifi/OSC and also prints the data in the serial monitor


// libraries necessary for the position sensor to work
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

#define PIN 13
Adafruit_NeoPixel strip = Adafruit_NeoPixel(124, PIN, NEO_GRB + NEO_KHZ800);


//
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055();

// to be included for the wifi chip to work
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif
#include <WiFiUdp.h>
#include <OSCMessage.h>


// wifi network information, network name and network password
char ssid[] = "PaulSwift";
char pass[] = "thankyou";

// A UDP instance to let us send and receive packets over UDP
WiFiUDP Udp;

// IP address of computer recieving data over the wifi network
// multiple IP address may be included by added more IPAddress constants
const IPAddress outIp1(192, 168, 1, 6);
const IPAddress outIp2(192, 168, 1, 8);
const IPAddress outIp3(192, 168, 1, 10);
//const IPAddress outIp3();
//const IPAddress outIp4();
//const IPAddress outIp5();

// remote ports to recieve OSC messages
// a different port number is used for each type of vector data

const unsigned int portEuler = 7600;

/*not used for this project
  const unsigned int portEvent = 7500;
  const unsigned int portMag = 7700;
  const unsigned int portGyro = 7800;
  const unsigned int portAccel = 7900;
  const unsigned int portLinAccel = 8000;
  const unsigned int portGrav = 8100;
  const unsigned int portQuat = 8200;
  not used for this project
*/

// local port to listen for OSC packets (probably not necessary for this project)
const unsigned int localPort = 8888;


// variables for x,y,z data from each separate set of vectors

// Event (.getEvent for positional data
float eventX, eventY, eventZ;

// Euler vector (degrees 0-359)
float eulerX, eulerY, eulerZ;

// magnetometer vector ( uT, micro Teslas)
float magX, magY, magZ;

// Gyroscope vector (rps, radians per second)
float gyroX, gyroY, gyroZ;

// Accelerometer vector (m/s^2)
float accelX, accelY, accelZ;

// Linear accelerometer vector (m/s^2)
float linAccelX, linAccelY, linAccelZ;

// Gravity vector (m/s^2)
float gravX, gravY, gravZ;

// Quaternions
float quatW, quatX, quatY, quatZ;

float hue = 0;
float saturation = 100;
float value = 50;

void setup() {

  strip.begin();
  strip.show(); // Initialize all pixels to 'off'  `
  Serial.begin(115200);

  // Connect to WiFi network
  // The network information should be entered before the setup() function
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // This section is for recieving OSC on the board.
  // (not necessary for this project)
  /*
    Serial.println("Starting UDP");
    Udp.begin(localPort);
    Serial.print("Local port: ");
    #ifdef ESP32
    Serial.println(localPort);
    #else
    Serial.println(Udp.localPort());
    #endif
  */

  Serial.println("Orientation Sensor Raw Data Test");
  Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);
  bno.setExtCrystalUse(true);


  // for measuring the temperature near the chip
  /*
    // Display the current temperature
    int8_t temp = bno.getTemp();
    Serial.print("Current Temperature: ");
    Serial.print(temp);
    Serial.println(" C");
    Serial.println("");
  */



}

void loop() {
  sensors_event_t event; 
  bno.getEvent(&event);

  // BEGIN Euler vector messages
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  OSCMessage eulerX("/eulerX");
  eulerX.add(float(euler.x()));
  Udp.beginPacket(outIp1, portEuler);
  eulerX.send(Udp);
  Udp.endPacket();
  Udp.beginPacket(outIp2, portEuler);
  eulerX.send(Udp);
  Udp.endPacket();
  Udp.beginPacket(outIp3, portEuler);
  eulerX.send(Udp);
  Udp.endPacket();
  eulerX.empty();
  delay(2);

  OSCMessage eulerY("/eulerY");
  eulerY.add(float(euler.y()));
  Udp.beginPacket(outIp1, portEuler);
  eulerY.send(Udp);
  Udp.endPacket();
  Udp.beginPacket(outIp2, portEuler);
  eulerY.send(Udp);
  Udp.endPacket();
  Udp.beginPacket(outIp3, portEuler);
  eulerY.send(Udp);
  Udp.endPacket();
  eulerY.empty();
  delay(2);


  // YAW - shifts values to keep range from 0-359
  if (float(euler.z()) > 0) {
    eulerZ = float(euler.z());
  }
  if (float(euler.z()) < 0) {
    eulerZ = float(euler.z()) + 360;
  }

  //float hue = int(map(euler.x(), 0, 359, 1, 255));
  if (euler.x()){
    hue = float(euler.x());
    hueShift(hue);
  }
  //Serial.println(hue);


  OSCMessage eulerZ("/eulerZ");
  eulerZ.add(float(euler.z()));
  Udp.beginPacket(outIp1, portEuler);
  eulerZ.send(Udp);
  Udp.endPacket();
  Udp.beginPacket(outIp2, portEuler);
  eulerZ.send(Udp);
  Udp.endPacket();
  Udp.beginPacket(outIp3, portEuler);
  eulerZ.send(Udp);
  Udp.endPacket();
  eulerZ.empty();
  delay(2);

  // Display the floating point data in serial monitor

  //   Serial.print("eulerX: ");
  //   Serial.println(euler.x());
  //   Serial.print("eulerY: ");
  //   Serial.println(euler.y());
  //   Serial.print("eulerZ: ");
  //   Serial.println(eulerZ);
  //   Serial.print("\t\t");

  // END Euler vector messages




}
// change all leds to a new hue
void hueShift(float hue) {
  uint16_t i;
  byte r, g, b;

  HSV_to_RGB(hue, saturation , value, r, g, b);
  for (i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(r, g, b));
  }
  strip.show();
}



void HSV_to_RGB(float h, float s, float v, byte &r, byte &g, byte &b) {
  int i;
  float f, p, q, t;

  h = max(0.0f, min(360.0f, h));
  s = max(0.0f, min(100.0f, s));
  v = max(0.0f, min(100.0f, v));
  s /= 100.0f;
  v /= 100.0f;

  if (s == 0) {
    // Achromatic (grey)
    r = g = b = round(v * 255.0f);
    return;
  }

  h /= 60.0f; // sector 0 to 5
  i = floor(h);
  f = h - i; // factorial part of h
  p = v * (1.0f - s);
  q = v * (1.0f - s * f);
  t = v * (1.0f - s * (1.0f - f));
  switch (i) {
    case 0:
      r = round(255.0f * v);
      g = round(255.0f * t);
      b = round(255.0f * p);
      break;
    case 1:
      r = round(255.0f * q);
      g = round(255.0f * v);
      b = round(255.0f * p);
      break;
    case 2:
      r = round(255.0f * p);
      g = round(255.0f * v);
      b = round(255.0f * t);
      break;
    case 3:
      r = round(255.0f * p);
      g = round(255.0f * q);
      b = round(255.0f * v);
      break;
    case 4:
      r = round(255.0f * t);
      g = round(255.0f * p);
      b = round(255.0f * v);
      break;
    default: // case 5:
      r = round(255.0f * v);
      g = round(255.0f * p);
      b = round(255.0f * q);
  }
}

