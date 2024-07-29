#include <Arduino.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <SingleFileDrive.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>

#include <vector>

#include "byteutils.h"
#include "config.h"
#include "imu.h"
#include "robot.h"
#include "wpilibudp.h" 
#include "encoder.h"

// Resource strings
extern "C" {
const unsigned char* GetResource_index_html(size_t* len);
const unsigned char* GetResource_normalize_css(size_t* len);
const unsigned char* GetResource_skeleton_css(size_t* len);
const unsigned char* GetResource_xrp_js(size_t* len);
const unsigned char* GetResource_VERSION(size_t* len);
}

char chipID[20];
char DEFAULT_SSID[32];

XRPConfiguration config;
NetworkMode netConfigResult;

// HTTP server
WebServer webServer(5000);

// UDP
WiFiUDP udp;
char udpPacketBuf[UDP_TX_PACKET_MAX_SIZE + 1];
IPAddress udpRemoteAddr;
uint16_t udpRemotePort;

// std::vector<std::string> outboundMessages;

// TEMP: Status
unsigned long _wsMessageCount = 0;
unsigned long _lastMessageStatusPrint = 0;
int _baselineUsedHeap = 0;

unsigned long _avgLoopTimeUs = 0;
unsigned long _loopTimeMeasurementCount = 0;

uint16_t seq = 0;

// Generate the status text file
void writeStatusToDisk() {
  File f = LittleFS.open("/status.txt", "w");
  size_t len;
  std::string versionString{reinterpret_cast<const char*>(GetResource_VERSION(&len)), len};
  f.printf("Version: %s\n", versionString.c_str());
  f.printf("Chip ID: %s\n", chipID);
  f.printf("WiFi Mode: %s\n", netConfigResult == NetworkMode::AP ? "AP" : "STA");
  if (netConfigResult == NetworkMode::AP) {
    f.printf("AP SSID: %s\n", config.networkConfig.defaultAPName.c_str());
    f.printf("AP PASS: %s\n", config.networkConfig.defaultAPPassword.c_str());
  }
  else {
    f.printf("Connected to %s\n", WiFi.SSID().c_str());
  }

  f.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());
  f.close();
}

// ==================================================
// UDP Management Functions
// ==================================================

// Update the remote UDP socket information (used to send data upstream)
void updateRemoteInfo() {
  // Update the remote address if needed
  if (!udpRemoteAddr.isSet()) {
    Serial.printf("[NET] Received first UDP connect from %s:%d\n", udp.remoteIP().toString().c_str(), udp.remotePort());
    udpRemoteAddr = udp.remoteIP();
    udpRemotePort = udp.remotePort();
  }
  else {
    bool shouldUpdate = false;
    if (udpRemoteAddr != udp.remoteIP()) {
      shouldUpdate = true;
    }
    if (udpRemotePort != udp.remotePort()) {
      shouldUpdate = true;
    }

    if (shouldUpdate) {
      udpRemoteAddr = udp.remoteIP();
      udpRemotePort = udp.remotePort();
    }
  }
}

uint dutycycleencoder[4];

void sendData() {
  int size = 0;
  char buffer[512];
  int ptr = 0;

  uint16ToNetwork(seq, buffer);
  buffer[2] = 0; // Unset the control byte
  ptr = 3;

  // Encoders
  for (int i = 0; i < 4; i++) {
    int encoderValue = xrp::readEncoderRaw(i);
    uint encoderPeriod = xrp::readEncoderPeriod(i);

    // We want to flip the encoder 0 value (left motor encoder) so that this returns
    // positive values when moving forward.
    if (i == 0) {
      encoderValue = -encoderValue;
      encoderPeriod ^= 1; //Last bit is direction bit; Flip it.
    }

    static constexpr uint divisor = xrp::Encoder::getDivisor();

    ptr += wpilibudp::writeEncoderData(i, encoderValue, encoderPeriod, divisor, buffer, ptr);
    ptr += wpilibudp::writeDutyCycleEncoderData(i, dutycycleencoder[i], buffer, ptr);
  } // 4x 15 bytes

  // DIO (currently just the button)
  ptr += wpilibudp::writeDIOData(0, xrp::isUserButtonPressed(), buffer, ptr);
  // 1x 4 bytes

  // Gyro and accel data
  float gyroRates[3] = {
    xrp::imuGetGyroRateX(),
    xrp::imuGetGyroRateY(),
    xrp::imuGetGyroRateZ()
  };

  float gyroAngles[3] = {
    xrp::imuGetRoll(),
    xrp::imuGetPitch(),
    xrp::imuGetYaw()
  };

  float accels[3] = {
    xrp::imuGetAccelX(),
    xrp::imuGetAccelY(),
    xrp::imuGetAccelZ()
  };

  ptr += wpilibudp::writeGyroData(gyroRates, gyroAngles, buffer, ptr);
  // 1x 26 bytes
  ptr += wpilibudp::writeAccelData(accels, buffer, ptr);
  // 1x 14 bytes

  if (xrp::reflectanceInitialized()) {
    ptr += wpilibudp::writeAnalogData(0, xrp::getReflectanceLeft5V(), buffer, ptr);
    ptr += wpilibudp::writeAnalogData(1, xrp::getReflectanceRight5V(), buffer, ptr);
  }

  if (xrp::rangefinderInitialized()) {
    ptr += wpilibudp::writeAnalogData(2, xrp::getRangefinderDistance5V(), buffer, ptr);
  }

  if( xrp::batteryInitialized()){
    ptr += wpilibudp::writeAnalogData(3, xrp::getBatteryVoltange(), buffer, ptr);
  }
  // ptr should now point to 1 past the last byte
  size = ptr;

  // Send
  if (udpRemoteAddr.isSet()) {
    udp.beginPacket(udpRemoteAddr.toString().c_str(), udpRemotePort);
    udp.write(buffer, size);
    udp.endPacket();
    seq++;
  }
}

// ==================================================
// Web Server Management Functions
// ==================================================
void setupWebServerRoutes() {
  webServer.on("/", []() {
    size_t len;
    webServer.send(200, "text/html", GetResource_index_html(&len), len);
  });

  webServer.on("/normalize.css", []() {
    size_t len;
    webServer.send(200, "text/css", GetResource_normalize_css(&len), len);
  });

  webServer.on("/skeleton.css", []() {
    size_t len;
    webServer.send(200, "text/css", GetResource_skeleton_css(&len), len);
  });

  webServer.on("/xrp.js", []() {
    size_t len;
    webServer.send(200, "text/javascript", GetResource_xrp_js(&len), len);
  });

  webServer.on("/getconfig", []() {
    File f = LittleFS.open("/config.json", "r");
    if (webServer.streamFile(f, "text/json") != f.size()) {
      Serial.println("[WEB] Sent less data than expected for /getconfig");
    }
    f.close();
  });

  webServer.on("/resetconfig", []() {
    if (webServer.method() != HTTP_POST) {
      webServer.send(405, "text/plain", "Method Not Allowed");
      return;
    }
    File f = LittleFS.open("/config.json", "w");
    f.print(generateDefaultConfig(DEFAULT_SSID).toJsonString().c_str());
    f.close();
    webServer.send(200, "text/plain", "OK");
  });

  webServer.on("/saveconfig", []() {
    if (webServer.method() != HTTP_POST) {
      webServer.send(405, "text/plain", "Method Not Allowed");
      return;
    }
    auto postBody = webServer.arg("plain");
    File f = LittleFS.open("/config.json", "w");
    f.print(postBody);
    f.close();
    Serial.println("[CONFIG] Configuration Updated Remotely");

    webServer.send(200, "text/plain", "OK");
  });
}

void checkPrintStatus() {
  if (millis() - _lastMessageStatusPrint > 5000) {

    int usedHeap = rp2040.getUsedHeap();
    Serial.printf("t(ms):%u h:%d msg:%u lt(us):%u w:%c\n", millis(), usedHeap, _wsMessageCount, _avgLoopTimeUs,
      wpilibudp::dsWatchdogActive()?'t':'f');
    _lastMessageStatusPrint = millis();
  }
}

void updateLoopTime(unsigned long loopStart) {
  unsigned long loopTime = micros() - loopStart;
  unsigned long totalTime = _avgLoopTimeUs * _loopTimeMeasurementCount;
  _loopTimeMeasurementCount++;

  _avgLoopTimeUs = (totalTime + loopTime) / _loopTimeMeasurementCount;
}


void setup() {
  // Generate the default SSID using the flash ID
  pico_unique_board_id_t id_out;
  pico_get_unique_board_id(&id_out);
  sprintf(chipID, "%02x%02x-%02x%02x", id_out.id[4], id_out.id[5], id_out.id[6], id_out.id[7]);
  sprintf(DEFAULT_SSID, "XRP-%s", chipID);

  Serial.begin(115200);
  LittleFS.begin();

  // Set up the I2C pins
  Wire1.setSCL(15); //19);
  Wire1.setSDA(14); //18);
  Wire1.begin();

  delay(2000);

  // Read Config
  config = loadConfiguration(DEFAULT_SSID);

  // Initialize IMU
  Serial.println("[IMU] Initializing IMU");
  xrp::imuInit(IMU_I2C_ADDR, &Wire1);

  Serial.println("[IMU] Beginning IMU calibration");
  xrp::imuCalibrate(5000);

  // Busy-loop if there's no WiFi hardware
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("[NET] No WiFi Module");
    while (true);
  }

  // Set up WiFi AP
  WiFi.setHostname(DEFAULT_SSID);

  // Use configuration information
  netConfigResult = configureNetwork(config);
  Serial.printf("[NET] Actual WiFi Mode: %s\n", netConfigResult == NetworkMode::AP ? "AP" : "STA");

  // Set up HTTP server routes
  Serial.println("[NET] Setting up Config webserver");
  setupWebServerRoutes();

  webServer.begin();
  Serial.println("[NET] Config webserver listening on *:5000");

  // Set up UDP
  udp.begin(3540);
  Serial.println("[NET] UDP socket listening on *:3540");

  Serial.println("[NET] Network Ready");
  Serial.printf("[NET] SSID: %s\n", WiFi.SSID().c_str());
  Serial.printf("[NET] IP: %s\n", WiFi.localIP().toString().c_str());

  xrp::robotInit();

  // NOTE: For now, we'll force init the reflectance sensor
  // TODO Enable this via configuration
  xrp::reflectanceInit();

  // NOTE: For now we'll force init the rangefinder
  // TODO enable this via configuration
  xrp::rangefinderInit();

  xrp::batteryInit();

  _lastMessageStatusPrint = millis();
  _baselineUsedHeap = rp2040.getUsedHeap();

  // Write current status file
  writeStatusToDisk();
  singleFileDrive.begin("status.txt", "XRP-Status.txt");
}

// drive: m1, turn: m2
void sendmotor(uint addr, float m1, float m2) {
  // -1->900; 0->1500; 1->2100
  byte v[4];
  uint vm = (m1 + 1.0) * 600 + 900;
  v[0] = vm & 0xff;
  v[1] = (vm>>8) & 0xff;
  vm = (m2 + 1.0) * 600 + 900;
  v[2] = vm & 0xff;
  v[3] = (vm>>8) & 0xff;
  Wire1.beginTransmission(addr);
  Wire1.write(v, 4);
  Wire1.endTransmission();
}

float speeds[8];
// front left, front right, back left, back right
// drive: 0, 2, 4, 6
// turn: 1, 3, 5, 7
void setmotor(int motor, double speed) {
  uint addr;
  float m1, m2;

  switch(motor){
  case 0:
  case 1:
    speeds[motor] = speed;
    m1 = speeds[0];
    m2 = speeds[1];
    addr = 16;
    break;
  case 2:
  case 3:
    speeds[motor] = speed;
    m1 = speeds[2];
    m2 = speeds[3];
    addr = 17;
    break;
  case 4:
  case 5:
    speeds[motor] = speed;
    m1 = speeds[4];
    m2 = speeds[5];
    addr = 18;
    break;
  case 6:
  case 7:
    speeds[motor] = speed;
    m1 = speeds[6];
    m2 = speeds[7];
    addr = 19;
    break;
  }
  sendmotor(addr, m1, m2);
}

uint net2host(byte *ptr){
  int u1 = ptr[0];
  u1 = u1&0x000000ff;
  int u2 = ptr[1];
  u2 = (u2<<8)&0x0000ff00;
  int u3 = ptr[2];
  u3 = (u3<<16)&0x00ff0000;
  int u4 = ptr[3];
  u4 = (u4<<24)&0xff000000;
  return u1+u2+u3+u4;
}

byte moduleaddr[4] = { 16, 17, 18, 19};

void encoderPeriodic() {
  byte buf[20];
  uint v;
  int i;
  for(int ind = 0; ind < 4; ind++){
    i = 0;
    Wire1.requestFrom(moduleaddr[ind], 20, true);
    while(Wire1.available()) {
      buf[i] = Wire1.read();
      if(i < 19)
        i++;
    }
    xrp::setencoderCount(ind, net2host(&buf[0]));
    xrp::setencoderPeriod(ind, net2host(&buf[4]));
    v = net2host(&buf[8]);
    dutycycleencoder[ind] = net2host(&buf[12]);
    v = net2host(&buf[16]);
  }
}

void loop() {
  unsigned long loopStartTime = micros();

  webServer.handleClient();

  int packetSize = udp.parsePacket();
  if (packetSize) {
    updateRemoteInfo();

    // Read the packet
    int n = udp.read(udpPacketBuf, UDP_TX_PACKET_MAX_SIZE);
    if(wpilibudp::processPacket(udpPacketBuf, n))
      _wsMessageCount++;
  }

  xrp::imuPeriodic();
  xrp::rangefinderPollForData();
  encoderPeriodic();

  // Disable the robot when the UDP watchdog timesout
  // Also reset the max sequence number so we can handle reconnects
  if (!wpilibudp::dsWatchdogActive()) {
    wpilibudp::resetState();
    xrp::robotSetEnabled(false);
    xrp::imuSetEnabled(false);
  }

  if (xrp::robotPeriodic()) {
    // Package up and send all the data
    sendData();
  }

  updateLoopTime(loopStartTime);
  checkPrintStatus();
}

void loop1() {
  if (xrp::rangefinderInitialized()) {
    xrp::rangefinderPeriodic();
  }

  delay(50);
}
