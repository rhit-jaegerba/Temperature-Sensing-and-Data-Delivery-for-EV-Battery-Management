#include <Wire.h>
#include <Wire1.h>

#define TMP102_ADDRESS 0x48
#define TEMPERATURE_REGISTER 0x00
#define CONFIG_REGISTER 0x01
#define T_LOW_REGISTER 0x02
#define T_HIGH_REGISTER 0x03
#define ALERT_PIN A3
#define SLAVE_ADDRESS 0x20

void setup() {
  Serial.begin(9600);
  Wire.begin();
  pinMode(ALERT_PIN, INPUT);

  if (!initializeTMP102()) {
    Serial.println("Cannot connect to TMP102. Check wiring and address.");
    while (1);
  }
  // Initialize the secondary I2C bus as a Slave
  Wire1.begin(SLAVE_ADDRESS);
  Wire1.onReceive(receiveEvent); // Callback for data received
  Wire1.onRequest(requestEvent); // Callback for data requested
  Serial.println("Slave I2C Initialized");
  configureSensor();
}

void loop() {
  float temperatureC = readTempC();
  float temperatureF = temperatureC * 9.0 / 5.0 + 32.0;
  
  //Serial.print("Temperature: ");
  //Serial.print(temperatureC);
  //Serial.print("°C / ");
  Serial.println(temperatureF);
  //Serial.print("°F");

  int alertPinState = digitalRead(ALERT_PIN);
  //Serial.print("\tAlert Pin State: ");
  //Serial.println(alertPinState);

  delay(100);
}

bool initializeTMP102() {
  Wire.beginTransmission(TMP102_ADDRESS);
  return Wire.endTransmission() == 0;
}

void configureSensor() {
  setConversionRate(2);        // Set conversion rate to 4 Hz
  setExtendedMode(true);      // Standard mode (-55°C to 128°C)
  setAlertPolarity(true);      // Active HIGH alert
  setAlertMode(false);         // Comparator mode
  setFaultQueue(0);            // Trigger alert on first fault
  setHighTempF(85.0);          // Set T_HIGH threshold to 85°F
  setLowTempF(84.0);           // Set T_LOW threshold to 84°F
}

float readTempC() {
  Wire.beginTransmission(TMP102_ADDRESS);
  Wire.write(TEMPERATURE_REGISTER);
  Wire.endTransmission();
  Wire.requestFrom(TMP102_ADDRESS, 2);

  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  
  int temp = (msb << 5) | (lsb >> 3); // 13-bit mode (left-justified)
  if (temp & 0x1000) temp |= 0xE000; // Sign extend negative numbers
  
  return temp * 0.0625;
}

void setHighTempF(float temperatureF) {
  setTempRegister(T_HIGH_REGISTER, (temperatureF - 32) * 5 / 9);
}

void setLowTempF(float temperatureF) {
  setTempRegister(T_LOW_REGISTER, (temperatureF - 32) * 5 / 9);
}

void setTempRegister(uint8_t reg, float temperatureC) {
  int16_t temp = temperatureC / 0.0625;
  uint8_t msb = (temp >> 4) & 0xFF;
  uint8_t lsb = (temp << 4) & 0xF0;
  
  Wire.beginTransmission(TMP102_ADDRESS);
  Wire.write(reg);
  Wire.write(msb);
  Wire.write(lsb);
  Wire.endTransmission();
}

void setConversionRate(uint8_t rate) {
  rate = (rate & 0x03) << 6;
  uint16_t config = readConfigRegister();
  config = (config & 0x3FFF) | rate;

  writeConfigRegister(config);
}

void setExtendedMode(bool mode) {
  uint16_t config = readConfigRegister();
  config = mode ? (config | 0x0010) : (config & 0xFFEF);

  writeConfigRegister(config);
}

void setAlertPolarity(bool polarity) {
  uint16_t config = readConfigRegister();
  config = polarity ? (config | 0x0400) : (config & 0xFBFF);

  writeConfigRegister(config);
}

void setAlertMode(bool mode) {
  uint16_t config = readConfigRegister();
  config = mode ? (config | 0x0200) : (config & 0xFDFF);

  writeConfigRegister(config);
}

void setFaultQueue(uint8_t faults) {
  faults = (faults & 0x03) << 3;
  uint16_t config = readConfigRegister();
  config = (config & 0xFFF7) | faults;

  writeConfigRegister(config);
}

uint16_t readConfigRegister() {
  Wire.beginTransmission(TMP102_ADDRESS);
  Wire.write(CONFIG_REGISTER);
  Wire.endTransmission();
  Wire.requestFrom(TMP102_ADDRESS, 2);

  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  return (msb << 8) | lsb;
}

void writeConfigRegister(uint16_t config) {
  Wire.beginTransmission(TMP102_ADDRESS);
  Wire.write(CONFIG_REGISTER);
  Wire.write(config >> 8);
  Wire.write(config & 0xFF);
  Wire.endTransmission();
}
// Slave receive event
void receiveEvent(int numBytes) {
  Serial.print("Slave Received: ");
  while (Wire1.available()) {
    char c = Wire1.read();
    Serial.print(c); // Print received byte
  }
  Serial.println();
}

// Slave request event
void requestEvent() {
  Serial.println("Here");
  Wire1.write(1); // Send a response to the master
}
