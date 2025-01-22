#include <Wire.h>

#define TMP102_ADDRESS 0x48
#define TEMPERATURE_REGISTER 0x00
#define CONFIG_REGISTER 0x01
#define T_LOW_REGISTER 0x02
#define T_HIGH_REGISTER 0x03
#define ALERT_PIN A3
uint16_t temperatureC;
const int I2C_SLAVE_ADDRESS = 0x08;  // Define the I2C address of this slave device

void setup() {
  Serial.begin(9600);                // Initialize Serial communication for printing
  Wire.begin(I2C_SLAVE_ADDRESS);     // Join I2C bus as a slave with the given address
  Wire.onReceive(receiveEvent);      // Register the receive event handler
  Wire.onRequest(requestEvent);      // Register the request event handler (optional, in case the master requests data)
  
  Serial.println("I2C Slave Initialized");
  pinMode(SDA, INPUT);  // Set the pin as an input without pull-up or pull-down resistor
  pinMode(SCL, INPUT);  // Set the pin as an input without pull-up or pull-down resistor
  //Wire.setClock(400000);
    pinMode(ALERT_PIN, INPUT);

  if (!initializeTMP102()) {
    Serial.println("Cannot connect to TMP102. Check wiring and address.");
    while (1);
  }
  
  configureSensor();
}

void loop() {
  Serial.print("Test");
  temperatureC = readTempC();
  float temperatureF = temperatureC * 9.0 / 5.0 + 32.0;
  
  Serial.print("Temperature: ");
  Serial.print(temperatureC);
  Serial.print("°C / ");
  Serial.print(temperatureF);
  Serial.println("°F");

  int alertPinState = digitalRead(ALERT_PIN);
  //Serial.print("\tAlert Pin State: ");
  //Serial.println(alertPinState);
  delay(250);
}

// Function to handle data received from the I2C master
void receiveEvent(int numBytes) {
  Serial.println("Packet Received");
  Serial.println(numBytes);
  if (numBytes > 0) {
    while (Wire.available()) {
      byte receivedData = Wire.read();   // Read one byte of incoming data
      Serial.print("Received from master: ");
      Serial.println(receivedData, HEX);  // Print in HEX format for better clarity
    }
    
    // Respond with 0x24 to the master
    //Wire.write(0x24);  // Send 0x24 back to the master
    Wire.write(0x21);
    //Serial.println("Responding with: 0x24");
  }
  else {
    //Serial.println("No data received");
    //Wire.write(0x24);
    Wire.write(0x21);
  }
  //Serial.println("");
  
}

// Function to handle requests from the I2C master (optional, if the master requests data from the slave)
void requestEvent() {
  // Send the same response (0x24) when the master requests data
  //uint16_t test = 0x25;
  Wire.write(temperatureC);  // Send 0x24 back to the master

  //Serial.println("Sending 0x24 to master upon request");
}


bool initializeTMP102() {
  Wire.beginTransmission(TMP102_ADDRESS);
  return Wire.endTransmission() == 0;
}

void configureSensor() {
  setConversionRate(2);        // Set conversion rate to 4 Hz
  setExtendedMode(false);      // Standard mode (-55°C to 128°C)
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
  
  int16_t temp = (msb << 4) | (lsb >> 4);
  if (temp & 0x800) temp |= 0xF000; // Handle negative temperatures
  
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
