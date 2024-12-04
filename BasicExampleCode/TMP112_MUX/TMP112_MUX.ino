#include <Wire.h>

#include <SparkFun_I2C_Mux_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_I2C_Mux
QWIICMUX myMux;

#define TMP102_ADDRESS_1 0x49

#define TEMPERATURE_REGISTER 0x00
#define CONFIG_REGISTER 0x01
#define T_LOW_REGISTER 0x02
#define T_HIGH_REGISTER 0x03
#define ALERT_PIN A3

void setup() {
  Serial.begin(9600);
  Wire.begin();


  if (myMux.begin() == false)
  {
    Serial.println("Mux not detected. Freezing...");
    while (1)
      ;
  }
  Serial.println("Mux detected");

  myMux.setPort(0); //Connect master to port labeled '1' on the mux


  pinMode(ALERT_PIN, INPUT);

  // Initialize sensors
  if (!initializeTMP102(TMP102_ADDRESS_1)) {
    Serial.println("Cannot connect to " + String(TMP102_ADDRESS_1) + ". Check wiring and addresses.");
    while (1);
  }
  myMux.setPort(1);
  // Configure both sensors
  configureSensor(TMP102_ADDRESS_1);

    if (!initializeTMP102(TMP102_ADDRESS_1)) {
    Serial.println("Cannot connect to " + String(TMP102_ADDRESS_1) + ". Check wiring and addresses.");
    while (1);
  }

  // Configure both sensors
  configureSensor(TMP102_ADDRESS_1);
}
unsigned long previousMillis = 0;
void loop() {
  myMux.setPort(0);
  float P1temperatureC1 = readTempC(TMP102_ADDRESS_1);
  float P1temperatureF1 = P1temperatureC1 * 9.0 / 5.0 + 32.0;
  myMux.setPort(1);
  float P2temperatureC1 = readTempC(TMP102_ADDRESS_1);
  float P2temperatureF1 = P2temperatureC1 * 9.0 / 5.0 + 32.0;
  /*
  unsigned long currentMillis = millis();  // Capture the current time at the start of the loop

  // Calculate the time taken for the previous loop cycle
  unsigned long cycleTime = currentMillis - previousMillis;

  // Print the cycle time
  Serial.print("Cycle time: ");
  Serial.print(cycleTime);
  Serial.println(" ms");

  // Update previousMillis to the current time for the next loop cycle
  previousMillis = currentMillis;
  */
  Serial.print("P1:");
  Serial.print(P1temperatureF1);
  Serial.print("    P2:");
  Serial.println(P2temperatureF1);
  delay(250);
}

bool initializeTMP102(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

void configureSensor(uint8_t address) {
  setConversionRate(address, 2);        // Set conversion rate to 4 Hz
  setExtendedMode(address, false);      // Standard mode (-55°C to 128°C)
  setAlertPolarity(address, true);      // Active HIGH alert
  setAlertMode(address, false);         // Comparator mode
  setFaultQueue(address, 0);            // Trigger alert on first fault
  setHighTempF(address, 85.0);          // Set T_HIGH threshold to 85°F
  setLowTempF(address, 84.0);           // Set T_LOW threshold to 84°F
}

float readTempC(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(TEMPERATURE_REGISTER);
  Wire.endTransmission();
  Wire.requestFrom(address, (uint8_t)2);  // Request 2 bytes from the sensor

  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  
  int16_t temp = (msb << 4) | (lsb >> 4);
  if (temp & 0x800) temp |= 0xF000; // Handle negative temperatures
  
  return temp * 0.0625;
}

void setHighTempF(uint8_t address, float temperatureF) {
  setTempRegister(address, T_HIGH_REGISTER, (temperatureF - 32) * 5 / 9);
}

void setLowTempF(uint8_t address, float temperatureF) {
  setTempRegister(address, T_LOW_REGISTER, (temperatureF - 32) * 5 / 9);
}

void setTempRegister(uint8_t address, uint8_t reg, float temperatureC) {
  int16_t temp = temperatureC / 0.0625;
  uint8_t msb = (temp >> 4) & 0xFF;
  uint8_t lsb = (temp << 4) & 0xF0;
  
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(msb);
  Wire.write(lsb);
  Wire.endTransmission();
}

void setConversionRate(uint8_t address, uint8_t rate) {
  rate = (rate & 0x03) << 6;
  uint16_t config = readConfigRegister(address);
  config = (config & 0x3FFF) | rate;

  writeConfigRegister(address, config);
}

void setExtendedMode(uint8_t address, bool mode) {
  uint16_t config = readConfigRegister(address);
  config = mode ? (config | 0x0010) : (config & 0xFFEF);

  writeConfigRegister(address, config);
}

void setAlertPolarity(uint8_t address, bool polarity) {
  uint16_t config = readConfigRegister(address);
  config = polarity ? (config | 0x0400) : (config & 0xFBFF);

  writeConfigRegister(address, config);
}

void setAlertMode(uint8_t address, bool mode) {
  uint16_t config = readConfigRegister(address);
  config = mode ? (config | 0x0200) : (config & 0xFDFF);

  writeConfigRegister(address, config);
}

void setFaultQueue(uint8_t address, uint8_t faults) {
  faults = (faults & 0x03) << 3;
  uint16_t config = readConfigRegister(address);
  config = (config & 0xFFF7) | faults;

  writeConfigRegister(address, config);
}

uint16_t readConfigRegister(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(CONFIG_REGISTER);
  Wire.endTransmission();
  Wire.requestFrom(address, (uint8_t)2);  // Request 2 bytes from the config register

  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  return (msb << 8) | lsb;
}

void writeConfigRegister(uint8_t address, uint16_t config) {
  Wire.beginTransmission(address);
  Wire.write(CONFIG_REGISTER);
  Wire.write(config >> 8);
  Wire.write(config & 0xFF);
  Wire.endTransmission();
}
