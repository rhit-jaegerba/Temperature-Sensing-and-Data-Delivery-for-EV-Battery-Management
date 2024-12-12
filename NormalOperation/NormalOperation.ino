#include <Wire.h>

const uint8_t NUMBER_OF_MODULES = 3;
uint16_t moduleMaxTemperature[NUMBER_OF_MODULES];
uint16_t moduleAvgTemperature[NUMBER_OF_MODULES];
uint16_t moduleMinTemperature[NUMBER_OF_MODULES];

//TCA9548A I2C Mux
#include <SparkFun_I2C_Mux_Arduino_Library.h>  //Click here to get the library: http://librarymanager/All#SparkFun_I2C_Mux
QWIICMUX mux;

const uint8_t MUX_ADDRESS = 0x70;
const uint8_t NUMBER_OF_MUX_PORTS = 8;



//TMP112 temperature sensor
const uint8_t TMP_ADDRESSES[4] = { 0x48, 0x49, 0x4A, 0x4B };
const uint8_t NUMBER_OF_TMP_ADDRESSES = 4;
#define TEMPERATURE_REGISTER 0x00
#define CONFIG_REGISTER 0x01
#define T_LOW_REGISTER 0x02
#define T_HIGH_REGISTER 0x03



//Sensors connected to the system
//presentSensors(Mux port 0-7, TMP_ADDRESSES 0-3)
//0-Not Connected
//x>0-Module number
const uint8_t connectedSensors[8][4] = {
  { 1, 0, 0, 0 },
  { 0, 0, 0, 0 },
  { 0, 0, 0, 0 },
  { 0, 0, 0, 0 },
  { 0, 0, 0, 0 },
  { 0, 0, 0, 0 },
  { 0, 0, 0, 0 },
  { 0, 0, 0, 0 }
};
uint16_t sensorValues[8][4];



void setup() {
  Serial.begin(9600);
  Wire.begin();

  if (mux.begin(MUX_ADDRESS) == false) {
    Serial.println("Mux not detected. Freezing...");
    while (1)
      ;
  }
  Serial.println("Mux detected");

  for (int i = 0; i < NUMBER_OF_MUX_PORTS; i++) {
    mux.setPort(i);
    for (int j = 0; j < NUMBER_OF_TMP_ADDRESSES; j++) {
      sensorValues[i][j] = 0; //Initalize array;
      if (connectedSensors[i][j]>0) {
        configureSensor(TMP_ADDRESSES[j]);
        if (!initializeTMP112(TMP_ADDRESSES[j])) {
        Serial.println("Cannot connect to mux port: "+ String(i) + " Address: " + String(TMP_ADDRESSES[j]));
        while (1);
        }
      }
    }
  }
}

void loop() {
  // collect sensor values
  for (int i = 0; i < NUMBER_OF_MUX_PORTS; i++) {
    mux.setPort(i);
    for (int j = 0; j < NUMBER_OF_TMP_ADDRESSES; j++) {
      if (connectedSensors[i][j]>0) {
        sensorValues[i][j] = readTempC(TMP_ADDRESSES[j]);
      }
    }
  }
  //Process Data
}

bool initializeTMP112(uint8_t address) {
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