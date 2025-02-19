#include <Wire.h>
#include <Wire1.h>

const uint8_t NUMBER_OF_MODULES = 3;
uint16_t moduleMaxTemperature[NUMBER_OF_MODULES];
uint16_t moduleAvgTemperature[NUMBER_OF_MODULES];
uint16_t moduleMinTemperature[NUMBER_OF_MODULES];
const uint8_t WINDOW_SIZE = 8;        //Size of the Floating Average Window
const float WINDOW_TOLERANCE = 0.25;  //Tolerance of the Floating Average Window

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

#define SLAVE_ADDRESS 0x20

//Sensors connected to the system
//presentSensors(Mux port 0-7, TMP_ADDRESSES 0-3)
//0-Not Connected
//x>0-Module number
const uint8_t connectedSensors[8][4] = {
  { 2, 2, 2, 2 },
  { 1, 1, 0, 0 },
  { 1, 1, 1, 1 },
  { 1, 1, 1, 1 },
  { 1, 1, 1, 1 },
  { 1, 1, 1, 1 },
  { 1, 1, 1, 1 },
  { 1, 1, 1, 1 }
};

int16_t sensorValues[8][4] = { 0 };  //Holds the last calculated average for each sensor

int16_t currentValues[WINDOW_SIZE];
int16_t firstAverage = 0;
int16_t secondAverage = 0;
uint8_t averageCounter = 0;
int16_t currentTemp = 0;

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
      //sensorValues[i][j][0] = 0; //Initalize array;
      if (connectedSensors[i][j] > 0) {
        configureSensor(TMP_ADDRESSES[j]);
        if (!initializeTMP112(TMP_ADDRESSES[j])) {
          Serial.println("Cannot connect to mux port: " + String(i) + " Address: " + String(TMP_ADDRESSES[j]));
          while (1)
            ;
        }
      }
    }
  }

  // Initialize the secondary I2C bus as a Slave
  Wire1.begin(SLAVE_ADDRESS);
  Wire1.onReceive(receiveEvent);  // Callback for data received
  Wire1.onRequest(requestEvent);  // Callback for data requested
  Serial.println("Slave I2C Initialized");
}

void loop() {
  // collect sensor

  for (int i = 0; i < NUMBER_OF_MUX_PORTS; i++) {
    mux.setPort(i);
    for (int j = 0; j < NUMBER_OF_TMP_ADDRESSES; j++) {
      if (connectedSensors[i][j] > 0) {
        for (int z = 0; z < WINDOW_SIZE; z++) {
          currentTemp = readTemp(TMP_ADDRESSES[j]);
          currentValues[z] = currentTemp;
          firstAverage += currentTemp;
        }

        firstAverage /= WINDOW_SIZE;

        for (int y = 0; y < WINDOW_SIZE; y++) {
          if (currentValues[y] < (firstAverage * (1 + WINDOW_TOLERANCE)) && currentValues[y] > (firstAverage * (1 - WINDOW_TOLERANCE))) {
            secondAverage += currentValues[y];
            averageCounter++;
          }
        }

        secondAverage /= averageCounter;
        sensorValues[i][j] = secondAverage;
        firstAverage = 0;
        secondAverage = 0;
        averageCounter = 0;
      }
    }
  }


  //Process Data
  for (int i = 0; i < 8; i++) {
    for (int j = 0; j < 4; j++) {
      Serial.print(sensorValues[i][j] * 0.0625);
      Serial.print("\t");  // Tab space for formatting
    }
    Serial.println();  // New line after each row
  }
  delay(1000);
  Serial.println();
  Serial.println();
}

bool initializeTMP112(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

void configureSensor(uint8_t address) {
  setConversionRate(address, 2);    // Set conversion rate to 4 Hz
  setExtendedMode(address, false);  // Standard mode (-55°C to 128°C)
  setAlertPolarity(address, true);  // Active HIGH alert
  setAlertMode(address, false);     // Comparator mode
  setFaultQueue(address, 0);        // Trigger alert on first fault
  setHighTempF(address, 85.0);      // Set T_HIGH threshold to 85°F
  setLowTempF(address, 84.0);       // Set T_LOW threshold to 84°F
}

int16_t readTemp(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(TEMPERATURE_REGISTER);
  Wire.endTransmission();
  Wire.requestFrom(address, (uint8_t)2);  // Request 2 bytes from the sensor

  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();

  int16_t temp = (msb << 4) | (lsb >> 4);
  if (temp & 0x800) temp |= 0xF000;  // Handle negative temperatures

  return temp;
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

byte receivedByte;
//Slave receive event
void receiveEvent(int numBytes) {
  //Serial.print("Slave Received: ");

  if (numBytes == 0) return;

  receivedByte = 0;
  while (Wire1.available()) {
    receivedByte = Wire1.read();  // Read the received byte
    //Serial.println(receivedByte, HEX); // Print received byte in HEX format
  }
}


//Slave request event
uint8_t command_high_nibble = 0;
uint16_t command_low_nibble = 0;
uint8_t moduleIndex = 0;
int16_t MinValue = 0xFD80;
int16_t AvgValue = 0x05D0;
int16_t MaxValue = 0x07D0;
void requestEvent() {
  command_high_nibble = receivedByte >> 4;
  command_low_nibble = receivedByte & 0b00001111;
  // Serial.print("command_high_nibble: ");
  // Serial.print(command_high_nibble);
  // Serial.print("        ");
  // Serial.print("command_low_nibble: ");
  // Serial.println(command_low_nibble);
  if ((command_high_nibble <= 7) && (command_low_nibble <= 3)) {
    Serial.print("muxIndex: ");
    Serial.print(command_high_nibble);
    Serial.print("        ");
    Serial.print("sensorIndex: ");
    Serial.println(command_low_nibble);
    Wire1.write(sensorValues[command_high_nibble][command_low_nibble] >> 8);
    Wire1.write(sensorValues[command_high_nibble][command_low_nibble]);
  } else if ((command_high_nibble >= 8) && (command_low_nibble <= 2)) {
    // Serial.print("command_high_nibble: ");
    // Serial.print(command_high_nibble);
    moduleIndex = command_high_nibble - 0x8;
    // Serial.print("moduleIndex: ");
    // Serial.print(moduleIndex);
    // Serial.print("        ");
    // Serial.print("Command type: ");
    // Serial.println(command_low_nibble);
    switch (command_low_nibble) {
      case 0:
        Serial.println("Default 0");
        MinValue = calculateModuleMin(moduleIndex); //Replace argument with selected module
        //MinValue = 0x07D0;
        Wire1.write(MinValue >> 8);
        Wire1.write(MinValue);
        break;
      case 1:
        Serial.println("Default 1");
        AvgValue = calculateModuleAverage(moduleIndex); //Replace argument with selected module
        AvgValue = 0x05D0;
        Wire1.write(AvgValue >> 8);
        Wire1.write(AvgValue);
        // AvgValue += 60;
        // if(AvgValue >= (125*16)){
        //   AvgValue = -40*16;
        // }
        break;
      case 2:
        Serial.println("Default 2");
        MaxValue = calculateModuleMax(moduleIndex); //Replace argument with selected module
        //MaxValue = 0xFD80;
        Wire1.write(MaxValue >> 8);
        Wire1.write(MaxValue);
        break;
      default:
        Serial.print("Default");
        break;
    }
  }
}

int16_t calculateModuleMax(uint8_t module) {
  int16_t max = 0x8000;
  for (int i = 0; i < NUMBER_OF_MUX_PORTS; i++) {
    //mux.setPort(i);
    for (int j = 0; j < NUMBER_OF_TMP_ADDRESSES; j++) {
      if (connectedSensors[i][j] == module && sensorValues[i][j] > max) {
        max = sensorValues[i][j];
      }
      Serial.print("max:");
      Serial.println(max*0.0625);
      return max;
    }
  }
}

int16_t calculateModuleMin(uint8_t module) {
  int16_t min = 0x7FFF;  //Larger than sensor will ever report
  for (int i = 0; i < NUMBER_OF_MUX_PORTS; i++) {
    //mux.setPort(i);
    for (int j = 0; j < NUMBER_OF_TMP_ADDRESSES; j++) {
      if (connectedSensors[i][j] == module && sensorValues[i][j] < min) {
        min = sensorValues[i][j];
      }
      Serial.print("min:");
      Serial.println(min*0.0625);
      return min;
    }
  }
}

int16_t calculateModuleAverage(uint8_t module) {
  int16_t average = 0;
  int16_t averageCounter = 0;
  for (int i = 0; i < NUMBER_OF_MUX_PORTS; i++) {
    //mux.setPort(i);
    for (int j = 0; j < NUMBER_OF_TMP_ADDRESSES; j++) {
      if (connectedSensors[i][j] == module) {
        average += sensorValues[i][j];
        averageCounter++;
      }

      average /= averageCounter;
      Serial.print("average:");
      Serial.println(average*0.0625);      
      return average;
    }
  }
}