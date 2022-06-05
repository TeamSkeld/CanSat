/*
 BME280

 This example shows how to read a BME280 Sensor
 The circuit:
 * BME280 connected to I2C bus as follows:
 ** Vcc - 3.3V
 ** SCL - pin G22
 ** SDA - pin G21
 ** GND - GND
  */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>



//#define SEALEVELPRESSURE_HPA (1013.25)
float ground_pressure = 1013.25;

Adafruit_BME280 bme; // I2C
float BMEPressure, BMETemp, BMEAltitude, BMEHumidity;
int RecordCounter;
String command;

unsigned long delayTime;

 void Initialise_BME_280(){
  Serial.println(F("BME280 test"));

  bool status;

  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  status = bme.begin(0x76);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
     Serial.println();
  }

  Serial.println("-- Default Test --");
  delayTime = 1000;
}

/*****************************************************************/

/*
 MPU6050

 This example shows how to read a MPU6050 Sensor
 The circuit:
 * MPU6050 connected to I2C bus as follows:
 ** Vcc - 3.3V
 ** SCL - pin G22
 ** SDA - pin G21
 ** GND - GND
  */
#include <Adafruit_MPU6050.h>
//#include <Adafruit_Sensor.h>
//#include <Wire.h>

Adafruit_MPU6050 mpu;

float AccTemp, AccX, AccY, AccZ;

void Initialise_IMU(){
  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
}

/***********************************************************/
/*
  SD card read/write

 This example shows how to write data to an SD card file
 The circuit:
 * SD card attached to SPI bus as follows:
 ** MOSI - pin G23
 ** MISO - pin G19
 ** SCK - pin G18
 ** CS - pin G5   
 ** VSS - pin v5
 ** GND - pin gnd
  */

String CanSatLog;
byte LogNo;
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>

// define the number of bytes you want to access
#define EEPROM_SIZE 4


File LogFile;
File myFile;

void Initialse_SD_Writer() {
  /* define baud rate for Serial ports */
 EEPROM.begin(EEPROM_SIZE);
  if (!SD.begin(5)) {
    Serial.println(F("initialization failed!"));
    return;
  }

 LogNo = EEPROM.read(1);
 Serial.println(LogNo);
  LogNo++;
  EEPROM.write(1, LogNo);
  EEPROM.commit();
  CanSatLog = "/Log_" + String(LogNo) + ".csv";
  Serial.println(LogNo);
 
   Serial.println(CanSatLog);
   
}

/************************************************************/
/*
 Ultimate GPS

 This example shows how to read a MPU6050 Sensor
 The circuit:
 * Ultimate GPS is connected to ESP32 Serial2 port as follows:
 ** V3.3V - 3.3V
 ** TX - pin G16
 ** RX - pin G17
 ** GND - GND
  */

#include <Adafruit_GPS.h>

// what's the name of the hardware serial port?
#define GPSSerial Serial2

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

uint32_t timer = millis();

void Initialse_GPS(){
  Serial.println("Adafruit GPS library basic parsing test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);
}

/****************************************************************************************/
/*
 APC220 Radio

  The circuit:
 * APC220 Radio is connected to ESP32 Serial1 port as follows:
 ** V3.3V - 3.3V
 ** TX - pin G25
 ** RX - pin G26
 ** GND - GND
  */

#include <HardwareSerial.h>

HardwareSerial MySerial(1);

/******************************************************************************************/

/* Landing gear

 */

std::deque<short> velLog;
short vm;
float vk;





void setup() {
  Serial.begin(9600);
  MySerial.begin(2400, SERIAL_8N1, 25, 26); //baudUART mode, Rx, Tx
  Initialise_BME_280();
  Initialise_IMU();
  Initialse_SD_Writer();
  Initialse_GPS();
   

  tone_gen();
 
}


void loop() { 
  

  Start_GPS();

  // approximately every 1 seconds or so, print out the current stats
  if (millis() - timer > 1000) {
    timer = millis(); // reset the timer
 
      Read_BME();
      Read_Accelerometer();
      Print_results();
      LogData(",");
      Transmit_data(",");
      checkTelecommand();
      
      if (velLog.size <5) {
        checkLandingGearNoPop();
      }
      else if (velLog >= 5) {
        checkLandingGear();
      }
 
  }

}

void Read_BME() {
 
  BMETemp = bme.readTemperature();
  
  BMEPressure = bme.readPressure() / 100.0;
  
  BMEAltitude = bme.readAltitude(ground_pressure);
  
  BMEHumidity = bme.readHumidity();
 
}

void Read_Accelerometer(){
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  
  AccX = a.acceleration.x;
  
  AccY = a.acceleration.y;
 
  AccZ = a.acceleration.z;

  AccTemp = temp.temperature;
 
}

void Print_results(){
  RecordCounter++;
  Serial.print(RecordCounter);
  Serial.print("; Pressure;  ");
  Serial.print(BMEPressure);
  Serial.print("; hPa ;");
  Serial.print(" Temperature ; ");
  Serial.print(BMETemp);
  Serial.print("; *C ;");
  Serial.print(" Altitude  ;");
  Serial.print(BMEAltitude);
  Serial.print("; m; ");
  Serial.print("Humidity  ;");
  Serial.print(BMEHumidity);
  Serial.print("; %;");

  Serial.print("      Acceleration X: ;");
   Serial.print(AccX);
   Serial.print("; Y: ;");
   Serial.print(AccY);
   Serial.print("; Y: ;");
   Serial.print(AccZ);
   Serial.print("; m/s^2 ;");
   Serial.print(" Temperature: ;");
   Serial.print(AccTemp);
   Serial.print("; degC;");
   Serial.print(" ");

    Serial.print("Location: ");
      
    Serial.print(GPS.latitude, 4);Serial.print(", "); Serial.print(GPS.lat);
    Serial.print(", ");
    Serial.print(GPS.longitude, 4);Serial.print(", "); Serial.print(GPS.lon);
      //Serial.print("Speed (knots): "); Serial.println(GPS.speed);
     // Serial.print("Angle: "); Serial.println(GPS.angle);
    Serial.print(", Altitude: ,"); Serial.print(GPS.altitude);
    Serial.print(", Satellites: "); Serial.println((int)GPS.satellites);
}

void LogData(String DeLim) {
  LogFile = SD.open(CanSatLog, FILE_APPEND);
  LogFile.print(RecordCounter);
  LogFile.print(DeLim);
  LogFile.print("Pressure  ");
  LogFile.print(DeLim);
  LogFile.print(BMEPressure);
  LogFile.print(DeLim);
  LogFile.print(" hPa ");
  LogFile.print(DeLim);
  LogFile.print(" Temperature ");
  LogFile.print(DeLim);
  LogFile.print(BMETemp);
  LogFile.print(DeLim);
  LogFile.print(" *C ");
  LogFile.print(DeLim);
  LogFile.print(" Altitude  ");
  LogFile.print(DeLim);
  LogFile.print(BMEAltitude);
  LogFile.print(DeLim);
  LogFile.print(" m ");
  LogFile.print(DeLim);
  LogFile.print("Humidity  ");
  LogFile.print(DeLim);
  LogFile.print(BMEHumidity);
  LogFile.print(DeLim);
  LogFile.print(" %");
  LogFile.print(DeLim);

  LogFile.print(" Acceleration X: ");
  LogFile.print(DeLim);
   LogFile.print(AccX);
   LogFile.print(DeLim);
   LogFile.print(" Y: ");
   LogFile.print(DeLim);
   LogFile.print(AccY);
   LogFile.print(DeLim);
   LogFile.print(" Y: ");
   LogFile.print(DeLim);
   LogFile.print(AccZ);
   LogFile.print(DeLim);
   LogFile.print(" m/s^2 ");
   LogFile.print(DeLim);
   LogFile.print(" Temperature: ");
   LogFile.print(DeLim);
   LogFile.print(AccTemp);
   LogFile.print(DeLim);
   LogFile.print(" degC");
   LogFile.println(" ");

   LogFile.print("Location: ");
      
    LogFile.print(GPS.latitude, 4);LogFile.print(DeLim); LogFile.print(GPS.lat);LogFile.print(DeLim);
    LogFile.print(GPS.longitude, 4);LogFile.print(DeLim); LogFile.print(GPS.lon);LogFile.print(DeLim);
    LogFile.print(" Altitude: "); LogFile.print(DeLim); LogFile.print(GPS.altitude);LogFile.print(DeLim);
    LogFile.print(" Satellites: "); LogFile.print(DeLim); LogFile.println((int)GPS.satellites);
  
  LogFile.close();
}

void Transmit_data(String DeLim){
  MySerial.print(RecordCounter);
  MySerial.print(DeLim);
  //MySerial.print("Pressure  ");
  //MySerial.print(DeLim);
  MySerial.print(BMEPressure);
  MySerial.print(DeLim);
 // MySerial.print(" hPa ");
 // MySerial.print(DeLim);
 // MySerial.print(" Temperature ");
  //MySerial.print(DeLim);
  MySerial.print(BMETemp);
  MySerial.print(DeLim);
  //MySerial.print(" *C ");
 // MySerial.print(DeLim);
  //MySerial.print(" Altitude  ");
 // MySerial.print(DeLim);
  MySerial.print(BMEAltitude);
  MySerial.print(DeLim);
 // MySerial.print(" m ");
 // MySerial.print(DeLim);
//  MySerial.print("Humidity  ");
//  MySerial.print(DeLim);
  MySerial.print(BMEHumidity);
  MySerial.print(DeLim);
//  MySerial.print(" %");
//  MySerial.print(DeLim);

//  MySerial.print(" Acceleration X: ");
 // MySerial.print(DeLim);
   MySerial.print(AccX);
   MySerial.print(DeLim);
//   MySerial.print(" Y: ");
//   MySerial.print(DeLim);
   MySerial.print(AccY);
   MySerial.print(DeLim);
//   MySerial.print(" Y: ");
//   MySerial.print(DeLim);
   MySerial.print(AccZ);
   MySerial.print(DeLim);
 //  MySerial.print(" m/s^2 ");
 //  MySerial.print(DeLim);
 //  MySerial.print(" Temperature: ");
//   MySerial.print(DeLim);
   MySerial.print(AccTemp);
   MySerial.print(DeLim);
//   MySerial.print(" degC");
//   MySerial.println(" ");

 //  MySerial.print("Location: ");
      
    MySerial.print(GPS.latitude, 4);MySerial.print(DeLim); MySerial.print(GPS.lat);MySerial.print(DeLim);
    MySerial.print(GPS.longitude, 4);MySerial.print(DeLim); MySerial.print(GPS.lon);MySerial.print(DeLim);
    MySerial.print(" Altitude: "); MySerial.print(DeLim); MySerial.print(GPS.altitude);MySerial.print(DeLim);
    MySerial.print(" Satellites: "); MySerial.print(DeLim); MySerial.println((int)GPS.satellites);
    MySerial.print("Speed (knots): "); MySerial.print(DeLim); MySerial.println(GPS.speed); MySerial.print(DeLim);
}

void tone_gen(){
  // Change this depending on where you put your piezo buzzer
  const int TONE_OUTPUT_PIN = 32;

  // The ESP32 has 16 channels which can generate 16 independent waveforms
  // We'll just choose PWM channel 0 here
  const int TONE_PWM_CHANNEL = 0; 
  
  ledcAttachPin(TONE_OUTPUT_PIN, TONE_PWM_CHANNEL);
    for(int x = 0; x < 5; x++){
      ledcWriteTone(TONE_PWM_CHANNEL, 1200);
      delay(100);
      ledcWriteTone(TONE_PWM_CHANNEL, 1800);
      delay(100);
      }
   ledcDetachPin(TONE_OUTPUT_PIN);
}

void Start_GPS(){
   // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
   // Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
}

void checkTelecommand (){  

    if(MySerial.available()){
        command = MySerial.readStringUntil('\n');
        if(command.equals("alt0")){
            Serial.println("Hello from JES Cansat!");
            ground_pressure = (bme.readPressure() / 100.0F);
            Serial.println(ground_pressure);
            Serial.println("Altitude set to 0 m!");
        }
        else if(command.equals("1")){
            tone_gen();
        }
        else if(command.equals("2")){
           tone_gen();
           tone_gen();
           tone_gen();
           tone_gen();
           
        }
        else{
            
        }
    }
}

void checkLandingGear() {
   //get speed in knots
  vk = GPS.speed;
  // roughly convert to m/s
  vk = vk / 2;
  //convert to short
  vm = static_cast<short>(vk);
  //add to queue
  velLog.push_back(vm);
  velLog.pop_front();

  //check if past 5 values of speed are within the parachutes range
  //if they are activates the landing gear
  short rangeCheck = 0;
  for (int i = 0; i < 5; ++i) {
    if (7 < velLog[i] < 12) {
      rangeCheck++;
    }
  if (rangeCheck = 5)
    activateLandingGear()
  } 
}
void activateLandingGear() {
  //TODO wire and program servos
  Serial.println("---LANDING GEAR ACTIVATED---");
}

void checkLandingGearNoPop() {
  //get speed in knots
  vk = GPS.speed;
  // roughly convert to m/s
  vk = vk / 2;
  //convert to short
  vm = static_cast<short>(vk);
  //add to queue
  velLog.push_back(vm);
}
