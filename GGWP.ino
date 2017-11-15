/*
   Satria Rizky Bismiansyah
   This is a source code from GGWP by GLHF

   This project using a;    1. NodeMCU V3 from Lolin,
                            2. MPU6050 Accelerometer and Gyroscope 3 Axis,
                            3. uBlox Neo-6m v2 GPS Module
                            4. Firebase from Google

*/

// Make sure all of this libarary is installed.
#include <Wire.h>  // Its already on arduino IDE
#include <ESP8266WiFi.h> // Its available when you have added NodeMCU board to your Board Manager.
#include <FirebaseArduino.h> // https://github.com/firebase/firebase-arduino
#include "TinyGPS++.h" // http://arduiniana.org/libraries/tinygpsplus/
#include "SoftwareSerial.h" // Its already on arduino IDE


// Firebase Setup. You can get this on your firebase when you create one.
#define FIREBASE_HOST "xxxx-1234567898765.firebaseio.com"
#define FIREBASE_AUTH "xXXXxxXXXXXXXXxxxxxxXXXXxxxXXxxxxv"

// Setup GPS
SoftwareSerial serial_connection(D3, D2); //RX=D3, TX=D2 (If you use arduino instead, remove the D on the pin number)
TinyGPSPlus gps; // This is the GPS object that will do all you need for GPS. Its a great library.

// Setup Wifi
const char* ssid     = "yourwifissid";
const char* password = "yourwifipassword";

// This is a Global Counter, So You Can Know The Devices is Connected To Your Firebase
// as It Will Uploading a Single Int Everytime The Data Uploaded To The Firebase
long globalcounter;

// Describe The Variable for The Gyro Calculation
double xawal;
double xselanjut;
double yawal;
double yselanjut;
double zawal;
double zselanjut;
double mphbefore;
int counterx = 0;
int countery = 0;
int counterz = 0;

// MPU6050 Slave Device Address (You didn't need to modify this variable)
const uint8_t MPU6050SlaveAddress = 0x68;

// Select SDA and SCL pins for I2C communication
// We are using the D6 and D7 for the MPU6050,
const uint8_t scl = D6;
const uint8_t sda = D7;

// sensitivity scale factor respective to full scale setting provided in datasheet (You didn't need to modify this variable)
const uint16_t AccelScaleFactor = 16384;
const uint16_t GyroScaleFactor = 131;

// MPU6050 few configuration register addresses (You didn't need to modify this variable)
const uint8_t MPU6050_REGISTER_SMPLRT_DIV   =  0x19;
const uint8_t MPU6050_REGISTER_USER_CTRL    =  0x6A;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1   =  0x6B;
const uint8_t MPU6050_REGISTER_PWR_MGMT_2   =  0x6C;
const uint8_t MPU6050_REGISTER_CONFIG       =  0x1A;
const uint8_t MPU6050_REGISTER_GYRO_CONFIG  =  0x1B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG =  0x1C;
const uint8_t MPU6050_REGISTER_FIFO_EN      =  0x23;
const uint8_t MPU6050_REGISTER_INT_ENABLE   =  0x38;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H =  0x3B;
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET  = 0x68;

int16_t AccelX, AccelY, AccelZ, Temperature, GyroX, GyroY, GyroZ;

// This Function Will Tell You Whether The Wifi is Connected or Not
void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event) {
    case WIFI_EVENT_STAMODE_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      break;
    case WIFI_EVENT_STAMODE_DISCONNECTED:
      Serial.println("WiFi lost connection");
      break;
  }
}

void setup(){
  MPU6050_Init();
  globalcounter = 0;
  Serial.begin(9600); // This opens up communications to the Serial monitor in the Arduino IDE
  serial_connection.begin(9600); // This opens up communications to the GPS
  
  Serial.println("Program Started"); // Just print to the monitor that he program is started

  Wire.begin(sda, scl);

  // delete old config
  WiFi.disconnect(true);

  delay(100);

  WiFi.onEvent(WiFiEvent);

  WiFi.begin(ssid, password);

  Serial.println();
  Serial.println();
  Serial.println("Wait for WiFi... ");
  delay(100);
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
}

void loop() {
  double Ax, Ay, Az, T, Gx, Gy, Gz;

  while (serial_connection.available()) //While there are characters to come from the GPS
  {
    gps.encode(serial_connection.read());//This feeds the serial NMEA data into the library one char at a time
  }
  if (gps.location.isUpdated()) //This will pretty much be fired all the time anyway but will at least reduce it to only after a package of NMEA data comes in
  {
    //Get the latest info from the gps object which it derived from the data sent by the GPS unit
    Serial.println("Satellite Count:");
    Serial.println(gps.satellites.value());
    Serial.println("Latitude:");
    Serial.println(gps.location.lat(), 6);
    Serial.println("Longitude:");
    Serial.println(gps.location.lng(), 6);
    Serial.println("Speed MPH:");
    Serial.println(gps.speed.mph());
    Serial.println("Altitude Feet:");
    Serial.println(gps.altitude.feet());
    Serial.println("");
    // It Will Upload The Latitude and Longitude from the GPS to the Firebase.
    Firebase.set("location/latitude", (gps.location.lat(), 6));
    Firebase.set("location/longitude", (gps.location.lng(), 6));
  }

  Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);

  // divide each with their sensitivity scale factor
  Ax = (double)AccelX / AccelScaleFactor; // This will give you the value of X Axis From Accelerometer
  Ay = (double)AccelY / AccelScaleFactor; // This will give you the value of Y Axis From Accelerometer
  Az = (double)AccelZ / AccelScaleFactor; // This will give you the value of Z Axis From Accelerometer 
  T = (double)Temperature / 340 + 36.53; //temperature formula
  Gx = (double)GyroX / GyroScaleFactor; // This will give you the value of X Axis From Gyroscope 
  Gy = (double)GyroY / GyroScaleFactor; // This will give you the value of Y Axis From Gyroscope 
  Gz = (double)GyroZ / GyroScaleFactor; // This will give you the value of Z Axis From Gyroscope 

  // You can pick which one is to show or not.
  Serial.print("Ax: "); Serial.print(Ax);
  Serial.print(" Ay: "); Serial.print(Ay);
  Serial.print(" Az: "); Serial.print(Az);
  Serial.print(" T: "); Serial.print(T);
  Serial.print(" Gx: "); Serial.print(Gx);
  Serial.print(" Gy: "); Serial.print(Gy);
  Serial.print(" Gz: "); Serial.println(Gz);

  // Thisis the logic when the gyroscope detect the unusual value from any axis (in this case, we are just using x and y)
  // Since the gyroscope is always get the counter back to zero when the iteration of this loop ends, we stored the value before
  // and compare the value from the current iteration.
  
  if (counterx == 0) {
    xawal = Gx;
    counterx = counterx + 1;
  }
  else if (counterx == 1) {
    xselanjut = Gx;
    counterx = 0;
    // The gyro will gave the negative and positive value for the opposite.
    if (xawal - xselanjut >= 50 || xawal - xselanjut <= -50) { // Set the threshold for the gyroscope.
      Serial.println("Bahaya X"); // 
      Firebase.set("angle", (xawal - xselanjut)); // Sent the value to the firebase
      Firebase.set("location/latitude", (gps.location.lat(), 6)); // Sent the value to the firebase
      Firebase.set("location/longitude", (gps.location.lng(), 6));// we use location/zzzztude because at the firebase, latitude and longitude located in the location section.
      if (mphbefore > 20) { // If the speed is greater than 20mph when the value of gyroscope trigered. This will let you know "accident" . 
        Firebase.set("kondisi", "Indikasi Kecelakaan");
        Firebase.set("speed", mphbefore);
      }
      else Firebase.set("kondisi", "Motor Terjatuh");
    }
    // This one is for Y axis.
    if (countery == 0) {
      yawal = Gy;
      countery = countery + 1;
    }
    else if (countery == 1) {
      yselanjut = Gy;
      countery = 0;

      if (yawal - yselanjut >= 50 || yawal - yselanjut <= -50) {
        Serial.println("Bahaya Y");
        Firebase.set("angle", xawal - xselanjut);
        Firebase.set("location/latitude", (gps.location.lat(), 6));

        Firebase.set("location/longitude", (gps.location.lng(), 6));

        if (mphbefore > 20) {
          Firebase.set("kondisi", "Indikasi Kecelakaan");
          Firebase.set("speed", mphbefore);
        }
        else Firebase.set("kondisi", "Motor Terjatuh");
      }
    }
    delay(750);
    Firebase.set("globalcounter", globalcounter); // Update the counter to the firebase
    globalcounter += 1;

  }
}

void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}

// read all 14 register
void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t)14);
  AccelX = (((int16_t)Wire.read() << 8) | Wire.read());
  AccelY = (((int16_t)Wire.read() << 8) | Wire.read());
  AccelZ = (((int16_t)Wire.read() << 8) | Wire.read());
  Temperature = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroX = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroY = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroZ = (((int16_t)Wire.read() << 8) | Wire.read());
}

//configure MPU6050
void MPU6050_Init() {
  delay(150);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_CONFIG, 0x00);//set +/-250 degree/second full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x00);// set +/- 2g full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, 0x00);
}

