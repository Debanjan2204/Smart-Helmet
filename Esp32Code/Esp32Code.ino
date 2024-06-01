#include <WiFi.h>
#include <FirebaseESP32.h>
#include "Wire.h"
#include <MPU6050_light.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>



#define FIREBASE_HOST "<YOUR FIREBASE RDBMS ID>"
#define FIREBASE_AUTH "<YOUR FIREBASE AUTH KEY>"
#define WIFI_SSID "<WIFI NAME>"
#define WIFI_PASSWORD "<WIFI PASSWORD>"


//Define FirebaseESP32 data object
FirebaseData firebaseData;
FirebaseJson json;

MPU6050 mpu(Wire);


#define GPSBaud 9600
HardwareSerial SerialGPS(2);   // use hardware serial to connect to GPS module
TinyGPSPlus gps;               // create a TinyGPS++ object to handle GPS data


int alc_sens;
float accX,accY,accZ,gyroX,gyroY,gyroZ;
float lati=0,longi=0;

void setup()
{

  Serial.begin(9600);
  pinMode(2,OUTPUT);
  Wire.begin();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  digitalWrite(2,HIGH);
  Serial.println();

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);

  //Set database read timeout to 1 minute (max 15 minutes)
  Firebase.setReadTimeout(firebaseData, 1000 * 60);
  //tiny, small, medium, large and unlimited.
  //Size and its write timeout e.g. tiny (1s), small (10s), medium (30s) and large (60s).
  Firebase.setwriteSizeLimit(firebaseData, "tiny");



  Serial.println("------------------------------------");
  Serial.println("Connected...");

 // MPU6050,MQ-3 AND NEO-6M INITIALIZATION//////////////////////////////////////////////////
byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");
  pinMode(34, INPUT);  // MQ-3
  pinMode(15, OUTPUT);  // MQ-3-node
  SerialGPS.begin(GPSBaud); // NEO6M GPRS
  
}

void loop() {
alc_sens=analogRead(34);

if(alc_sens>500)
{
  digitalWrite(15,HIGH);
}
else
{
  digitalWrite(15,LOW);
}


  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());  // get GPS data from NEO-6M module and feed it to TinyGPS++
  }
  if (gps.location.isUpdated()) {  // check if GPS data is valid
    lati= gps.location.lat();
    longi=gps.location.lng();
  }
mpu.update();
accX=mpu.getAccX();
accY=mpu.getAccY();
accZ=mpu.getAccZ();

gyroX=mpu.getGyroX();
gyroY=mpu.getGyroY();
gyroZ=mpu.getGyroZ();
//float sensorValue = float( random(0,1000))/100;  // Replace with your desired float value

////////////////////// Upload mq3 data/////////////////////////////
  if (Firebase.setInt(firebaseData, "/sensorData/alc", alc_sens)) {
    Serial.println("MQ3 Data uploaded successfully!");
  } else {
    Serial.println("Error uploading MQ3 Data to Firebase!");
    Serial.println(firebaseData.errorReason());
  }
////////////////////////////////////////////////////////////////////////////

////////////////////// Upload mpu6050 data///////////////////////////// 
//////////////////////ACCELERATION DATA///////////////////////////////
  if (Firebase.setFloat(firebaseData, "/sensorData/mpu6050/acc/accx", accX)) {
    Serial.println("MPU AccX Data uploaded successfully!");
  } else {
    Serial.println("Error uploading AccX data to Firebase!");
    Serial.println(firebaseData.errorReason());
  }

  if (Firebase.setFloat(firebaseData, "/sensorData/mpu6050/acc/accy", accY)) {
    Serial.println("MPU AccY Data uploaded successfully!");
  } else {
    Serial.println("Error uploading AccY data to Firebase!");
    Serial.println(firebaseData.errorReason());
  }

  if (Firebase.setFloat(firebaseData, "/sensorData/mpu6050/acc/accz", accZ)) {
    Serial.println("MPU AccZ Data uploaded successfully!");
  } else {
    Serial.println("Error uploading AccZ data to Firebase!");
    Serial.println(firebaseData.errorReason());
  }
  
//////////////////////GYROSCOPE DATA///////////////////////////////
  if (Firebase.setFloat(firebaseData, "/sensorData/mpu6050/gyro/gyrox", gyroX)) {
    Serial.println("MPU GyroX Data uploaded successfully!");
  } else {
    Serial.println("Error uploading GyroX data to Firebase!");
    Serial.println(firebaseData.errorReason());
  }

  if (Firebase.setFloat(firebaseData, "/sensorData/mpu6050/gyro/gyroy", gyroY)) {
    Serial.println("MPU  Data uploaded successfully!");
  } else {
    Serial.println("Error uploading GyroY data to Firebase!");
    Serial.println(firebaseData.errorReason());
  }

  if (Firebase.setFloat(firebaseData, "/sensorData/mpu6050/gyro/gyroz", gyroZ)) {
    Serial.println("MPU GyroZ Data uploaded successfully!");
  } else {
    Serial.println("Error uploading GyroZ data to Firebase!");
    Serial.println(firebaseData.errorReason());
  }
//////////////////////////////////////////////////////////////////////////
//////////////////////gps DATA///////////////////////////////
  if (Firebase.setFloat(firebaseData, "/sensorData/gps/lat", lati)) {
    Serial.println("GPS(LAT) Data uploaded successfully!");
  } else {
    Serial.println("Error uploading GPS(LAT) data to Firebase!");
    Serial.println(firebaseData.errorReason());
  }

    if (Firebase.setFloat(firebaseData, "/sensorData/gps/lng", longi)) {
    Serial.println("GPS(LNG) Data uploaded successfully!");
  } else {
    Serial.println("Error uploading GPS(LNG) data to Firebase!");
    Serial.println(firebaseData.errorReason());
  }

  
  delay(5000); // Wait for 5 seconds before the next upload
}

