#include <ArduinoJson.h>
#include <Arduino_LSM6DS3.h>
#include <ArduinoMotorCarrier.h>

const size_t JSON_BUFFER_SIZE = 128;
char jsonBuffer[JSON_BUFFER_SIZE];
StaticJsonDocument<JSON_BUFFER_SIZE> doc;

//Peripheral pins that get initialized by user
int trigPin = 255;
int echoPin = 255;
int tonePin = 255;

// Function to perform digitalRead operation and generate JSON reply
void performDigitalRead(int pin) {
  int digitalValue = digitalRead(pin);
  sendJsonValue(digitalValue);
}

// Function to perform analogRead operation and generate JSON reply
void performAnalogRead(int pin) {
  int analogValue = analogRead(pin);
  sendJsonValue(analogValue);
}

// Function to perform digitalWrite operation
void performDigitalWrite(int pin, int value) {
  digitalWrite(pin, value);
  sendAck();
}

// Function to perform analogRead operation and generate JSON reply
void performAccelRead() {
  // Read accelerometer values
  float x = 0;
  float y = 0;
  float z = 0;
  while (!IMU.accelerationAvailable()) {}
  IMU.readAcceleration(x, y, z);
  StaticJsonDocument<JSON_BUFFER_SIZE> replyDoc;
  replyDoc["x"] = x;
  replyDoc["y"] = y;
  replyDoc["z"] = z;
  char replyBuffer[JSON_BUFFER_SIZE];
  size_t replySize = serializeJson(replyDoc, replyBuffer, JSON_BUFFER_SIZE);
  Serial.write(replyBuffer, replySize);
  Serial.println();
}

void performUltrasonicRead() {
  if (trigPin < 255 && echoPin < 255){
    unsigned long ultraread = 0; //pulseIn returns an unsigned long
    digitalWrite(trigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    ultraread = pulseIn(echoPin, HIGH, 100000);
    sendJsonValue(int(ultraread));
    }
    else {
      sendError();
    }
}

void performEncoderRead(int pin){
  int count = 0;
  int countper = 0;
  switch(pin){
    case 1:
      count = encoder1.getRawCount();
      countper = encoder1.getCountPerSecond();
      encoder1.resetCounter(0);
      break;
    case 2:
      count = encoder2.getRawCount();
      countper = encoder2.getCountPerSecond();
      encoder2.resetCounter(0);
      break;
  }
  StaticJsonDocument<JSON_BUFFER_SIZE> replyDoc;
  replyDoc["count"] = count;
  replyDoc["countper"] = countper;
  char replyBuffer[JSON_BUFFER_SIZE];
  size_t replySize = serializeJson(replyDoc, replyBuffer, JSON_BUFFER_SIZE);
  Serial.write(replyBuffer, replySize);
  Serial.println();
}

void performPiezoTone(int frequency, int duration) {
  if (tonePin < 255){ //Only do this if the piezo has been initialized
//    tone(tonePin, frequency, duration);
    sendAck();
  }
  else {
    sendError(); 
  }
}

void setMotor(int motor, int value) {
  switch (motor) {
      case 1:
        M1.setDuty(value);
        break;
      case 2:
        M2.setDuty(value);
        break;
      case 3:
        M3.setDuty(value);
        break;
      case 4:
        M4.setDuty(value);
        break;
  }
  sendAck();
}
void setServo(int servo, int value) {
  switch (servo) {
    case 1:
          servo1.setAngle(value);
          break;
      case 2:
          servo2.setAngle(value);
          break;
      case 3:
          servo3.setAngle(value);
          break;
      case 4:
          servo4.setAngle(value);
          break;
  }
  sendAck();
}

// Function to parse the JSON message
bool parseJsonMessage(const char* jsonMsg) {
  return deserializeJson(doc, jsonMsg) == DeserializationError::Ok;
}

void sendJsonValue(int val){
  StaticJsonDocument<JSON_BUFFER_SIZE> replyDoc;
  replyDoc["value"] = val;
  char replyBuffer[JSON_BUFFER_SIZE];
  size_t replySize = serializeJson(replyDoc, replyBuffer, JSON_BUFFER_SIZE);
  Serial.write(replyBuffer, replySize);
  Serial.println();
}

void sendAck(){
  StaticJsonDocument<JSON_BUFFER_SIZE> replyDoc;
  replyDoc["message"] = "ack";
  char replyBuffer[JSON_BUFFER_SIZE];
  size_t replySize = serializeJson(replyDoc, replyBuffer, JSON_BUFFER_SIZE);
  Serial.write(replyBuffer, replySize);
  Serial.println();
}

void sendError(){
  StaticJsonDocument<JSON_BUFFER_SIZE> replyDoc;
  replyDoc["message"] = "error";
  char replyBuffer[JSON_BUFFER_SIZE];
  size_t replySize = serializeJson(replyDoc, replyBuffer, JSON_BUFFER_SIZE);
  Serial.write(replyBuffer, replySize);
  Serial.println();
}

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  if (!IMU.begin()) {Serial.println("Failed to initialize LSM6DS3!"); while (1);}
  if (!controller.begin()) {Serial.println("Failed to connect to Motor Carrier!"); while(1);}

  controller.reboot();
  delay(500);
  encoder1.resetCounter(0);
  encoder2.resetCounter(0);
  
  Serial.begin(115200);
  while(!Serial);
}

void loop() {
  // Check if there is data available to read from serial
  if (Serial.available() > 0) {
    // Read the incoming data into the jsonBuffer
    size_t bytesRead = Serial.readBytesUntil('\n', jsonBuffer, JSON_BUFFER_SIZE - 1);
    jsonBuffer[bytesRead] = '\0'; // Null-terminate the string
    // Parse the JSON message
    if (parseJsonMessage(jsonBuffer)) {
      // Retrieve the JSON packet fields
      const char* mode = doc["mode"];
      const char* periph = doc["periph"];
      int pin = doc["pin"];
      int value = doc["value"];

      // perform a "read" operation
      if (strcmp(mode, "read") == 0) {
        if (strcmp(periph, "digital") == 0){performDigitalRead(pin);}
        else if (strcmp(periph, "analog") == 0){performAnalogRead(pin);}
        else if (strcmp(periph, "accel") == 0){performAccelRead();}
        else if (strcmp(periph, "encoder") == 0){performEncoderRead(pin);}
        else if (strcmp(periph, "ultrasonic") == 0){performUltrasonicRead();}
        //else if (strcmp(periph, "reflectance") ==0){}
        //else if (strcmp(periph, "color") ==0){}
      }
      // perform a "write" operation
      else if (strcmp(mode, "write") == 0){
        if (strcmp(periph, "digital") == 0){performDigitalWrite(pin,value);}
        if (strcmp(periph, "led") == 0){performDigitalWrite(LED_BUILTIN,value);}
        if (strcmp(periph, "motor") == 0){setMotor(pin,value);}
        if (strcmp(periph, "servo") == 0){setServo(pin,value);}
        if (strcmp(periph, "piezo") == 0){performPiezoTone(pin,value);}
      }
      // perform an "init" operation
      else if (strcmp(mode, "init") == 0) {
        if (strcmp(periph, "arduino") == 0) {sendAck();}
        else if (strcmp(periph, "dinput") == 0) {pinMode(pin, INPUT_PULLUP); sendAck();} 
        else if (strcmp(periph, "ainput") == 0) {pinMode(pin, INPUT); sendAck();} 
        else if (strcmp(periph, "output") == 0) {pinMode(pin, OUTPUT); sendAck();}
        else if (strcmp(periph, "ultrasonic") ==0) {
          trigPin = pin; pinMode(trigPin, OUTPUT); 
          echoPin = value; pinMode(echoPin, INPUT); 
          sendAck();}
        else if (strcmp(periph, "piezo") ==0) {tonePin = pin; pinMode(tonePin, OUTPUT); sendAck();}
        //else if (strcmp(periph, "reflectance") ==0){}
        //else if (strcmp(periph, "color") ==0){}
       }
    }
    else {
      // Print an error message if parsing failed
        Serial.print("JSON parsing error");
    }
  }
}
