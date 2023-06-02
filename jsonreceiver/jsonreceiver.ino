#include <ArduinoJson.h>
#include <Arduino_LSM6DS3.h>

const size_t JSON_BUFFER_SIZE = 1024;
char jsonBuffer[JSON_BUFFER_SIZE];
StaticJsonDocument<JSON_BUFFER_SIZE> doc;

// Function to parse the JSON message
bool parseJsonMessage(const char* jsonMsg) {
  return deserializeJson(doc, jsonMsg) == DeserializationError::Ok;
}

// Function to perform digitalRead operation and generate JSON reply
void performDigitalRead(int pin) {
  int digitalValue = digitalRead(pin);
  StaticJsonDocument<JSON_BUFFER_SIZE> replyDoc;
  replyDoc["value"] = digitalValue;
  char replyBuffer[JSON_BUFFER_SIZE];
  size_t replySize = serializeJson(replyDoc, replyBuffer, JSON_BUFFER_SIZE);
  Serial.write(replyBuffer, replySize);
  Serial.println();
}

// Function to perform analogRead operation and generate JSON reply
void performAnalogRead(int pin) {
  int analogValue = analogRead(pin);
  StaticJsonDocument<JSON_BUFFER_SIZE> replyDoc;
  replyDoc["value"] = analogValue;
  char replyBuffer[JSON_BUFFER_SIZE];
  size_t replySize = serializeJson(replyDoc, replyBuffer, JSON_BUFFER_SIZE);
  Serial.write(replyBuffer, replySize);
  Serial.println();
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

void sendAck(){
  StaticJsonDocument<JSON_BUFFER_SIZE> replyDoc;
  replyDoc["message"] = "ack";
  char replyBuffer[JSON_BUFFER_SIZE];
  size_t replySize = serializeJson(replyDoc, replyBuffer, JSON_BUFFER_SIZE);
  Serial.write(replyBuffer, replySize);
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  if (!IMU.begin()) {
  Serial.println("Failed to initialize LSM6DS3!");
  while (1);
  }
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

      // Perform digitalRead operation if requested
      if (strcmp(mode, "read") == 0) {
        if (strcmp(periph, "digital") == 0){performDigitalRead(pin);}
        else if (strcmp(periph, "analog") == 0){performAnalogRead(pin);}
        else if (strcmp(periph, "accel") == 0){performAccelRead();}
      }
      else if (strcmp(mode, "write") == 0){
        if (strcmp(periph, "digital") == 0){performDigitalWrite(pin,value);}
        if (strcmp(periph, "led") == 0){performDigitalWrite(LED_BUILTIN,value);}
      }
      // Perform pinMode operation if requested
      else if (strcmp(mode, "init") == 0) {
        if (strcmp(periph, "dinput") == 0) {pinMode(pin, INPUT_PULLUP); sendAck();} 
        else if (strcmp(periph, "ainput") == 0) {pinMode(value, INPUT); sendAck();} 
        else if (strcmp(periph, "output") == 0) {pinMode(value, OUTPUT); sendAck();}
       }
    }
    else {
      // Print an error message if parsing failed
        Serial.print("JSON parsing error");
    }
  }
}
