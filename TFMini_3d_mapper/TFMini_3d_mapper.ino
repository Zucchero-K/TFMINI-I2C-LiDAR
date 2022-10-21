/* Written for Arduino Uno/Nano.
  Requirements:
  - button in pull-up mode
  - two sg90 servos
  - TMini I2C module (https://www.sparkfun.com/products/14786)
  - SD-card module
  - 3D-printed robotic stand, to move the sensor over jaw and pitch
*/

#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include "SdFat.h"

SdFat SD;

File DataFile;
Servo servo_jaw;
Servo servo_pitch;

#define SD_CS_PIN SS // SS = pin 10

const byte TFM_addr = 0x10; //TFMini I2C Address

const int jaw_pin = 6;
const int pitch_pin = 7;
const int startButton = 5;

const int z_offset = 97; // height of sensor above underlaying ground

uint16_t dist;
uint16_t strength;

int R; // Distance
int X; // Coordinates
int Y;
int Z;

bool valid_data = false;
bool triggerDone = false;

const float pi = 3.14159;
const float conv = pi / 180.0;

int stepAngle = 3; // Degrees per step
int stabilize_delay = 150; // Delay between each pitch adjustment

//int stepAngle = 90; // Degrees per step
//int stabilize_delay = 0;

const String fileName = "data_file";
const String fileExtension = ".txt";
String stringBuffer = "";
const int namesize = 20;
char charBuffer[namesize];

void setup() {

  servo_jaw.attach(jaw_pin);
  servo_pitch.attach(pitch_pin);
  pinMode(startButton, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  servo_jaw.write(0);
  servo_pitch.write(0);
  delay(50);
  Serial.begin(115200);
  Wire.begin();
  delay(50);

  Serial.print("Initializing SD card...");
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("initialization failed!");
    error();
  }
  Serial.println("initialization done.");
}

void loop() {

  if (!digitalRead(startButton)) { // Active when LOW (pull-up resistor config)
    scan();
    while (!digitalRead(startButton)) {
      ;// Wait until button is released
    }
  }

}

void scan() {

  createNewFile();
  if (DataFile) {
    bool pitch_direction = true;
    for (int jaw = 0; jaw < 181; jaw += stepAngle) {
      if (pitch_direction) {
        for (int pitch = 0; pitch < 181; pitch += stepAngle) {
          moveServos(jaw, pitch);
          getPoint(jaw, pitch);
        }
        //goto endTest;
        pitch_direction = false;
      }
      else {
        for (int pitch = 180; pitch >= 0; pitch -= stepAngle) {
          moveServos(jaw, pitch);
          getPoint(jaw, pitch);
        }
        pitch_direction = true;
      }
    }
    //endTest:
    DataFile.close();
  } else {
    return;
  }
  servo_jaw.write(0);
  servo_pitch.write(0);
}

void moveServos(int jaw, int pitch) {
  servo_jaw.write(jaw);
  servo_pitch.write(pitch);
  delay(stabilize_delay); // Allow some time to stabilize the servos
}

void getPoint(int jaw, int pitch) {
  if (!getDistance(TFM_addr)) {
    Serial.println(F("Invalid data."));
    return;
  }
  R = dist;
  sphericalToCartesian(R, angleFix(jaw), angleFix(pitch), X, Y, Z);
  if (DataFile) {
    DataFile.println(String(X) + " " + String(Y) + " " + String(Z));
  }
}

bool getDistance(uint8_t devAddr) {
  //jmp_start:
  valid_data = false;
  //triggerDone = false;
  Wire.beginTransmission(devAddr);
  Wire.write(0x01); //MSB
  Wire.write(0x02); //LSB
  Wire.write(7); //Data length: 7 bytes for distance data
  if (Wire.endTransmission(false) != 0) {
    Serial.println(F("Sensor !ACK"));
    return false; //Sensor did not ACK
  }
  Wire.requestFrom(devAddr, (uint8_t)7); //Ask for 7 bytes

  if (Wire.available()) {
    for (uint8_t x = 0 ; x < 7 ; x++) {
      uint8_t incoming = Wire.read();
      /*
        if (x == 0) {
        //Trigger done
        if (incoming == 0x01) {
          triggerDone = true;
        }
        else if(incoming == 0x00){
          Serial.println(F("Previous frame"));
        }
        }
      */
      if (x == 2)
        dist = incoming; //LSB of the distance value "Dist_L"
      else if (x == 3)
        dist |= incoming << 8; //MSB of the distance value "Dist_H"
      else if (x == 4)
        strength = incoming; //LSB of signal strength value
      else if (x == 5)
        strength |= incoming << 8; //MSB of signal strength value
    }
    if (strength > (uint16_t)20) {
      valid_data = true;
    }
  }
  else {
    Serial.println(F("No wire data available"));
    return false;
  }
  /*
    if(!triggerDone){
    delay(30); // Short delay before sending new request
    goto jmp_start;
    }*/
  if (valid_data) {
    return true;
  }
  else {
    return false;
  }
}

// Note: The equations have been altered to better match the servo angle values
void sphericalToCartesian(int r, int jaw, int pitch, int& x, int& y, int& z) {
  x = r * cos(jaw * conv) * cos(pitch * conv);
  y = r * sin(jaw * conv) * cos(pitch * conv);
  z = r * sin(pitch * conv) + z_offset;
}

// Cheap servo angles are not very accurate, so actual angle values require correction
// Flat objects can appear concave when a closest point of the surface is in detection range
// of the sensor's FOV for multiple steps of rotation !!!
int angleFix(int angle) {
  angle = map(angle, 0, 180, 0, 170);
  return angle;
}

void clearFilename(char* filename, uint8_t namesize) {
  for (int i = 0; i < namesize; i++) {
    filename[i] = 0x00;
  }
  // memset(filename,0x00,namesize); // maybe faster
}

void createNewFile() {

  DataFile.close();
  clearFilename(charBuffer, namesize);
  stringBuffer = fileName + fileExtension;
  stringBuffer.toCharArray(charBuffer, namesize);
  if (!SD.exists(charBuffer)) {
    DataFile = SD.open(charBuffer, FILE_WRITE);
  }
  else {
    int maxNum = 255;
    int num = 2;
    stringBuffer = fileName + " (" + String(num) + ")" + fileExtension;
    stringBuffer.toCharArray(charBuffer, namesize);
    while (SD.exists(charBuffer)) {
      if (num == maxNum) error();
      num++;
      clearFilename(charBuffer, namesize);
      stringBuffer = fileName + " (" + String(num) + ")" + fileExtension;
      stringBuffer.toCharArray(charBuffer, namesize);
    }
    DataFile = SD.open(charBuffer, FILE_WRITE);
  }
  if (!DataFile) {
    error();
  }
}

void error() { // Blinking led means an error occurred.
  DataFile.close();
  Serial.println("Error initializing the SD card or creating a file.");
  while (1) {
    
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    
  }
}
