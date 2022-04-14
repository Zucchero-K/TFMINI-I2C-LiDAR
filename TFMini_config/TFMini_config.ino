/*
  Example sketch for the Qwiic Enabled TFMini.
  Written in collaboration by Nate Seidle, Benewake.
 (https://www.sparkfun.com/products/14786)

  TFMini_config (I2C)
  Supplemented by Zucchero Klein.
  Sketch now allows the user to parse configuration
  commands to the TFMini through serial monitor.

  Commands can be entered in serial monitor using HEX codes given in the Benewake product manual.
  Format: "config" + RegAddr(HEX) + Data(HEX)
  As an example, to change the unit of distance output to (mm), enter:
  "config 0066 00".
  Another example, to set the range output limit threshold to 12000 mm, enter:
  "config 0056 2EE0".

*/

#include <Wire.h>

uint16_t distance = 0; //distance
uint16_t strength = 0; // signal strength
uint8_t rangeType = 0; //range scale
/*Value range:
  00 (short distance)
  03 (intermediate distance)
  07 (long distance) */

boolean valid_data = false; //ignore invalid ranging data

byte address = 0x10; //TFMini I2C Address

void configTF(uint8_t addr, uint8_t RegH, uint8_t RegL, uint8_t len, uint8_t DataL, uint8_t DataH = 0x00);
uint8_t getHexChar(char MS, char LS);

void setup() {
  Wire.begin();

  Serial.begin(115200);
  Serial.println("TFMini I2C Test");
}

void loop() {
  if (address){
    if (readDistance(address) == true) {
      if (valid_data == true) {
        Serial.print("\tDist[");
        Serial.print(distance);
        Serial.print("]\tstrength[");
        Serial.print(strength);
        Serial.print("]\tmode[");
        Serial.print(rangeType);
        Serial.print("]");
        Serial.println();
      }
      else {
        //don't print invalid data
      }
    }
    else {
      Serial.println("Read fail");
    }
    delay(50); //Delay small amount between readings
  }
}

boolean readDistance(uint8_t deviceAddress) {

  Wire.beginTransmission(deviceAddress);
  Wire.write(0x01); //MSB
  Wire.write(0x02); //LSB
  Wire.write(7); //Data length: 7 bytes for distance data
  if (Wire.endTransmission(false) != 0) {
    return (false); //Sensor did not ACK
  }
  Wire.requestFrom(deviceAddress, (uint8_t)7); //Ask for 7 bytes

  if (Wire.available()) {
    for (uint8_t x = 0 ; x < 7 ; x++) {
      uint8_t incoming = Wire.read();

      if (x == 0) {
        //Trigger done
        if (incoming == 0x00) {
          //Serial.print("Data not valid: ");//for debugging
          valid_data = true;
          //return(false);
        }
        else if (incoming == 0x01) {
          //Serial.print("Data valid:     ");
          valid_data = true;
        }
      }
      else if (x == 2)
        distance = incoming; //LSB of the distance value "Dist_L"
      else if (x == 3)
        distance |= incoming << 8; //MSB of the distance value "Dist_H"
      else if (x == 4)
        strength = incoming; //LSB of signal strength value
      else if (x == 5)
        strength |= incoming << 8; //MSB of signal strength value
      else if (x == 6)
        rangeType = incoming; //range scale
    }
  }
  else {
    Serial.println("No wire data avail");
    return (false);
  }
  return (true);
}

void configTF(uint8_t addr, uint8_t RegH, uint8_t RegL, uint8_t len, uint8_t DataL, uint8_t DataH = 0x00){
  if ( addr < 0x10 || 0x78 <= addr){
    Serial.println(F("Address not in range: 0x10 - 0x78"));
    return;
  }
  if ( len < 1 || 2 < len){
    Serial.println(F("Incorrect data length."));
    return;
  }
  Wire.beginTransmission(addr); // Begin a transmission to the I2C Slave device with the given address.
  Wire.write(RegH);
  Wire.write(RegL);
  Wire.write(len);
  if (Wire.endTransmission(false) != 0) { // Send a START Sign
    return (false); //Sensor did not ACK
  }
  Wire.beginTransmission(addr);
  if (len == 2){ // In case of a 16 bit value, write the higher byte first
    Wire.write(DataH);
  }
  Wire.write(DataL);
  Wire.endTransmission(true);  // Send a STOP Sign
  delay(500);
}

uint8_t getHexChar(char MS, char LS) {
  uint8_t val = 0x00;
  // MS 4 bits
  if (48 <= int(MS) && int(MS) <= 57) {
    val |= ((int)MS - 48) << 4;
  }
  else if (97 <= int(MS) && int(MS) <= 102) {
    val |= ((int)MS - 87) << 4;
  }
  else {
    return 0x00;
  }
  // LS 4 bits
  if (48 <= int(LS) && int(LS) <= 57) {
    val |= ((int)LS - 48);
  }
  else if (97 <= int(LS) && int(LS) <= 102) {
    val |= (int)LS - 87;
  }
  else {
    return 0x00;
  }
  return val;
}

void RESET_I2CBus() {
  //send 0x06 to address 0x00
  Wire.beginTransmission(0);
  Wire.write(6);
  Wire.endTransmission(1);
  delay(1000);
}

void serialEvent() {
  if (Serial.available()) {
    String inc = Serial.readStringUntil('\n');
    Serial.println(inc);
    if (inc.length() == 7 && inc.substring(0, 4) == "addr") { // e.g. "addr 1A"
      // Doesn't change the address on the TFMini but tells the controller which address to use
      address = getHexChar(inc.charAt(5), inc.charAt(6));
      Serial.println(address, HEX);
    }
    else if ( inc.length() == 14 && inc.substring(0, 6) == "config") { // e.g. "config ABCD 12"
      uint8_t len = 1; // 1 byte of Data to send
      uint8_t RegH = getHexChar(inc.charAt(7), inc.charAt(8));
      uint8_t RegL = getHexChar(inc.charAt(9), inc.charAt(10));
      uint8_t data = getHexChar(inc.charAt(12), inc.charAt(13));
      configTF(address,RegH,RegL,len,data);
    }
    else if ( inc.length() == 16 && inc.substring(0, 6) == "config") { // e.g. "config ABCD 1234"
      uint8_t len = 2; // 2 bytes of Data to send
      uint8_t RegH = getHexChar(inc.charAt(7), inc.charAt(8));
      uint8_t RegL = getHexChar(inc.charAt(9), inc.charAt(10));
      uint8_t dataH = getHexChar(inc.charAt(12), inc.charAt(13));
      uint8_t dataL = getHexChar(inc.charAt(14), inc.charAt(15));
      configTF(address,RegH,RegL,len,dataL,dataH);
    }
    else if ( inc.length() == 5 && inc.substring(0,5) == "reset") {
      RESET_I2CBus();
    }
    else {
      Serial.println("Incompatible command.");
    }
  }
}
