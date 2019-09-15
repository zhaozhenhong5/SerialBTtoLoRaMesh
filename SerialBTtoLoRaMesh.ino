//This example code is in the Public Domain (or CC0 licensed, at your option.)
//By Evandro Copercini - 2018
//
//This example creates a bridge between Serial and Classical Bluetooth (SPP)
//and also demonstrate that SerialBT have the same functionalities of a normal Serial

#include "BluetoothSerial.h"

const int readPin = 32; // Use GPIO number. See ESP32 board pinouts
const int LED = 27; // Could be different depending on the dev board. I used the DOIT ESP32 dev board.
const int PowerButtonPin = 4;     // the number of the PowerButtonPin pin
const int button1Pin = 12;     // the number of the button1Pin pin
const int button2Pin = 14;     // the number of the button2Pin pin
const int HoldPin =  22;      // the number of the PowerHold pin
const int ledPin =  26;      // the number of the LED pin
const int GPScontrolPin = 5;     // the number of the GPScontrolPin pin
const int bzPin = 21;     // the number of the bzPin pin

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

HardwareSerial mySerial1(1);
HardwareSerial mySerial2(2);
BluetoothSerial SerialBT;

void setup() {
//  Serial.begin(115200);
mySerial1.begin(115200, SERIAL_8N1, 18, 19);
mySerial2.begin(9600, SERIAL_8N1, 16, 17);
Serial.begin(115200,SERIAL_8N1,RX,TX);

  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(button1Pin, INPUT_PULLUP);
  pinMode(button2Pin, INPUT_PULLUP);
  
  pinMode(PowerButtonPin, INPUT);
  pinMode(HoldPin, OUTPUT);
  digitalWrite(HoldPin, LOW);
  
  pinMode(LED, OUTPUT);
  
  pinMode(bzPin, OUTPUT);

  pinMode(GPScontrolPin, OUTPUT);
  digitalWrite(GPScontrolPin, HIGH);

    
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
}

void loop() {
   if(digitalRead(PowerButtonPin))
  {
    digitalWrite(HoldPin, HIGH);
    // turn LED on:
    digitalWrite(ledPin, HIGH);
  }

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (digitalRead(button2Pin)) 
  {
    // turn LED on:
    digitalWrite(bzPin, HIGH);
  } else {
    // turn LED off:
    digitalWrite(bzPin, LOW);
  }
/* 
  // read from port 2, send to port 0:
  if (mySerial2.available()) {
    int inByte = mySerial2.read();
    Serial.write(inByte);
  }    
*/ 
  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (digitalRead(button1Pin)) 
  {
    // turn LED on:
    digitalWrite(ledPin, HIGH);
  } else {
    // turn LED off:
    digitalWrite(ledPin, LOW);
  }
    
  
  // read from port 1, send to port 0:

  if (Serial.available()) {
    int inByte = Serial.read();
    mySerial1.write(inByte);
  }

  if (mySerial1.available()) {
    int inByte = mySerial1.read();
    Serial.write(inByte);
  }
/*
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  if (SerialBT.available()) {
    Serial.write(SerialBT.read());
  }

  while (Serial.available())
  {
    // Delay to wait for enough input, since we have a limited transmission buffer
    delay(2);

    uint8_t buf[64];
    int count = Serial.readBytes(buf, sizeof(buf));
    SerialBT.write( buf, count );
  }
  
  // Forward from BLEUART to HW Serial
  while ( SerialBT.available() )
  {
    uint8_t ch;
    ch = (uint8_t) SerialBT.read();
    Serial.write(ch);
  }    */
//  delay(20);
}
