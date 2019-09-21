# Data Logger (and using cool sensors!)

*A lab report by Frans Fourie. Student.*

## In The Report

Include your responses to the bold questions on your own fork of [this lab report template](https://github.com/FAR-Lab/IDD-Fa18-Lab2). Include snippets of code that explain what you did. Deliverables are due next Tuesday. Post your lab reports as README.md pages on your GitHub, and post a link to that on your main class hub page.

For this lab, we will be experimenting with a variety of sensors, sending the data to the Arduino serial monitor, writing data to the EEPROM of the Arduino, and then playing the data back.

## Part A.  Writing to the Serial Monitor
 
**a. Based on the readings from the serial monitor, what is the range of the analog values being read?**

0 - 1023
 
**b. How many bits of resolution does the analog to digital converter (ADC) on the Arduino have?**

10 bits

## Part B. RGB LED

I used the FSR to control the color of the RGB LED by varying the color depending on how hard you press the FSR.

The following is a link to the youtube video:
https://youtu.be/oulqR6I8oBw

The following is the code used:
```
int redPin = 11;
int greenPin = 10;
int bluePin = 9;
 
//uncomment this line if using a Common Anode LED
//#define COMMON_ANODE

int redC = 0;
int redG = 0;
int redB = 0;
int fsrAnalogPin = 0; // FSR is connected to analog 0
int fsrReading;      // the analog reading from the FSR resistor divider
int LEDbrightness;
 
void setup()
{
  Serial.begin(9600);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);  
}
 
void loop()
{
  fsrReading = analogRead(fsrAnalogPin);
  Serial.print("Analog reading = ");
  Serial.println(fsrReading);
  LEDbrightness = map(fsrReading, 0, 1023, 0, 765);
  if (LEDbrightness <= 255){
    setColor(LEDbrightness, 0, 0);
  }
  if ((LEDbrightness > 255) || (LEDbrightness <= 510)){
    redC = 510 - LEDbrightness;
    redG = LEDbrightness - 255;
    setColor(redC, redG, 0);
  }
  if ((LEDbrightness > 510) || (LEDbrightness <= 765)){
    redG = 765 - LEDbrightness;
    redB = LEDbrightness - 510;
    setColor(0, redG, redB);
  }

}
 
void setColor(int red, int green, int blue)
{
  #ifdef COMMON_ANODE
    red = red - 255;
    green = green - 255;
    blue = blue - 255;
  #endif
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);  
}
```

## Part C. Voltage Varying Sensors 
 
### 1. FSR, Flex Sensor, Photo cell, Softpot

**a. What voltage values do you see from your force sensor?**

I got between 0 - 1002 which correlates to 0V - 4.89736V

**b. What kind of relationship does the voltage have as a function of the force applied? (e.g., linear?)**

It's an almot linear relationship except at low force values where a small amount of force reduces the resistance greatly. As resistance decreases the voltage will increase. Thus there is a linear relationship between force and voltage as, as the force increases so does the voltage.

The following is the graph of resistance vs force of the FSR
![](Images/graphFSR.PNG)


**c. Can you change the LED fading code values so that you get the full range of output voltages from the LED when using your FSR?**

Changing the line of code ```LEDbrightness = map(fsrReading, 0, 1023, 0, 255);``` to ```LEDbrightness = map(fsrReading, 0, 1000, 0, 255);``` this will allow for all 256 values of the LED intensitiy to be reached.

**d. What resistance do you need to have in series to get a reasonable range of voltages from each sensor?**

For the softpot reistor I put a 10k ohm resistor in series between power and the one side leg pin of the resistor and I also put a 10k ohm resistor between ground and the other side leg of the softpot resistor. I did this as it was what was recommended by the datasheet. With this I got voltages between 1.8V - 3.2V.

For the photo cell I put a 10k ohm resistor paralel with the analog input pin and the one leg of the photo cell. The other leg of the photo cell was connected to 5V. With the lights off I got voltages as low as 0.0879V and by shinning a torch on the resistor I got voltage readings as high as 4.8387V.



**e. What kind of relationship does the resistance have as a function of stimulus? (e.g., linear?)**

There is an Inverse correlation between stimulus and resistance. As the stimulus such as light on photo cell increases so does the resistance decrease.


### 2. Accelerometer
 
**a. Include your accelerometer read-out code in your write-up.**

The following is a Video of my accelerometer working:
https://youtu.be/DFjhx0JTU-8

The following is my code for the accelerometer with the RGB light and the LCD display:
```
// Basic demo for accelerometer readings from Adafruit LIS3DH
#include <LiquidCrystal.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

// Used for software SPI
#define LIS3DH_CLK 13
#define LIS3DH_MISO 12
#define LIS3DH_MOSI 11
// Used for hardware & software SPI
#define LIS3DH_CS 10

// software SPI
//Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);
// hardware SPI
//Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS);
// I2C
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

int redPin = 6;
int greenPin = 10;
int bluePin = 9;
int LEDbrightness;

void setup(void) {
#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif

  Serial.begin(9600);
  Serial.println("LIS3DH test!");
  lcd.begin(16, 2);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT); 
  
  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1);
  }
  Serial.println("LIS3DH found!");
  
  lis.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!
  
  Serial.print("Range = "); Serial.print(2 << lis.getRange());  
  Serial.println("G");
}

void loop() {
  lis.read();      // get X Y and Z data at once
  // Then print out the raw data
  Serial.print("X:  "); Serial.print(lis.x); 
  Serial.print("  \tY:  "); Serial.print(lis.y); 
  Serial.print("  \tZ:  "); Serial.print(lis.z); 

  /* Or....get a new sensor event, normalized */ 
  sensors_event_t event; 
  lis.getEvent(&event);
  
  /* Display the results (acceleration is measured in m/s^2) */
  lcd.setCursor(0, 0);
  lcd.print("x=" + String(event.acceleration.x));
  lcd.setCursor(8, 0);
  lcd.print("y=" + String(event.acceleration.y));
  lcd.setCursor(0, 2);
  lcd.print("z=" + String(event.acceleration.z));
  lcd.setCursor(7, 2);
  lcd.print("m/s^2");  
  Serial.print("\t\tX: "); Serial.print(event.acceleration.x);
  Serial.print(" \tY: "); Serial.print(event.acceleration.y); 
  Serial.print(" \tZ: "); Serial.print(event.acceleration.z); 
  Serial.println(" m/s^2 ");
  setColor(event.acceleration.x, event.acceleration.y, event.acceleration.z);

  Serial.println();
 
  delay(200); 
}


void setColor(int red, int green, int blue)
{
  #ifdef COMMON_ANODE
    red = 255 - red;
    green = 255 - green;
    blue = 255 - blue;
  #endif
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);  
}
```

### 3. IR Proximity Sensor

**a. Describe the voltage change over the sensing range of the sensor. A sketch of voltage vs. distance would work also. Does it match up with what you expect from the datasheet?**

**b. Upload your merged code to your lab report repository and link to it here.**

## Optional. Graphic Display

**Take a picture of your screen working insert it here!**

## Part D. Logging values to the EEPROM and reading them back
 
### 1. Reading and writing values to the Arduino EEPROM

**a. Does it matter what actions are assigned to which state? Why?**

**b. Why is the code here all in the setup() functions and not in the loop() functions?**

**c. How many byte-sized data samples can you store on the Atmega328?**

**d. How would you get analog data from the Arduino analog pins to be byte-sized? How about analog data from the I2C devices?**

**e. Alternately, how would we store the data if it were bigger than a byte? (hint: take a look at the [EEPROMPut](https://www.arduino.cc/en/Reference/EEPROMPut) example)**

**Upload your modified code that takes in analog values from your sensors and prints them back out to the Arduino Serial Monitor.**

### 2. Design your logger
 
**a. Insert here a copy of your final state diagram.**

### 3. Create your data logger!
 
**a. Record and upload a short demo video of your logger in action.**
