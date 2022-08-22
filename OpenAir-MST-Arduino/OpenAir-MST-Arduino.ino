/*
  Microcontroller: Arduino Nano 33 IoT (SAMD)wea
 
  Adapted from Basic Arduino example for K-Series sensor
  Created by Jason Berger / Co2meter.com
  Updated for Nano 33 IOT by replacing
    software serial with Serial1 (built-in second UART on pins 0 and 1)
  Added OLED display
  Added transistor-control of pump for gas sampling
  Added data push to Adafruit IO via MQTT
  Added Bosch BME280 sensor for temp, pressure, humidity
  Added SCD30 as second CO2 sensor inside chamber
  4/2022  Added second K30 for measuring pre- and post-chamber CO2 levels
          Added Omron flow sensor for verifying flow rate through test  
*/

//System can be configured to run the pump intermittently, to maximize motor life (rated at a few hundred hours)
//This pin drives the gate of the mosfet that activates the pump.
//Pump can also be momentarily activated with a pushbutton; always on via a jumper; or powered from a separate power supply
#define pumpPin 9

//Global settings for sample frequency, pump, wifi, BME280, SCD30
unsigned long int sampleIntervalMillis = 5 * 1000, //X second sample interval
                  lastSampleMillis = -sampleIntervalMillis;        //track last time sample was recorded, startoff with immediate sample

boolean usePump = false, //activate pump before/after sample?
        pumpSoftStart = false; //ramp pump on/off?
int     preSampleMillis = 2000,
        postSampleMillis = 0;

boolean useWIFI = false;
boolean useBME280 = true; //Can be configured to read additional humidity/temperature/pressure sensor on I2C bus
boolean useSCD30 = true;  //Can be configured to read additional CO2 data from SCD30 on I2C bus

//Second Nano hardware serial port for interfacting with second K30
#include <Wire.h>
#include "wiring_private.h"
Uart softSerial (&sercom0, 5, 6, SERCOM_RX_PAD_1, UART_TX_PAD_0); // Create the new UART instance assigning it to pin 5 and 6
void SERCOM0_Handler() {  softSerial.IrqHandler(); }

//K30 request packet and byte array for holding response
byte readCO2[] = {0xFE, 0X44, 0X00, 0X08, 0X02, 0X9F, 0X25};  //Command packet to read Co2 (see app note)
byte response[] = {0, 0, 0, 0, 0, 0, 0}; //create an array to store the response
byte response2[] = {0, 0, 0, 0, 0, 0, 0}; //create an array to store the response
//multiplier for value. default is 1. set to 3 for K-30 3% and 10 for K-33 ICB
int valMultiplier = 1;
boolean error = false;
boolean error2 = false;

//Omron D6 flow sensor reports flow rate as analog outoput. Has non-linear output. Output will be reported in raw DAC and volts.
//See details: https://omronfs.omron.com/en_US/ecb/products/pdf/en-d6f_p.pdf
boolean useFlowSensor;
#define FLOWINPUT A0

//built in LED state - will blink between samples to indicate system is running
boolean led = false;

//I2C OLED Display
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, A6);
String errorText = "";

//Wifi and MQTT service:
#include <SPI.h>
#include <WiFiNINA.h>
#include "config.h"
int count = 0;
int status = WL_IDLE_STATUS;
//Set up the wifi client
WiFiClient client;
//Note, since we're not using an Adafruit board (e.g. feather) making do with their simpler/partial MQTT library. No group support.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish m_co2 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/openair.co2");
Adafruit_MQTT_Publish m_pressure = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/openair.pressure");
Adafruit_MQTT_Publish m_humidity = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/openair.relative-humidity");
Adafruit_MQTT_Publish m_temperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/openair.temperature");

//Bosch BME280 temp/humidity/pressure:
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C

//SCD30
#include <Adafruit_SCD30.h>
Adafruit_SCD30  scd30;

//Globals for sensor data
float temp, pressure, humidity;
int flowReading;
float flowVolts;
unsigned long valCO2; //first K30
unsigned long valCO2b;//second K30

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);         //Opens the main serial port to communicate with the computer
  Serial1.begin(9600);    //Opens the virtual serial port with a baud of 9600
  softSerial.begin(9600); //Open a THIRD serial port using SERCOM0 on pins 6(TX) and 5(RX) - note different pin order from other serial
  pinPeripheral(5, PIO_SERCOM_ALT);
  pinPeripheral(6, PIO_SERCOM_ALT);
  
  pinMode(LED_BUILTIN, OUTPUT);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for (;;) {
      digitalWrite(LED_BUILTIN, led); // Don't proceed, loop forever
      led = !led;
      delay(100);
    }
  }

  // Title screen
  display.clearDisplay();
  showTitle();
  delay(2000);
  int msgDelay = 1000, blankTime = 250;

  //Test the pump
  blankScreen(250);
  if (usePump) {
    displayText("Testing Pump", 1);
    display.print("Soft start: ");
    if (pumpSoftStart) display.println("yes");
    else display.println(" no");
    display.print("Pre-sample: ");
    display.print(preSampleMillis);
    display.println(" ms");
    display.print("Post-sample: ");
    display.print(postSampleMillis);
    display.println(" ms");
    display.display();
    pinMode(pumpPin, OUTPUT);
    setPump(HIGH);
    delay(2000);
    setPump(LOW);
  } else {
    displayText("Pump not selected.", 1);
    display.println("Use manual options.");
    display.display();
    delay(msgDelay);
  }

  //Online data:
  blankScreen(250);
  if (useWIFI) {
    displayText("Connecting to wifi...", 1);
    status = WiFi.begin(WIFI_SSID, WIFI_PASS);
    if ( status != WL_CONNECTED) {
      Serial.println("Couldn't get a wifi connection");
      // don't do anything else:
      while (true);
    }
    String s = "Connected to " + String(WIFI_SSID);
    display.println(s);
    display.display();
  } else {
    displayText("Wifi not selected", 1);
  }
  delay(msgDelay); //leave time for message on screen

  //BME280
  blankScreen(250);
  if (useBME280) {
    displayText("Starting BME280...", 1);
    unsigned status;
    status = bme.begin();
    if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
      Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(), 16);
      Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      Serial.print("        ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("        ID of 0x60 represents a BME 280.\n");
      Serial.print("        ID of 0x61 represents a BME 680.\n");
      display.println("Error with BME280, bypassing");
      display.display();
      useBME280 = false;
    } else {
      display.println("Started");
      display.display();
    }
  } else {
    displayText("BME280 not selected", 1);
  }
  delay(msgDelay); //leave time for message on screen

  //SCD30:
  blankScreen(250);
  if (useSCD30) {
    displayText("Starting SCD30...", 1);
    display.display();
    if (!scd30.begin()) {
      Serial.println("Failed to find SCD30 chip");
      display.print("SCD30 error.");
      useSCD30 = false;
    } else {
      if (!scd30.setMeasurementInterval(2)) {
        Serial.println("SCD30 Failed to set measurement interval"); //TODO: reflect to OLED display
        while (1) {
          delay(10); //TODO: handle
        }
      }
      Serial.print("SCD30 Measurement Interval: ");
      Serial.print(scd30.getMeasurementInterval());
      Serial.println(" seconds");
      Serial.println("done!");
      display.print("Found SCD30");
    }
    display.display();
  } else {
    displayText("SCD30 not selected", 1);
  }
  delay(msgDelay); //leave time for message on screen


  //Send "header" to the Serial monitor, in case this is captured for e.g. pasting in Excel
  Serial.println("\n\n[*** BEGIN DATA OUTPUT ***]");
  if (useBME280) { //TODO - update to handle more of the optional sensors
    Serial.print("Elapsed time (s), CO2-K30 (ppm), CO2-SCD30 (ppm), Temp-SCD30 (C), RH-SCD30 (%), ");
    Serial.println("Temp-BME280 (C), RH-BME280 (%), Pressure-BME280 (hPa)"); //column value //TODO - handle not using SCD30
  } else {
    Serial.println("Elapsed time (s), CO2 In (ppm), CO2 Out (ppm), Flow (volts)");
  }
}

void loop() {
  unsigned long currentTime = millis();
  float elapsedSec = currentTime / 1000.0;
  if (currentTime - lastSampleMillis > sampleIntervalMillis) {
    //report a sample
    lastSampleMillis = currentTime;

    if (usePump) {
      setPump(HIGH);
      delay(preSampleMillis);
    }

    sendRequest(readCO2);
    valCO2 = getValue(response);
    sendRequest2(readCO2);
    valCO2b = getValue(response2);

    flowReading = analogRead(FLOWINPUT);
    flowVolts = ((3.3/1024.0) * flowReading);

    if (!error && !error2) { //error global is flag set in getValue function
      displayCO2Data(valCO2, valCO2b);
      //Serial.print("Co2 ppm = ");
      Serial.print(elapsedSec); //1
      Serial.print(", ");//sec
      Serial.print(elapsedSec/60);//sec
      Serial.print(", ");//place holder for min
      Serial.print(valCO2); //2
      Serial.print(", ");//ppmA
      Serial.print(valCO2b); //3
      Serial.print(",\t");//ppmB
      int delta = valCO2b - valCO2;
      Serial.print(delta);
      Serial.print(", "); //place holder for delta
      Serial.print(flowVolts); //4
      Serial.print("v,\t");

      if (useSCD30) {
        if (scd30.dataReady()) {
          if (scd30.read()) {            
            Serial.print(scd30.CO2); //5
            Serial.print("ppmC, ");
            Serial.print(scd30.temperature); //6
            Serial.print("temp,\t");
            Serial.print(scd30.relative_humidity); //7
            Serial.print("%,\t");
          } else {
            Serial.print("-1, -1, -1");
          }
        }
      }

      if (useBME280) {
        temp = bme.readTemperature();
        pressure = bme.readPressure() / 100.0F;
        humidity = bme.readHumidity();

        //Add BME280 readings to Serial Monitor output, as CSV format

        Serial.print(temp); //8
        Serial.print("tempB, ");
        Serial.print(humidity); //9
        Serial.print("%B, ");
        Serial.print(pressure); //10
        Serial.print("hPa");
        Serial.println();
        displayEnviroData(temp, pressure, humidity);
      }
      else { //no additional data to report, just newline
        Serial.println();
      }
    }

    else { //getValue from K30 encountered an error
      Serial.println("K30 communications error");
      displayText("K30 ERROR", 2);
    }

    if (useWIFI) {
      MQTT_connect();
      if (! m_co2.publish((valCO2))) {
        Serial.println(F("[*MQTT CO2 publish failed*]"));
      } else {
        //Serial.println(F("[*MQTT publish OK!*]"));
      }
      if (! m_temperature.publish(temp)) {
        Serial.println(F("[*MQTT temperature publish failed*]"));
      } else {
        //Serial.println(F("[*MQTT publish OK!*]"));
      }
      if (! m_pressure.publish(pressure)) {
        Serial.println(F("[*MQTT pressure publish failed*]"));
      } else {
        //Serial.println(F("[*MQTT publish OK!*]"));
      }
      if (! m_humidity.publish(humidity)) {
        Serial.println(F("[*MQTT CO2 publish failed*]"));
      } else {
        //Serial.println(F("[*MQTT publish OK!*]"));
      }
    }

    if (usePump) {
      delay(postSampleMillis);
      setPump(LOW);
    }

    //TODO - add configurable sample frequence, and pump timing
    //delay(2000);
    digitalWrite(LED_BUILTIN, led);
    led = !led;
  }

  else {
    //waiting to take next sample, display a little animation to show things are running
    display.fillRect(0, 24, 132, 8, SSD1306_BLACK); //erase previous time w/out redrawing whole screen
    display.drawRect(0, 27, 62, 4, SSD1306_WHITE);
    display.fillRect(0, 27, 62.0 * (currentTime - lastSampleMillis) / sampleIntervalMillis, 4, SSD1306_WHITE);
    display.setCursor(64, 24);
    display.print(elapsedSec);
    display.print(" s");
    display.display();
    delay(250); //don't go crazt drawing to this screen
  }
  
}


//some helpers for writing to the OLED
//Display the latest reading
void displayCO2Data(int d, int d2) {
  display.clearDisplay();
  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  //display.cp437(true);
  //display.print(F("C02: "));
  display.println(d);
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.println(F("ppm IN >>>"));
  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setCursor(64, 0);     // Start at top-left corner
  display.print(d2);
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setCursor(64, 16);     // Start at top-left corner
  display.println(F("ppm OUT"));
  display.display();
}

void displayEnviroData(float t, float p, float h) {
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setCursor(64, 0);
  display.print(t);
  display.print(" C");
  display.setCursor(64, 8);
  display.print(p);
  display.print("hPa");
  display.setCursor(64, 16);
  display.print(h);
  display.print(" %");
  display.display();
}

//Show an OpenAir title screen
void showTitle() {
  display.clearDisplay();
  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  //display.cp437(true);
  display.println(F("OPEN Air"));
  //display.println();
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.println(F("Direct Air Carbon"));
  display.println(F("Capture"));
  display.display();
}

//Clear the display and write some text
void displayText(String s, int sz) {
  display.clearDisplay();
  display.setTextSize(sz);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.println(s);
  display.display();
}

//blank screen for an interval
void blankScreen(int d) {
  display.clearDisplay();
  display.display();
  delay(d);
}

//helpers for pump, since we might want to finesse it to increase MTBF
void setPump(int s) {
  if (s == HIGH) { //Turn on pump
    if (pumpSoftStart) {
      for (int i = 120; i <= 255; i++) {
        analogWrite(pumpPin, i);
        delay(5);
      }
    } else {
      digitalWrite(pumpPin, HIGH);
    }
  } else { //Turn off pump
    if (pumpSoftStart) {
      for (int i = 255; i >= 20; i--) {
        analogWrite(pumpPin, i);
        delay(5);
      }
      analogWrite(pumpPin, 0);
    } else {
      digitalWrite(pumpPin, LOW);
    }
  }
}

//Send a request to K30 1 on built-in serial port 
void sendRequest(byte packet[])
{
  error = false;
  while (!Serial1.available()) //keep sending request until we start to get a response
  {
    Serial1.write(readCO2, 7);
    delay(50);
  }

  int timeout = 0; //set a timeoute counter
  while (Serial1.available() < 7 ) //Wait to get a 7 byte response
  {
    timeout++;
    if (timeout > 10)   //if it takes to long there was probably an error
    {
      error = true;
      while (Serial1.available()) //flush whatever we have
        Serial1.read();
      break;                        //exit and try again
    }
    delay(50);
  }

  for (int i = 0; i < 7; i++)
  {
    response[i] = Serial1.read();
  }
}

//Send a request to K30 2 on software serial port on pins 6 and 5
void sendRequest2(byte packet[]) 
{
  error2 = false;
  while (!softSerial.available()) //keep sending request until we start to get a response
  {
    softSerial.write(readCO2, 7);
    delay(50);
  }

  int timeout = 0; //set a timeoute counter
  while (softSerial.available() < 7 ) //Wait to get a 7 byte response
  {
    timeout++;
    if (timeout > 10)   //if it takes to long there was probably an error
    {
      error2 = true;
      while (softSerial.available()) //flush whatever we have
        softSerial.read();
      break;                        //exit and try again
    }
    delay(50);
  }

  for (int i = 0; i < 7; i++)
  {
    response2[i] = softSerial.read();
  }
}

unsigned long getValue(byte packet[])
{
  int high = packet[3];                        //high byte for value is 4th byte in packet in the packet
  int low = packet[4];                         //low byte for value is 5th byte in the packet

  unsigned long val = high * 256 + low;              //Combine high byte and low byte with this formula to get value
  return val * valMultiplier;
}


//MQTT:
void MQTT_connect() {
  int8_t ret;

  // attempt to connect to Wifi network:
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(WIFI_SSID);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(WIFI_SSID, WIFI_PASS);

    // wait 10 seconds for connection:
    uint8_t timeout = 10;
    while (timeout && (WiFi.status() != WL_CONNECTED)) {
      timeout--;
      delay(1000);
    }
  }

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  //Serial.print("Connecting to MQTT... ");

  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
  }
  //Serial.println("MQTT Connected!");
}
