
// for TLV  experiment , Miriam Hahashmonait 12/3/2020
// two sht T and Rh sensors on i2c
// two analog temp sensors on one-wire
//1 relays to control heating room, 
//2 mosfets to control fans (low-side switches, ideally on pwm channels) 


#include <IoTGuru.h>

#include <ESP8266WiFi.h>
//#include <WiFi.h>

#include <pins_arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <ClosedCube_SHT31D.h>
//#include "RTClib.h"

#include <DallasTemperature.h>

#include <OneWire.h>


#include "secrets.h"

char ssid[] = SECRET_SSID;   // your network SSID (name)
char pass[] = SECRET_PASS;   // your network password
int keyIndex = 0;            // your network key index number (needed only for WEP)
WiFiClient  client;



String userShortId    = USER_ID;
String deviceShortId  = DEVICE_ID;
String deviceKey      = DEVICE_KEY;
IoTGuru iotGuru = IoTGuru(userShortId, deviceShortId, deviceKey);

//if you dont have any sensors , test program with test_mode=1
boolean test_mode=0;

// ********************
// Experiment and data logger settings
#define LOG_INTERVAL  60000  // mills between entries (reduce to take more/faster data)
#define SYNC_INTERVAL 15000 // mills between calls to flush() - to write data to the card

#define LOGTOFILE 0 // select whether to log data to SD card or not
#define ECHO_TO_SERIAL   1 // echo data to serial port
#define WAIT_TO_START    0 // Wait for serial input in setup()


// SHT-31 settings
#define SHT31D_SDA_PIN D14 //blue, connect to SDO
#define SHT31D_SCL_PIN D15 //brown, connect to SDI
#define USING_SHT31 1

ClosedCube_SHT31D sht31d_1;
ClosedCube_SHT31D sht31d_2;

// DS18B20 temp sensor settings
#define ONE_WIRE_BUS_PIN D8// was 48   //maybe 20??
#define TEMPERATURE_PRECISION 9
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS_PIN);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device addresses
//DeviceAddress insideThermometer, outsideThermometer;

boolean do_scale=0;

//relay addresses
/* #define RELAY1_PIN 3
#define RELAY2_PIN 4
#define RELAY3_PIN 5
#define RELAY4_PIN 44
 */
//fan addresses
#define FAN1_PIN D1
#define FAN2_PIN D2

#define FAN4_PIN 2

#define LEDpin 7 // pin for green LED
boolean blinkLED = false;
int LEDstate = LOW;

const long interval = 500; 



unsigned long previousLEDMillis = 0;
unsigned long previousLogMillis = 0;
uint32_t syncTime = 0; // time of last sync()

volatile unsigned long nextSendUptime = 0;

//measurements_s measurements;

// *******************************************************************
void loop() {
    
	iotGuru.loop();
    delay(1);
	if (millis()%1000) activate_output_pins(); //update state every second

    
    if (nextSendUptime < millis()) {
        nextSendUptime = millis() + 60000;
		 int rtn = I2C_ClearBus(); // clear the I2C bus first before calling Wire.begin()
		  if (rtn != 0) {
			Serial.println(F("I2C bus error. Could not clear"));
			if (rtn == 1) {
			  Serial.println(F("SCL clock line held low"));
			} else if (rtn == 2) {
			  Serial.println(F("SCL clock line held low by slave clock stretch"));
			} else if (rtn == 3) {
			  Serial.println(F("SDA data line held low"));
			}
		  } else { // bus clear
			// re-enable Wire
			// now can start Wire Arduino master
			Wire.begin();
		  }
		do_measurements();
		echo_to_serial();
		send_to_iotGuru();
   // activate_output_pins(&measurements);

     
    }
}

/**
 * I2C_ClearBus
 * (http://www.forward.com.au/pfod/ArduinoProgramming/I2C_ClearBus/index.html)
 * (c)2014 Forward Computing and Control Pty. Ltd.
 * NSW Australia, www.forward.com.au
 * This code may be freely used for both private and commerical use
 */

#include <Wire.h>

/**
 * This routine turns off the I2C bus and clears it
 * on return SCA and SCL pins are tri-state inputs.
 * You need to call Wire.begin() after this to re-enable I2C
 * This routine does NOT use the Wire library at all.
 *
 * returns 0 if bus cleared
 *         1 if SCL held low.
 *         2 if SDA held low by slave clock stretch for > 2sec
 *         3 if SDA held low after 20 clocks.
 */
int I2C_ClearBus() {
#if defined(TWCR) && defined(TWEN)
  TWCR &= ~(_BV(TWEN)); //Disable the Atmel 2-Wire interface so we can control the SDA and SCL pins directly
#endif

  pinMode(SDA, INPUT_PULLUP); // Make SDA (data) and SCL (clock) pins Inputs with pullup.
  pinMode(SCL, INPUT_PULLUP);

  delay(2500);  // Wait 2.5 secs. This is strictly only necessary on the first power
  // up of the DS3231 module to allow it to initialize properly,
  // but is also assists in reliable programming of FioV3 boards as it gives the
  // IDE a chance to start uploaded the program
  // before existing sketch confuses the IDE by sending Serial data.

  boolean SCL_LOW = (digitalRead(SCL) == LOW); // Check is SCL is Low.
  if (SCL_LOW) { //If it is held low Arduno cannot become the I2C master. 
    return 1; //I2C bus error. Could not clear SCL clock line held low
  }

  boolean SDA_LOW = (digitalRead(SDA) == LOW);  // vi. Check SDA input.
  int clockCount = 20; // > 2x9 clock

  while (SDA_LOW && (clockCount > 0)) { //  vii. If SDA is Low,
    clockCount--;
  // Note: I2C bus is open collector so do NOT drive SCL or SDA high.
    pinMode(SCL, INPUT); // release SCL pullup so that when made output it will be LOW
    pinMode(SCL, OUTPUT); // then clock SCL Low
    delayMicroseconds(10); //  for >5uS
    pinMode(SCL, INPUT); // release SCL LOW
    pinMode(SCL, INPUT_PULLUP); // turn on pullup resistors again
    // do not force high as slave may be holding it low for clock stretching.
    delayMicroseconds(10); //  for >5uS
    // The >5uS is so that even the slowest I2C devices are handled.
    SCL_LOW = (digitalRead(SCL) == LOW); // Check if SCL is Low.
    int counter = 20;
    while (SCL_LOW && (counter > 0)) {  //  loop waiting for SCL to become High only wait 2sec.
      counter--;
      delay(100);
      SCL_LOW = (digitalRead(SCL) == LOW);
    }
    if (SCL_LOW) { // still low after 2 sec error
      return 2; // I2C bus error. Could not clear. SCL clock line held low by slave clock stretch for >2sec
    }
    SDA_LOW = (digitalRead(SDA) == LOW); //   and check SDA input again and loop
  }
  if (SDA_LOW) { // still low
    return 3; // I2C bus error. Could not clear. SDA data line held low
  }

  // else pull SDA line low for Start or Repeated Start
  pinMode(SDA, INPUT); // remove pullup.
  pinMode(SDA, OUTPUT);  // and then make it LOW i.e. send an I2C Start or Repeated start control.
  // When there is only one I2C master a Start or Repeat Start has the same function as a Stop and clears the bus.
  /// A Repeat Start is a Start occurring after a Start with no intervening Stop.
  delayMicroseconds(10); // wait >5uS
  pinMode(SDA, INPUT); // remove output low
  pinMode(SDA, INPUT_PULLUP); // and make SDA high i.e. send I2C STOP control.
  delayMicroseconds(10); // x. wait >5uS
  pinMode(SDA, INPUT); // and reset pins as tri-state inputs which is the default state on reset
  pinMode(SCL, INPUT);
  return 0; // all ok
}



void callback(const char* nodeShortId, const char* fieldName, const char* message) {
   
   String received, is;
  
   //Serial.print(nodeShortId);Serial.print(" - ");Serial.print(fieldName);Serial.print(": ");Serial.println(message);
	
	
   for (int m=0; m< NumFields; m++) {
	   
		
       // using this order for efficiency: as the condition assesment is lazy, we will only strcmp (which is heavier) on output_pin fields      
     if (measurements_array[m].output_pin!=NON_EDITABLE) {
		 is=String(node_ID[measurements_array[m].node]);
		 received=String(nodeShortId);
	      if (received.equals(is) ){
			  is=measurements_array[m].fieldname;
			  received=fieldName;
		      if (received.equalsIgnoreCase(is))
				{
					received=String(message);
					measurements_array[m].value=received.toInt();
						   
				/*	Serial.print("Changing pin ");
					Serial.print(measurements_array[m].output_pin);
					Serial.print(" to value ");
					Serial.println(measurements_array[m].value);*/
	 }}}
		 //iotGuru.sendMqttValue(node_ID[measurements_array[m].node],measurements_array[m].fieldname, measurements_array[m].value);      
	}
}

void activate_output_pins(){
  
  int  scaled_value;
  for (int m=0; m< NumFields; m++) {
       // using this order for efficiency: as the condition assesment is lazy, we will only strcmp (which is heavier) on output_pin fields      
     if (measurements_array[m].output_pin!=NON_EDITABLE)
     {
        scaled_value=(int)(measurements_array[m].value*measurements_array[m].scale);
        
        analogWrite(measurements_array[m].output_pin,scaled_value);
        /*Serial.print("setting pin ");
        Serial.print(measurements_array[m].output_pin);
        Serial.print(" to value ");
        Serial.println(scaled_value); */
        
     }
    }
}



void do_measurements()
{
  

//  Serial.print("doing measurements\n");
  if (test_mode) //fake some measurements
  {
	  
    measurements_array[SHD31d_1_temp].value=1.2;
    measurements_array[SHD31d_1_humidity].value=2.1;
    measurements_array[SHD31d_2_temp].value=1.3;
    measurements_array[SHD31d_2_humidity].value=3.1;

    measurements_array[TempInside_temp].value = 9.1;
    measurements_array[TempOutside_temp].value = 9.2;
    measurements_array[TempPassiveWall_temp].value = 9.3;
    measurements_array[TempActiveWall_temp].value = 9.4;
	
    previousLogMillis = millis();
  }
  else
  {
    
	sensors.requestTemperatures(); // Send the command to get temperatures
    
    measurements_array[TempInside_temp].value = sensors.getTempCByIndex(0);
    measurements_array[TempOutside_temp].value = sensors.getTempCByIndex(1);
    measurements_array[TempPassiveWall_temp].value = sensors.getTempCByIndex(2);
	measurements_array[TempActiveWall_temp].value = sensors.getTempCByIndex(3);
 #ifdef USING_SHT31   
    // **********************
    // Read data from the sensors
    SHT31D result1 = sht31d_1.periodicFetchData();
    if (result1.error == NO_ERROR) {
        measurements_array[SHD31d_1_temp].value=result1.t;
        measurements_array[SHD31d_1_humidity].value=result1.rh;
    } // end NO_ERROR
    SHT31D result2 = sht31d_2.periodicFetchData();
    if (result2.error == NO_ERROR) {
        measurements_array[SHD31d_2_temp].value=result2.t;
        measurements_array[SHD31d_2_humidity].value=result2.rh;
    }
#endif
   }  

  // save the last time you logged
  previousLogMillis = millis();


}


void send_to_iotGuru()
{
	 for (int m=0; m< NumFields; m++) {
		 
		 iotGuru.sendMqttValue(node_ID[measurements_array[m].node],measurements_array[m].fieldname, measurements_array[m].value);
		 
	 }
	  
}

void header_to_serial()
{
	 for (int m=0; m< NumFields; m++) {
		 Serial.print(node_names[measurements_array[m].node]);
		 
		 Serial.print("\t");
	 }
	 Serial.println();
	 
	 for (int m=0; m< NumFields; m++) {
		 Serial.print(measurements_array[m].fieldname);
		 Serial.print("[");
		 Serial.print(measurements_array[m].unit);
		 Serial.print("],\t");
	 }
	  Serial.println("\r\r"); // remove the \t and comma at the end
}
void echo_to_serial()
{
	 for (int m=0; m< NumFields; m++) {
		 Serial.print(measurements_array[m].value);
		 Serial.print(",\t");
	 } 
	  
    Serial.println("\r\r");
}

void do_blink_led()
{/*
    // LED blink for SD write warning
    unsigned long prevLEDMillis;
    LEDstate - measurements->LEDstate;
    prevLEDMillis = measurements->previousLEDMillis;
    unsigned long currentLEDMillis = millis();
    if (currentLEDMillis - previousLEDMillis >= interval) {
      // save the last time you blinked the LED
      previousLEDMillis = currentLEDMillis;
   
      // if the LED is off turn it on and vice-versa:
      if (LEDstate == LOW) {
        LEDstate = HIGH;
      } else {
        LEDstate = LOW;
      }
   
      // set the LED with the ledState of the variable:
     // digitalWrite(LEDpin, LEDstate);
    }
  
    else {
      //LEDstate = HIGH;
      //digitalWrite(LEDpin, HIGH);
    }
    measurements->LEDstate=LEDstate;
    measurements->previousLEDMillis=previousLEDMillis;*/
}


  
//-----------------------------------------------------------------------------
// Error function
void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
  
  //digitalWrite(LEDpin, LOW);

//  while(1);
}

//------------------------------------------------------------------------------
// call back for file timestamps
void dateTime(uint16_t* d, uint16_t* t) {
 char timestamp[20];
// DateTime now = RTC.now();
//sprintf(timestamp, "%02d:%02d:%02d %2d/%2d/%2d \n", now.hour(),now.minute(),now.second(),now.day(),now.month(),now.year()-2000);
// // return date using FAT_DATE macro to format fields
// *d = FAT_DATE(now.year(), now.month(), now.day());
//
// // return time0 u0sing FAT_TIME macro to format fields
// *t = FAT_TIME(now.hour(), now.minute(), now.second());
}
//------------------------------------------------------------------------------


void setup() {
  Serial.begin(115200);
  //pinMode(A4, INPUT);
  //pinMode(A5, INPUT);



    WiFi.begin(ssid, pass);
    while (WiFi.status() != WL_CONNECTED) {
        delay(50);
        Serial.print(".");
    }
    Serial.println("Connected");
    

  //Serial3.begin(115200);
 // WiFi.init(&Serial3);
    /**
     * Set the callback function.
     */
    iotGuru.setCallback(&callback);
    /**
     * Set the debug printer (optional).
     */
 //   iotGuru.setDebugPrinter(&Serial);
    /**
     * Set the network client.
     */
    iotGuru.setNetworkClient(&client);
    

//**************************
  for (int m=0; m< NumFields; m++) 
  {
     if (measurements_array[m].output_pin!=NON_EDITABLE)
     {
		 Serial.print("Setting ");
		 Serial.print(measurements_array[m].fieldname);
		 Serial.print(" as output on pin ");
		 Serial.println(measurements_array[m].output_pin);
         pinMode(measurements_array[m].output_pin,OUTPUT);
     }
  }

// SHT-31 *************************
  pinMode(SHT31D_SDA_PIN, INPUT);
  pinMode(SHT31D_SCL_PIN, INPUT);
  Serial.println("Initializing ClosedCube SHT31-D");

  sht31d_1.begin(0x44);
  Serial.print("sht31d Serial #");
  Serial.println(sht31d_1.readSerialNumber());

  sht31d_2.begin(0x45);
  Serial.print("sht31d Serial #");
  Serial.println(sht31d_2.readSerialNumber());

  // if (sht31d.periodicStart(REPEATABILITY_HIGH, FREQUENCY_10HZ) != NO_ERROR)
  //   Serial.println("[ERROR] Cannot start periodic mode");
  if (sht31d_1.periodicStart(REPEATABILITY_HIGH, FREQUENCY_1HZ) != NO_ERROR)
  {
     Serial.println("1:[ERROR] Cannot start periodic mode");
  }
  else
  {
      Serial.println("Initialized periodic start");
  }
  if (sht31d_2.periodicStart(REPEATABILITY_HIGH, FREQUENCY_1HZ) != NO_ERROR)
  {
     Serial.println("2:[ERROR] Cannot start periodic mode");
  }
  else
  {
      Serial.println("Initialized periodic start");
  }

  // print header
  header_to_serial();

 // if all went well, light up green LED
  //pinMode(LEDpin, OUTPUT);
  //digitalWrite(LEDpin, HIGH); 

  //start the onewire sensors (temp in our case) 
  if (! test_mode)  { // without sensors , next line hangs
    sensors.begin();
    sensors.requestTemperatures();
  delay(1000);
	Serial.println(sensors.getDeviceCount());
  }

}
