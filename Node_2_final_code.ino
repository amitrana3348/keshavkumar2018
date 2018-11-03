/***************************************************
  Adafruit MQTT Library ESP8266 Example

  Must use ESP8266 Arduino from:
    https://github.com/esp8266/Arduino

  Works great with Adafruit's Huzzah ESP board & Feather
  ----> https://www.adafruit.com/product/2471
  ----> https://www.adafruit.com/products/2821

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Tony DiCola for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <Wire.h>
#include <BH1750.h>

BH1750 lightMeter;

/************************* WiFi Access Point *********************************/

#define WLAN_SSID       "Robotics_Lab"
#define WLAN_PASS       "vidya123"

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    "keshavkumars"
#define AIO_KEY         "fedba84f9b414061803c924dc61cb866"

/************ Global State (you don't need to change this!) ******************/

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
// or... use WiFiFlientSecure for SSL
//WiFiClientSecure client;

// Store the MQTT server, username, and password in flash memory.
// This is required for using the Adafruit MQTT library.
const char MQTT_SERVER[]     = AIO_SERVER;
const char MQTT_USERNAME[]   = AIO_USERNAME;
const char MQTT_PASSWORD[]   = AIO_KEY;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, AIO_SERVERPORT, MQTT_USERNAME, MQTT_PASSWORD);

/****************************** Feeds ***************************************/

// Setup a feed called 'photocell' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
const char light_feed[]  = AIO_USERNAME "/feeds/light";
Adafruit_MQTT_Publish light = Adafruit_MQTT_Publish(&mqtt, light_feed);

const char gas_feed[]  = AIO_USERNAME "/feeds/gas";
Adafruit_MQTT_Publish gas = Adafruit_MQTT_Publish(&mqtt, gas_feed);

// Setup a feed called 'onoff' for subscribing to changes.
const char ONOFF_FEED1[]  = AIO_USERNAME "/feeds/onoffbutton1";
Adafruit_MQTT_Subscribe onoffbutton1 = Adafruit_MQTT_Subscribe(&mqtt, ONOFF_FEED1);

// Setup a feed called 'onoff' for subscribing to changes.
const char ONOFF_FEED2[]  = AIO_USERNAME "/feeds/onoffbutton2";
Adafruit_MQTT_Subscribe onoffbutton2 = Adafruit_MQTT_Subscribe(&mqtt, ONOFF_FEED2);

// Setup a feed called 'onoff' for subscribing to changes.
const char ONOFF_FEED3[]  = AIO_USERNAME "/feeds/onoffbutton3";
Adafruit_MQTT_Subscribe onoffbutton3 = Adafruit_MQTT_Subscribe(&mqtt, ONOFF_FEED3);


// Setup a feed called 'onoff' for subscribing to changes.
//const char ONOFF_FEED4[]  = AIO_USERNAME "/feeds/weather.sp4";
//Adafruit_MQTT_Subscribe onoffbutton4 = Adafruit_MQTT_Subscribe(&mqtt, ONOFF_FEED4);


/*************************** Sketch Code ************************************/

// Bug workaround for Arduino 1.6.6, it seems to need a function declaration
// for some reason (only affects ESP8266, likely an arduino-builder bug).
void MQTT_connect();


int relay1 = D5;  // pin of microcontroller where relay1 is connected
int relay2 = D6;  // pin of microcontroller where relay2 is connected
int relay3 = D7;  // pin of microcontroller where relay3 is connected


void setup() {
  pinMode(relay1,OUTPUT); // make the relay pin work as output pin
  pinMode(relay2,OUTPUT);// make the relay pin work as output pin
  pinMode(relay3,OUTPUT);// make the relay pin work as output pin
  digitalWrite(relay1,LOW); // make the relay pin act as LOW initially, to keep the relay off
  digitalWrite(relay2,LOW); // make the relay pin act as LOW initially, to keep the relay off
  digitalWrite(relay3,LOW); // make the relay pin act as LOW initially, to keep the relay off
  digitalWrite(LED_BUILTIN,OUTPUT); // use built in LED  for indication
  Serial.begin(115200);   // Initialize serial port for sending msg on PC for debugging
  Wire.begin(); // initialize the I2C protocol for reading data from Lux Sensor
  // On esp8266 you can select SCL and SDA pins using Wire.begin(D4, D3);

  lightMeter.begin(); // lux sensor initialization function
  delay(10);

  Serial.println(F("Adafruit MQTT demo"));  // send some data on PC to get started

  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  // connect to wifi network with given ID and password
  WiFi.begin(WLAN_SSID, WLAN_PASS);

  // if wifi is not connected, keep trying
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  // when wifi is connected, print its IP address
  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());

  // Setup MQTT subscription for onoff feed.
  mqtt.subscribe(&onoffbutton1);
  mqtt.subscribe(&onoffbutton2);
  mqtt.subscribe(&onoffbutton3);
 
}

uint32_t x=0;


// function to blink LED for indication purpose
void blink_led()
{
  digitalWrite(LED_BUILTIN,HIGH); delay(100);  digitalWrite(LED_BUILTIN,LOW); delay(100);
  digitalWrite(LED_BUILTIN,HIGH); delay(100);digitalWrite(LED_BUILTIN,LOW); delay(100);
}
void loop() 
{
  
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();

  // this is our 'wait for incoming subscription packets' busy subloop
  // try to spend your time here

  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000)))  // set subscription time to be 5 seconds
  {
    if (subscription == &onoffbutton1) // if relay1 button has changed its value
    {
      Serial.print(F("Got: "));
      Serial.println((char *)onoffbutton1.lastread);
      if(strcmp((char *)onoffbutton1.lastread,"ON") == 0)// read if the value is ON, if so, turn on relay
      {
        Serial.println("Checked");  
        digitalWrite(relay1,HIGH);
      }
      if(strcmp((char *)onoffbutton1.lastread,"OFF") == 0) // if the value is OFF, turn off the relay
      {
        Serial.println("Checked");  
        digitalWrite(relay1,LOW);
      }
    }
    if (subscription == &onoffbutton2) // same as above for relay2
    {
      Serial.print(F("Got: "));
      Serial.println((char *)onoffbutton2.lastread);
      if(strcmp((char *)onoffbutton2.lastread,"ON") == 0)
      {
        Serial.println("Checked");  
        digitalWrite(relay2,HIGH);
      }
      if(strcmp((char *)onoffbutton2.lastread,"OFF") == 0)
      {
        Serial.println("Checked");  
        digitalWrite(relay2,LOW);
      }
    }
    if (subscription == &onoffbutton3) // same as above for relay 3
    {
      Serial.print(F("Got: "));
      Serial.println((char *)onoffbutton3.lastread);
      if(strcmp((char *)onoffbutton3.lastread,"ON") == 0)
      {
        Serial.println("Checked");  
        digitalWrite(relay3,HIGH);
      }
      if(strcmp((char *)onoffbutton3.lastread,"OFF") == 0)
      {
        Serial.println("Checked");  
        digitalWrite(relay3,LOW);
      }
    }
    
  }
  
  uint16_t lux = lightMeter.readLightLevel();   // read light level
  
  
  // print on serial port
  Serial.print("Light: ");
  Serial.print(lux);
  Serial.println(" lx");
  
  // Now we can publish stuff!
  Serial.print(F("\nSending Light val "));
  Serial.print(lux);
  Serial.print("...");

  // publish the lux value to adafruit server, print ok / failed as the status goes by
  if (! light.publish(lux)) 
  {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }
  delay(2000);

  // read the LPG Sensor value
  int val = analogRead(A0);
  if(val <= 50) val = 50;
  if(val >= 800) val = 800;

  // map the LPG sensor value in Percentage from 0-100%
  val = map(val,50,800,0,100);
  delay(1000);

  // print LPG value on serial port
  Serial.print("Gas Value = ");
  Serial.println(val);
  
  // Now we can publish stuff!
  Serial.print(F("\nSending Gas val "));
  Serial.print(val);
  Serial.print("...");
  delay(2000);
  
  // publish the lux value to adafruit server, print ok / failed as the status goes by
  if (! gas.publish(val)) 
  {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }
 
  // ping the server to keep the mqtt connection alive
  // NOT required if you are publishing once every KEEPALIVE seconds
  /*
  if(! mqtt.ping()) {
    mqtt.disconnect();
  }
  */
  blink_led();
  delay(5000);
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() 
{
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}
