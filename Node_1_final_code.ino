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

#include <OneWire.h> // library for Temperature sensor

int DS18S20_Pin1 = D7; //DS18S20 Signal pin on digital D7

OneWire ds1(DS18S20_Pin1);  // on digital pin A3


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
const char temp_feed[]  = AIO_USERNAME "/feeds/temp";
Adafruit_MQTT_Publish temp = Adafruit_MQTT_Publish(&mqtt, temp_feed);

const char vibration_feed[]  = AIO_USERNAME "/feeds/vibration";
Adafruit_MQTT_Publish vibration = Adafruit_MQTT_Publish(&mqtt, vibration_feed);

const char fire_feed[]  = AIO_USERNAME "/feeds/fire";
Adafruit_MQTT_Publish fire = Adafruit_MQTT_Publish(&mqtt, fire_feed);






void MQTT_connect();


void blink_led()
{
  digitalWrite(LED_BUILTIN,HIGH); delay(100);  digitalWrite(LED_BUILTIN,LOW); delay(100);
  digitalWrite(LED_BUILTIN,HIGH); delay(100);digitalWrite(LED_BUILTIN,LOW); delay(100);
}


void setup() 
{
   Serial.begin(115200);   // starts a serial port for debugging purpose
  pinMode(LED_BUILTIN,OUTPUT);
  // On esp8266 you can select SCL and SDA pins using Wire.begin(D4, D3);

  delay(10);

  Serial.println(F("Adafruit MQTT demo"));  //test msg

  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());

}

uint32_t x=0;


void loop() 
{
  
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();
  // this is our 'wait for incoming subscription packets' busy subloop
  // try to spend your time here

  unsigned int color;
/********************************************************************/
 
  float temperature1 = getTemp1();
  Serial.print("Temp1 = ");
  Serial.println(temperature1);
  // Now we can publish stuff!
  
  Serial.print(F("\nSending Temp val "));
  Serial.print(temperature1);
  Serial.print("...");
  int tempr = (int)temperature1;
  // BELOW is the code to publish the temperature value
  if (! temp.publish(tempr)) 
  {
    Serial.println(F("Failed"));
  } else 
  {
    Serial.println(F("OK!"));
  }
  delay(2000);
  /*****************************************************************/
  if (! vibration.publish("YES")) 
  {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }
  delay(2000);
/***************************************************/
  int fir = digitalRead(D5);
  
  Serial.print(F("\nSending Fire val "));
  Serial.print(fir);
  Serial.print("...");


  if (! fire.publish(fir)) 
  {
    Serial.println(F("Failed"));
  } 
  else 
  {
    Serial.println(F("OK!"));
  }

 
  // ping the server to keep the mqtt connection alive
  // NOT required if you are publishing once every KEEPALIVE seconds
  if(! mqtt.ping()) {
    mqtt.disconnect();
  }
  blink_led();
  delay(5000);
  
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() 
{
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) 
  {
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

float getTemp1()
{
  //returns the temperature from one DS18S20 in DEG Celsius

  byte data1[12];
  byte addr1[8];

  if ( !ds1.search(addr1)) {
      //no more sensors on chain, reset search
      ds1.reset_search();
      return -1000;
  }

  if ( OneWire::crc8( addr1, 7) != addr1[7]) {
      Serial.println("CRC is not valid!");
      return -1000;
  }

  if ( addr1[0] != 0x10 && addr1[0] != 0x28) {
      Serial.print("Device is not recognized");
      return -1000;
  }

  ds1.reset();
  ds1.select(addr1);
  ds1.write(0x44,1); // start conversion, with parasite power on at the end

  byte present = ds1.reset();
  ds1.select(addr1);    
  ds1.write(0xBE); // Read Scratchpad

  
  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data1[i] = ds1.read();
  }
  
  ds1.reset_search();
  
  byte MSB = data1[1];
  byte LSB = data1[0];

  float tempRead1 = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum1 = tempRead1 / 16;
  
  return TemperatureSum1;
  
}
