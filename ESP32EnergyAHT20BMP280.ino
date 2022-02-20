/*
 * This sketch is a branc of my PubSubWeather sketch.
 * This sketch will use a AHT20/BMP280 combination sensor to show temperature, pressure, and humidity.
 * The ESP-32 SDA pin is GPIO21, and SCL is GPIO22.
 */
#include "WiFi.h"						// This header is part of the standard library.  https://www.arduino.cc/en/Reference/WiFi
#include <Wire.h>						// This header is part of the standard library.  https://www.arduino.cc/en/reference/wire
#include <PubSubClient.h>			// PubSub is the MQTT API.  Author: Nick O'Leary  https://github.com/knolleary/pubsubclient
#include <SparkFun_Qwiic_Humidity_AHT20.h>	// Library used to interface with the AHT20.  Author: SparkFun  https://github.com/sparkfun/SparkFun_Qwiic_Humidity_AHT20_Arduino_Library
#include "Seeed_BMP280.h"							// https://github.com/Seeed-Studio/Grove_BMP280
#include "EmonLib.h"					// The EnergyMonitor Library.
#include "privateInfo.h"			// I use this file to hide my network information from random people browsing my GitHub repo.


/**
 * Declare network variables.
 * Adjust the commented-out variables to match your network and broker settings.
 * The commented-out variables are stored in "privateInfo.h", which I do not upload to GitHub.
 */
//const char* wifiSsid = "yourSSID";				// Typically kept in "privateInfo.h".
//const char* wifiPassword = "yourPassword";		// Typically kept in "privateInfo.h".
//const char* mqttBroker = "yourBrokerAddress";	// Typically kept in "privateInfo.h".
//const int mqttPort = 1883;							// Typically kept in "privateInfo.h".
const char* mqttTopic = "ajhWeather";
const char* mqttEnergyTopic = "ajhEnergy";
const char clientId[31] = "ESP32withSCT013";
const char sketchName[35] = "ESP32CurrentVoltageAHT20BMP280.ino";
const char* notes = "HiLetgo ESP32 with SCT013, AHT20, and BMP280";
String ipString = "127.0.0.1";
char macCharArray[18];
int loopCount = 0;
int mqttPublishDelayMS = 60000;
int ADC_GPIO = 36;	// GPIO 36 is channel 0 of ADC 1 on an ESP32.
float seaLevelPressure = 1018.0;		// Mean Sea Level in Pa.


// Create class objects.
WiFiClient esp32Client;							// Network client used by the MQTT library.
PubSubClient mqttClient( esp32Client );	// MQTT client.
AHT20 aht20Sensor;
BMP280 bmp280Sensor;
EnergyMonitor emon1;								// Instantiate an EnergyMonitor class object.


/**
 * The setup() function runs once when the device is booted, and then loop() takes over.
 */
void setup()
{
	// Start the Serial communication to send messages to the computer.
	Serial.begin( 115200 );
	while ( !Serial )
		delay( 100 );
	Serial.println();
	Serial.print( "Running setup() in " );
	Serial.println( sketchName );
	Wire.begin();	// Join I2C bus.

	Serial.println( "Initializing the BMP280 sensor..." );
	// Initialize the BMP280.
	if( !bmp280Sensor.init() )
	{
		Serial.println( "BMP280 could not be initialized!" );
	}
	else
	{
		Serial.println( "BMP280 has been initialized." );
	}

	Serial.println( "Initializing the AHT20 sensor..." );
	// Initialize the AHT20.
	if( aht20Sensor.begin() == false )
	{
		Serial.println( "AHT20 not detected. Please check wiring." );
		while( 1 );
	}
	Serial.println( "AHT20 has been initialized." );

	// Set the MQTT client parameters.
	mqttClient.setServer( mqttBroker, mqttPort );

	// Read the MAC address into a character array.
	snprintf( macCharArray, 18, "%s", WiFi.macAddress().c_str() );

	emon1.current( ADC_GPIO, 111.1 );			// Current: input pin, calibration.

	// Try to connect to the configured WiFi network, and then the MQTT broker, up to 10 times each.
	wifiConnect( 10 );
	mqttConnect( 10 );
} // End of setup() function.


void wifiConnect( int maxAttempts )
{
	Serial.println( "\nEntering wifiConnect()" );
	// Announce WiFi parameters.
	Serial.print( "WiFi connecting to SSID \"" );
	Serial.print( wifiSsid );
	Serial.println( "\"" );

	// Connect to the WiFi network.
	Serial.printf( "Wi-Fi mode set to WIFI_STA %s\n", WiFi.mode( WIFI_STA ) ? "" : "Failed!" );
	WiFi.begin( wifiSsid, wifiPassword );

	int attemptCount = 0;
	/*
     WiFi.status() return values:
     0 : WL_IDLE_STATUS when WiFi is in process of changing between statuses
     1 : WL_NO_SSID_AVAIL in case configured SSID cannot be reached
     3 : WL_CONNECTED after successful connection is established
     4 : WL_CONNECT_FAILED if wifiPassword is incorrect
     6 : WL_DISCONNECTED if module is not configured in station mode
  */
	// Loop until WiFi has connected.
	while( WiFi.status() != WL_CONNECTED && attemptCount < maxAttempts )
	{
		delay( 1000 );
		Serial.println( "Waiting for a connection..." );
		Serial.print( "WiFi status: " );
		Serial.println( WiFi.status() );
		Serial.print( ++attemptCount );
		Serial.println( " seconds" );
	}

	// Print that WiFi has connected.
	Serial.println( '\n' );
	Serial.println( "WiFi connection established!" );
	ipString = WiFi.localIP().toString();																													// Read the IP address into a String.
	Serial.println( "IP address: " + ipString );

	Serial.println( "Exiting wifiConnect()\n" );
} // End of wifiConnect() function.


// mqttConnect() will attempt to (re)connect the MQTT client.
void mqttConnect( int maxAttempts )
{
	/*
	 * Possible states defined in PubSubClient.h:
	 * -4 : MQTT_CONNECTION_TIMEOUT - the server didn't respond within the keepalive time
	 * -3 : MQTT_CONNECTION_LOST - the network connection was broken
	 * -2 : MQTT_CONNECT_FAILED - the network connection failed
	 * -1 : MQTT_DISCONNECTED - the client is disconnected cleanly
	 * 0 : MQTT_CONNECTED - the client is connected
	 * 1 : MQTT_CONNECT_BAD_PROTOCOL - the server doesn't support the requested version of MQTT
	 * 2 : MQTT_CONNECT_BAD_CLIENT_ID - the server rejected the client identifier
	 * 3 : MQTT_CONNECT_UNAVAILABLE - the server was unable to accept the connection
	 * 4 : MQTT_CONNECT_BAD_CREDENTIALS - the username/password were rejected
	 * 5 : MQTT_CONNECT_UNAUTHORIZED - the client was not authorized to connect
	 */
	Serial.println( "\nEntering mqttConnect()" );
	int i = 0;
	if( !mqttClient.setBufferSize ( 512 ) )
		Serial.println( "Unable to set the buffer size to 512!" );

	// Loop until MQTT has connected.
	while( !mqttClient.connected() && i < maxAttempts )
	{
		Serial.print( "Attempting MQTT connection..." );
		// Connect to the broker using the MAC address for a clientID.  This guarantees that the clientID is unique.
		if( mqttClient.connect( clientId ) )
		{
			Serial.println( "connected!" );
		}
		else
		{
			Serial.print( " failed, return code: " );
			Serial.println( mqttClient.state() );
			Serial.println( "Will try again in 2 seconds." );
			// Wait 5 seconds before retrying.
			delay( 5000 );
		}
		i++;
	}
	Serial.println( "Exiting mqttConnect()\n" );
} // End of mqttConnect() function.


long printCurrentNet()
{
	Serial.println( "\nEntering printCurrentNet()" );

	// Print the SSID of the network you're attached to:
	Serial.print( "SSID: " );
	Serial.println( WiFi.SSID() );

	// Print the signal strength:
	long rssi = WiFi.RSSI();
	Serial.print( "signal strength (RSSI): " );
	Serial.println( rssi );

	// Print the encryption type:
//	byte encryption = WiFi.encryptionType();
//	Serial.print( "Encryption Type:" );
//	Serial.println( encryption, HEX );

	Serial.println( "End of printCurrentNet()\n" );
	return rssi;
} // End of printCurrentNet() function.


/**
 * The loop() function begins after setup(), and repeats as long as the unit is powered.
 */
void loop()
{
	// Prepare character arrays to hold the JSON.
	char mqttString[512];
	char mqttEnergyString[512];
	float ahtTemp = 0;
	float ahtHumidity = 0;

	loopCount++;
	Serial.println( "\n\n" );
	Serial.println( sketchName );
	Serial.print( "Loop count: " );
	Serial.println( loopCount );

	if( WiFi.status() != WL_CONNECTED )
		wifiConnect( 2 );

	long rssi = printCurrentNet();

	// Check the mqttClient connection state.
	// The loop() function facilitates the receiving of messages and maintains the connection to the broker, and returns 'true' if the client is connected.
	if( !mqttClient.loop() )
		mqttConnect( 2 );

	// Power consumption.
	Serial.println( "Electrical measurements:" );
	double iRMS = emon1.calcIrms( 1480 );		// Calculate RMS for current.
	double power = iRMS * 117.0;
	Serial.print( "\tPower: " );
	Serial.println( power );						// Apparent power
	Serial.print( "\tCurrent (RMS): " );
	Serial.println( iRMS );
	snprintf( mqttEnergyString, 512, "{\n\t\"sketch\": \"%s\",\n\t\"mac\": \"%s\",\n\t\"ip\": \"%s\",\n\t\"iRMS\": %.1f,\n\t\"power\": %.1f,\n\t\"uptime\": %d,\n\t\"notes\": \"%s\",\n\t\"rssi\": %ld\n}", sketchName, macCharArray, ipString, iRMS, power, loopCount, notes, rssi );
	// Publish the JSON to the MQTT broker.
	if( mqttClient.loop() )
	{
		Serial.print( "Publishing to the " );
		Serial.print( mqttEnergyTopic );
		Serial.println( " topic:" );
		Serial.println( mqttEnergyString );
		if( !mqttClient.publish( mqttEnergyTopic, mqttEnergyString ) )
			Serial.println( "~~~~~~~~ Energy publish failed!" );
		else
		{
			// Print the JSON to the Serial port.
			Serial.print( "Publishing to the " );
			Serial.print( mqttTopic );
			Serial.println( " topic:" );
			Serial.println( mqttString );
		}
	}
	else
	{
		Serial.println( "\n\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" );
		Serial.print( "Unable to publish to the " );
		Serial.print( mqttEnergyTopic );
		Serial.println( " topic!" );
		Serial.println( "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n\n\n" );
	}

	// The AHT20 can respond with a reading every ~50ms.
	// However, increased read time can cause the IC to heat around 1.0C above ambient temperature.
	// The datasheet recommends reading every 2 seconds.
//	if( aht20Sensor.available() == true )
//	{
	// Get temperature and humidity data from the AHT20.
	ahtTemp = aht20Sensor.getTemperature();
	ahtHumidity = aht20Sensor.getHumidity();

	// Print the AHT20 data.
	Serial.print( "AHT20 Temperature:   " );
	Serial.print( ahtTemp, 2 );
	Serial.print( " C\t" );
	Serial.print( "Humidity: " );
	Serial.print( ahtHumidity, 2 );
	Serial.print( "% RH" );
	Serial.println();
//	}
//	else
//	{
//		Serial.println( "\n\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" );
//		Serial.println( "The aht20Sensor.available() returned false!" );
//		Serial.println( "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n\n\n" );
//	}

	// Get temperature and humidity data from the BMP280.
	float bmpTemp = bmp280Sensor.getTemperature();
	float bmpPressure = bmp280Sensor.getPressure();
	float bmpAlt = bmp280Sensor.calcAltitude( bmpPressure );

	// Print the BMP280 data.
	Serial.print( "BMP280 Temperature:  " );
	Serial.print( bmpTemp, 2 );
	Serial.print( " C\t" );
	Serial.print( "Pressure: " );
	Serial.print( bmpPressure, 0 );
	Serial.print( " Pa\t" );
	Serial.print( "Altitude: " );
	Serial.print( bmpAlt, 2 );
	Serial.print( " m\n" );

	Serial.print( "Average Temperature: " );
	Serial.print( ( ahtTemp + bmpTemp ) / 2, 2 );
	Serial.println( " C\n" );

	// Write the weather readings to the character array in JSON format.
	snprintf( mqttString, 512, "{\n\t\"sketch\": \"%s\",\n\t\"mac\": \"%s\",\n\t\"ip\": \"%s\",\n\t\"tempC\": %.1f,\n\t\"humidity\": %.1f,\n\t\"temp2C\": %.1f,\n\t\"pressure\": %.1f,\n\t\"altitude\": %.1f,\n\t\"uptime\": %d,\n\t\"notes\": \"%s\",\n\t\"rssi\": %ld\n}", sketchName, macCharArray, ipString, ahtTemp, ahtHumidity, bmpTemp, bmpPressure, bmpAlt, loopCount, notes, rssi );
	// Publish the JSON to the MQTT broker.
	if( mqttClient.loop() )
	{
		if( !mqttClient.publish( mqttTopic, mqttString ) )
		{
			Serial.println( "\n\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" );
			Serial.println( "Weather publish failed!" );
			Serial.print( "MQTT client state: " );
			Serial.println( mqttClient.state() );
			Serial.println( "Attempted to publish this message:" );
			Serial.println( mqttString );
			Serial.println( "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n\n\n" );
		}
		else
		{
			// Print the JSON to the Serial port.
			Serial.print( "Publishing to the " );
			Serial.print( mqttTopic );
			Serial.println( " topic:" );
			Serial.println( mqttString );
		}
	}
	else
	{
		Serial.println( "\n\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" );
		Serial.print( "Unable to publish to the " );
		Serial.print( mqttTopic );
		Serial.println( " topic!" );
		Serial.println( "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n\n\n" );
	}

	Serial.print( "Pausing for " );
	Serial.print( mqttPublishDelayMS / 1000 );
	Serial.println( " seconds..." );
	delay( mqttPublishDelayMS );	// Wait for the configured time.
} // End of loop() function.
