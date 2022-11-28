#include "ThingSpeak.h"
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Firebase_ESP_Client.h>
#include <MAX30105.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <addons/RTDBHelper.h>
#include <addons/TokenHelper.h>
#include <secrets.h>
#include <spo2_algorithm.h>

float _temp = 0;

// GY615
typedef struct
{
	float e;
	float to;
	float ta;
	float bo;
} gyir;
SoftwareSerial mySerial(D5, D6); // RX, TX
byte add = 0xa4;
byte len = 0, start_reg = 0;
unsigned char Re_buf[30], counter = 0;
unsigned char sign = 0;
gyir my_ir;

// MAX30105
MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255

uint32_t irBuffer[100];	 // infrared LED sensor data
uint32_t redBuffer[100]; // red LED sensor data

int32_t bufferLength; // data length
int32_t spo2;		  // SPO2 value
int8_t validSPO2;	  // indicator to show if the SPO2 calculation is valid
int32_t heartRate;	  // heart rate value
int8_t
	validHeartRate; // indicator to show if the heart rate calculation is valid

byte pulseLED = D3;			// Must be on PWM pin
byte readLED = LED_BUILTIN; // Blinks with each data read

// Firebase
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
unsigned long lastUploadToFirebase = 0;

// ThingSpeak
WiFiClient client;
unsigned long lastUploadToThingSpeak = 0;

void signIn(const char *email, const char *password)
{
	/* Assign the user sign in credentials */
	auth.user.email = email;
	auth.user.password = password;

	/* Reset stored authen and config */
	Firebase.reset(&config);

	/* Initialize the library with the Firebase authen and config */
	Firebase.begin(&config, &auth);
}

void setup()
{
	Serial.begin(115200);

	// GY615
	mySerial.begin(9600);
	mySerial.listen();

	// MAX30105
	pinMode(pulseLED, OUTPUT);
	pinMode(readLED, OUTPUT);

	// Initialize sensor
	if (!particleSensor.begin(
			Wire, I2C_SPEED_FAST)) // Use default I2C port, 400kHz speed
	{
		Serial.println(F("MAX30105 was not found. Please check wiring/power."));
		while (1)
			;
	}

	Serial.println(F("\nAttach sensor to finger with rubber band.\nPress any key "
					 "to start conversion"));
	while (Serial.available() == 0)
		; // wait until user presses a key

	Serial.read();

	byte ledBrightness = 60; // Options: 0=Off to 255=50mA
	byte sampleAverage = 4;	 // Options: 1, 2, 4, 8, 16, 32
	byte ledMode = 2;		 // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
	byte sampleRate = 100;	 // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
	int pulseWidth = 411;	 // Options: 69, 118, 215, 411
	int adcRange = 4096;	 // Options: 2048, 4096, 8192, 16384

	particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate,
						 pulseWidth,
						 adcRange); // Configure sensor with these settings

	WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
	Serial.print("Connecting to Wi-Fi");
	while (WiFi.status() != WL_CONNECTED)
	{
		Serial.print(".");
		delay(300);
	}
	Serial.println();
	Serial.print("Connected with IP: ");
	Serial.println(WiFi.localIP());
	Serial.println();

	Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

	/* Assign the api key (required) */
	config.api_key = API_KEY;

	/* Assign the RTDB URL */
	config.database_url = DATABASE_URL;

	Firebase.reconnectWiFi(true);
	fbdo.setResponseSize(4096);

	/* Assign the callback function for the long running token generation task */
	config.token_status_callback =
		tokenStatusCallback; // see addons/TokenHelper.h

	/** Sign in as user 1 */
	signIn(USER_EMAIL1, USER_PASSWORD1);

	ThingSpeak.begin(client); // Initialize ThingSpeak
}

void uploadToFireBase()
{
	if (millis() - lastUploadToFirebase > 1000)
	{
		lastUploadToFirebase = millis();

		if (Firebase.ready())
		{
			String uid = auth.token.uid.c_str();
			String tempPath = "/UsersData/" + uid + "/temperature";
			String spo2Path = "/UsersData/" + uid + "/oxygen";
			String BPMPath = "/UsersData/" + uid + "/heartbeat";

			Serial.printf("Current UID: %s\n", auth.token.uid.c_str());

			Serial.printf("Set temp... %s\n",
						  Firebase.RTDB.setFloat(&fbdo, tempPath, _temp)
							  ? "ok"
							  : fbdo.errorReason().c_str());

			Serial.printf("Set spo2... %s\n",
						  Firebase.RTDB.setFloat(&fbdo, spo2Path, spo2)
							  ? "ok"
							  : fbdo.errorReason().c_str());

			Serial.printf("Set BPM... %s\n",
						  Firebase.RTDB.setFloat(&fbdo, BPMPath, heartRate)
							  ? "ok"
							  : fbdo.errorReason().c_str());
		}
	}
}

void uploadToThingSpeak()
{
	if (millis() - lastUploadToThingSpeak > 15000)
	{
		String myStatus = "";

		ThingSpeak.setField(1, heartRate);
		ThingSpeak.setField(2, spo2);
		ThingSpeak.setField(3, _temp);

		// set the status
		ThingSpeak.setStatus(myStatus);

		// write to the ThingSpeak channel
		int x = ThingSpeak.writeFields(MY_CH_ID, MY_WRITE_API_KEY);
		if (x == 200)
		{
			Serial.println("Channel update successful.");
		}
		else
		{
			Serial.println("Problem updating channel. HTTP error code " + String(x));
		}

		lastUploadToThingSpeak = millis();
	}
}

void gy615()
{
	unsigned char i = 0, sum = 0;
	while (mySerial.available())
	{
		Re_buf[counter] = (unsigned char)mySerial.read();
		switch (counter)
		{
		case 0:
			if (Re_buf[0] != add)
				return;
			break;
		case 1:

			if (Re_buf[1] != 0x03)
			{
				counter = 0;
				return;
			}
			break;
		case 2:
			if (Re_buf[2] < 16)
				start_reg = Re_buf[2];
			else
			{
				counter = 0;
				return;
			}
			break;
		case 3:
			if ((start_reg + Re_buf[3]) < 16)
				len = Re_buf[3];
			else
			{
				counter = 0;
				return;
			}

			break;
		default:
			if (len + 5 == counter)
			{
				sign = 1;
			}
			break;
		}

		if (sign)
		{
			sign = 0;
			for (i = 0; i < counter - 1; i++)
				sum += Re_buf[i];
			counter = 0;
			if (sum == Re_buf[i]) // 检查帧头，帧尾
			{
				if (start_reg == 0x07)
				{
					my_ir.to = (Re_buf[5] << 8) | Re_buf[6];
					my_ir.ta = (Re_buf[7] << 8) | Re_buf[8];
					my_ir.bo = (Re_buf[9] << 8) | Re_buf[10];
					my_ir.e = Re_buf[4];
				}
				// Serial.print("E:");
				// Serial.print((float)my_ir.e / 100);
				// Serial.print(",to:");
				// Serial.print((float)my_ir.to / 100);
				// Serial.print(",ta:");
				// Serial.print((float)my_ir.ta / 100);
				// Serial.print(" ,bo:");
				// Serial.println((float)my_ir.bo / 100);
				_temp = (float)my_ir.to / 100;
			}
			//     else
			//        {
			//          Serial.print(" sum ");
			//          Serial.println(sum);
			//      }
		}
		else
			counter++;
		//   Serial.print("cont:");
		//   Serial.println(counter);
	}
}

void max30105()
{
	bufferLength =
		100; // buffer length of 100 stores 4 seconds of samples running at 25sps

	// read the first 100 samples, and determine the signal range
	for (byte i = 0; i < bufferLength; i++)
	{
		while (particleSensor.available() == false) // do we have new data?
			particleSensor.check();					// Check the sensor for new data

		redBuffer[i] = particleSensor.getRed();
		irBuffer[i] = particleSensor.getIR();
		particleSensor
			.nextSample(); // We're finished with this sample so move to next sample

		// Serial.print(F("red="));
		// Serial.print(redBuffer[i], DEC);
		// Serial.print(F(", ir="));
		// Serial.println(irBuffer[i], DEC);
	}

	// calculate heart rate and SpO2 after first 100 samples (first 4 seconds of
	// samples)
	maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer,
										   &spo2, &validSPO2, &heartRate,
										   &validHeartRate);

	// Continuously taking samples from MAX30102.  Heart rate and SpO2 are
	// calculated every 1 second
	while (1)
	{
		// dumping the first 25 sets of samples in the memory and shift the last 75
		// sets of samples to the top
		for (byte i = 25; i < 100; i++)
		{
			redBuffer[i - 25] = redBuffer[i];
			irBuffer[i - 25] = irBuffer[i];
		}

		// take 25 sets of samples before calculating the heart rate.
		for (byte i = 75; i < 100; i++)
		{
			while (particleSensor.available() == false) // do we have new data?
				particleSensor.check();					// Check the sensor for new data

			digitalWrite(
				readLED,
				!digitalRead(readLED)); // Blink onboard LED with every data read

			redBuffer[i] = particleSensor.getRed();
			irBuffer[i] = particleSensor.getIR();
			particleSensor.nextSample(); // We're finished with this sample so move to
										 // next sample

			// send samples and calculation result to terminal program through UART
			// Serial.print(F("red="));
			// Serial.print(redBuffer[i], DEC);
			// Serial.print(F(", ir="));
			// Serial.print(irBuffer[i], DEC);

			// Serial.print(F(", HR="));
			// Serial.print(heartRate, DEC);

			// Serial.print(F(", HRvalid="));
			// Serial.print(validHeartRate, DEC);

			// Serial.print(F(", SPO2="));
			// Serial.print(spo2, DEC);

			// Serial.print(F(", SPO2Valid="));
			// Serial.println(validSPO2, DEC);
		}

		// After gathering 25 new samples recalculate HR and SP02
		maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer,
											   &spo2, &validSPO2, &heartRate,
											   &validHeartRate);

		break;
	}
}

void loop()
{
	for (size_t i = 0; i < 12; i++)
	{
		gy615();
	}
	max30105();

	Serial.println("Temperature: " + String(_temp));
	Serial.println("BPM: " + String(heartRate));
	Serial.println("spo2: " + String(spo2) + "%");

	uploadToFireBase();
	uploadToThingSpeak();
}