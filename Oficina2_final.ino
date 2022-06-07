#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"

MAX30105 particleSensor;

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif

//constantes
#define lowLevel 35 //GPIO2 - D2 - Setup for leads off detection LO -
#define highLevel 34 //GPIO4 - D4 - Setup for leads off detection LO +
#define ECGsignalIn 32 //GPIO15 - D15
#define shutdown_pin NULL
#define led 23
#define MAX_BRIGHTNESS 255

//variáveis globais
int j = 2;
int values[2697]; // SPO2: values[0]; BPM: values[1]; ...
int timer = 0;

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

byte pulseLED = 11; //Must be on PWM pin
byte readLED = 13; //Blinks with each data read

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

int q = 75;
int w = 0;

//escopos
void signalRange(bool first);
void SPO2Measure();
void BPMMeasure();
int ecgRead ();
void postDataToServer();
void arrayCreate (int BPM, int SPO2, int ECG);
void taskEcgRead (void* parameter);


//wifi
const char* ssid = "ssid";
const char* password =  "password";

void setup() {
  Serial.begin(115200);

  pinMode (highLevel, INPUT);
  pinMode (lowLevel, INPUT);
  pinMode (led, OUTPUT);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) { //Check for the connection
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  Serial.println("Connected to the WiFi network");

  setupOximeter();


  xTaskCreate(taskEcgRead, "taskEcgRead", 10000, NULL, 1, NULL);
}


void loop() {
  digitalWrite (23, !digitalRead(23));
  signalRange(false);
  SPO2Measure();
  BPMMeasure();
  w++;
  timer++;
  delay (1);
}

void taskEcgRead (void* parameter) {
  while (1) {
    int leitura;

    if ((digitalRead(lowLevel) == 1) || (digitalRead(highLevel) == 1)) {
      digitalWrite(shutdown_pin, LOW); //standby mode
      leitura = 0;
    }
    else {
      // send the value of analog input 0:
      digitalWrite(shutdown_pin, HIGH); //normal mode
      leitura = analogRead(ECGsignalIn);
    }
    /////////////////////////////////////////////////////////////////// montando o vetor
    values[0] = spo2;
    values[1] = beatAvg;

    if (j <= 2697) {
      values[j] = leitura;
      j++;
    } else {
      postDataToServer();
      j = 2;
      values[j] = leitura;
    }
  }
}


void postDataToServer() {

  Serial.println("Posting JSON data to server...");
  // Block until we are able to connect to the WiFi access point
  if (WiFi.status() == WL_CONNECTED) {

    HTTPClient http;

    http.begin("https://httpbin.org/anything");
    //http.begin("https://webhook.site/1bc501b7-18d7-4443-80ca-fd67ab57ccb3");
    http.addHeader("Content-Type", "application/json");

    DynamicJsonDocument doc(24576);//24576 com 2697 postiions
    // Add values in the document
    //
    doc["SPO2"] = values[0];
    doc["BPM"] = values[1];

    // Add an array.
    //
    JsonArray data = doc.createNestedArray("ECGData");
    for (int i = 2; i <= 2697; i++) {//1349
      data.add(values[i]);
    }

    String requestBody;
    serializeJson(doc, requestBody);

    int httpResponseCode = http.POST(requestBody);

    if (httpResponseCode > 0) {

      String response = http.getString();

      Serial.println(httpResponseCode);
      //Serial.println(response);

    }
    else {

      Serial.printf("Error occurred while sending HTTP POST:");

    }

  }
}


void signalRange(bool first) {
  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps

  //read the first 100 samples, and determine the signal range
  if (w == 120000 || first == true) {
    for (byte i = 0 ; i < bufferLength ; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample

      Serial.print(F("red="));
      Serial.print(redBuffer[i], DEC);
      Serial.print(F(", ir="));
      Serial.println(irBuffer[i], DEC);
    }
    //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
    first = false;
  } else {
    w = 0;
  }
}

void SPO2Measure() {
  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second

  //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
  for (byte i = 25; i < 100; i++)
  {
    redBuffer[i - 25] = redBuffer[i];
    irBuffer[i - 25] = irBuffer[i];
  }

  //take 25 sets of samples before calculating the heart rate.
  if (q < 100)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

    redBuffer[q] = particleSensor.getRed();
    irBuffer[q] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

    values[0] = spo2;
    q++;
  } else {
    q = 75;
  }

  if (q == 99) {
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  }
}

void BPMMeasure() {
  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }
  values[1] = beatAvg;
}

void setupOximeter() {

  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    digitalWrite(23, HIGH);
    while (1);
  }


  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  signalRange(true);
}
