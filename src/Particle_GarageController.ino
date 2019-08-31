/*******************************************************************************
 * Project  : GarageController
 * Hardware : Particle Photon
 * Date     : 31.08.2019
 * Author   : Tim Hornikel
 * License  : GNU General Public License v3+
*******************************************************************************/

// Particle Cloud Variables
String sensorStatus = "Initialization";
String firmwareVersion = "v0.3.1"; // Version MAJOR.MINOR.PATCH
String vehicleInGarageCloud = "Initialization";
String garageDoorStateCloud = "Initialization";

// includings of 3rd party libraries
// @todo: check necessary includings
#include "PietteTech_DHT.h" // Temperature, Humidity Sensor Libray

// system defines
// @todo: check necessary defines
#define DHTTYPE AM2302                     // Sensor type DHT11/21/22/AM2301/AM2302
#define DHTPIN D2                          // Digital pin for communications
#define DHT_SAMPLE_INTERVAL 10 * 60 * 1000 // Sample every ten minutes

/*******************************************************************************
 * Description    : Use of callback_wrapper has been deprecated but left in this example to confirm backwards compatibility.
 * NOTE           : This wrapper is in charge of calling must be defined like this for the lib work
 * Input          : None
 * Output         : None
 *******************************************************************************/
void dht_wrapper(); // must be declared before the lib initialization

// Lib instantiate
// ref: https://www.sparkfun.com/datasheets/Sensors/Temperature/DHT22.pdf
PietteTech_DHT DHT(DHTPIN, DHTTYPE, dht_wrapper);

// Select external Antenna
STARTUP(WiFi.selectAntenna(ANT_EXTERNAL));

// globals
int DHTnextSampleTime = 0;            // Next time we want to start sample
bool bDHTstarted;                     // flag to indicate we started acquisition
bool bTempAlert = false;              // variable for toggeling Temperature alert state
bool bAutomaticDoorOperation = false; // variable for setting automatic door mode (true = activated, false = deactivated)

double dHumidity = 40.0;
double dTemperature = 20.0;

const int sensorHight = 200; // hight of mounted sensor 1 in cm
const int vehicleHight = 30; // hight of vehicle to detect in cm

bool sensorDetect = false; // status of detected vehicle for ultrasonic sensor

int vehicleInGarage = 2; // 0 = initialization, 1 = inside garage, 2 = transition, 3 = outside in garage
int garageDoorState = 2; // 0 = initialization, 1 = closed, 2 = transition, 3 = open

// declaration of I/Os
const int garageTrigger = A0;     // garage door trigger relay
const int usTrigger = D3;         // trigger signal for ultrasonic sensors
const int usEcho = D4;            // echo signal for ultrasonic sensor 1
const int doorSensor1 = D0;       // garage door sensor1 (HIGH = door open)
const int doorSensor2 = D1;       // garage door sensor2 (HIGH = door closed)
const int statusLEDParticle = D6; // status LED for signalling Particle Cloud errors
const int statusLEDVehicle = D7;  // status LED for signalling if vehcile detected with sensor 1

const float tempOffset = 4.7; // temperat offset

// Declaration of time variables
unsigned long lastSync = millis();              // last synchronization of time in internet
unsigned long previousMillisVehicle = millis(); // stores last time vehicle was detected
unsigned long previousMillisUs = millis();      // stores last time ultrasonic sensors were updated

// Declaration of time constants
unsigned long ONE_DAY_MILLIS(24 * 60 * 60 * 1000);  // 24 hours
unsigned long vehicleDetectionThreshold(60 * 1000); // threshold of 60s for vehicle detection

// Setup of timer for garage door notification
Timer garageDoorTimer(15 * 60 * 1000, garageDoorNotification, true); // notification after 15 min

/*******************************************************************************
 * Description    : Setup
 * Input          : None
 * Output         : None
 *******************************************************************************/
void setup()
{

  // serial communications
  Serial.begin(9600);
  // Wait for a USB serial connection for up to 3 seconds
  //waitFor(Serial.isConnected, 3000);
  Serial.printlnf("System version: %s", System.version().c_str());
  Serial.printlnf("Firmware version: %s", firmwareVersion.c_str());
  Serial.println("---------------");

  // particle variables
  Particle.variable("FW-Version", firmwareVersion);
  Particle.variable("TempStatus", sensorStatus);
  Particle.variable("Temperature", &dTemperature, DOUBLE);
  Particle.variable("Humidity", &dHumidity, DOUBLE);
  Particle.variable("garageDoorState", garageDoorStateCloud);
  Particle.variable("vehicleState", vehicleInGarageCloud);

  // particle functions
  Particle.function("garageTrigger", triggerGarage);
  Particle.function("automaticMode", automaticMode);

  // Definition of I/O-Pins
  pinMode(garageTrigger, OUTPUT);
  pinMode(statusLEDVehicle, OUTPUT);
  pinMode(statusLEDParticle, OUTPUT);

  // setting trigger and echo pin
  pinMode(usTrigger, OUTPUT);
  pinMode(usEcho, INPUT);

  //setting door sensor pins
  pinMode(doorSensor1, INPUT_PULLDOWN);
  pinMode(doorSensor2, INPUT_PULLDOWN);

  // setting trigger pins to low during initialization
  digitalWrite(garageTrigger, LOW);
  digitalWriteFast(usTrigger, LOW);
}

/*******************************************************************************
 * Description    : Loop
 * Input          : None
 * Output         : None
 *******************************************************************************/
void loop()
{

  // checks status if patricle is connected to cloud
  checkCloudStatus();

  // reads temperature and humidity of sensor
  readTempHumid();

  // reads garage door state
  readGarageDoorState();

  // reads vehicle state
  // readVehicleState();  // feature currently not used

  // updates the time from the internet if necessary
  updateTime();
}

/*******************************************************************************
 * Description    : Reads the state of the garage door
 * Input          : None
 * Output         : None
 *******************************************************************************/
void readGarageDoorState()
{

  int doorSwitchOpen = digitalRead(doorSensor1);
  int doorSwitchClosed = digitalRead(doorSensor2);

  if (doorSwitchOpen == HIGH && doorSwitchClosed == LOW && garageDoorState > 1)
  {
    // garage door open

    garageDoorState = 1; // 1 = garage door open
    garageDoorStateCloud = "open";
    Serial.println("Garage door open");

    // start timer for notification threshold
    garageDoorTimer.start();
  }
  else if (doorSwitchOpen == LOW && doorSwitchClosed == LOW && garageDoorState != 2)
  {
    // garage door in transition

    garageDoorState = 2; // 2 = garage door in transition
    garageDoorStateCloud = "transition";
    Serial.println("Garage door in transition");

    // stop timer for notification threshold
    garageDoorTimer.stop();
  }
  else if (doorSwitchOpen == LOW && doorSwitchClosed == HIGH && garageDoorState < 3)
  {
    // garage door closed

    garageDoorState = 3; // 3 = garage door closed
    garageDoorStateCloud = "closed";
    Serial.println("Garage door closed");
  }
  else if (doorSwitchOpen == HIGH && doorSwitchClosed == HIGH)
  {
    // Sensor failure

    bAutomaticDoorOperation = false; // deactivating automatic door mode, to be activated manually afterwards
    garageDoorStateCloud = "Sensor failure";
    // Error treatment to be defined
    Serial.println("Garage door failure!");
  }
}

/*******************************************************************************
 * Description    : Notifies user about a left open garage door (see threshold)
 * Input          : None
 * Output         : None
 *******************************************************************************/
void garageDoorNotification()
{

  Particle.publish("GarageDoor", "Alert");
}

/*******************************************************************************
 * Description    : Notifies user about low temperature (see threshold)
 * Input          : None
 * Output         : None
 *******************************************************************************/
void temperatureNotification()
{

  Particle.publish("Temperature", "Alert");
}

/*******************************************************************************
 * Description    : Reads the state of vehicle presence
 * Input          : None
 * Output         : None
 *******************************************************************************/
void readVehicleState()
{

  // takes ultrasonic measurements (int sample time in ms)
  takeMeasurements(500);

  if (sensorDetect == true && vehicleInGarage > 1)
  {
    // vehcile detected

    if (vehicleInGarage == 3)
    {

      // resets the timer
      previousMillisVehicle = millis();
    }

    if (millis() - previousMillisVehicle < vehicleDetectionThreshold)
    {
      // vehicle just arrived

      vehicleInGarageCloud = "transition";
      vehicleInGarage = 2;
      blinkLED(statusLEDVehicle, 500, 500);
    }
    else
    {
      // vehicle inside garage

      vehicleInGarageCloud = "inside";
      vehicleInGarage = 1;
      digitalWrite(statusLEDVehicle, HIGH);
    }
  }
  else if (sensorDetect == false && vehicleInGarage < 3)
  {
    // no vehicle detected

    vehicleInGarageCloud = "outside";
    vehicleInGarage = 3;
    digitalWrite(statusLEDVehicle, LOW);

    // close garage door immediately when vehicle is leaving the garage
    if (bAutomaticDoorOperation)
    {
      triggerGarage("close");
    }
  }
}

/*******************************************************************************
 * Description    : Takes ultrasonic measurements
 * Input          : None
 * Output         : None
 *******************************************************************************/
void takeMeasurements(int interval)
{

  if (millis() - previousMillisUs >= interval)
  {

    // save the last time a measurement was taken
    previousMillisUs = millis();

    // Sensor 1 = vehicle inside garage
    if (detectVehicle() == true)
    {
      // vehicle in garage

      //digitalWrite(statusLEDVehicle, HIGH);
      sensorDetect = true;
    }
    else
    {

      //digitalWrite(statusLEDVehicle, LOW);
      sensorDetect = false;
    }
  }
}

/*******************************************************************************
 * Description    : Checks the presence of a vehcile in the garage
 * Input          : None
 * Output         : TRUE (vehicle detected), FALSE (no vehicle detected)
 *******************************************************************************/
bool detectVehicle()
{

  int distance = sensorHight - measureDistance();

  if (distance >= vehicleHight)
  {

    return true;
  }
  else
  {

    return false;
  }
}

/*******************************************************************************
 * Description    : Measures the Distance to an object using ultrasonic sensors
 * Input          : Sensor Number
 * Output         : Distance (cm)
 *******************************************************************************/
int measureDistance()
{

  uint32_t distance, timeMeasurement, timeDistance;

  // Ausbreitungsgeschwindigkeit (in Luft) = 331,5 + (0,6 * Temp°C) speedOfSound = 331,5 + ( 0.6 * dTemperature);
  //float speedOfSound = 331.5 + 0.6 * 22; // 22 °C
  double speedOfSound = 331.5 + 0.6 * dTemperature; // 22 °C

  // trigger the sensor by sending a HIGH pulse of 10 or more microseconds
  digitalWrite(usTrigger, LOW);
  delayMicroseconds(3);

  // critical, time-sensitive code starts
  //noInterrupts();

  digitalWriteFast(usTrigger, HIGH);
  delayMicroseconds(10);
  digitalWriteFast(usTrigger, LOW);

  // take ulrasonic measurement
  timeMeasurement = pulseIn(usEcho, HIGH);

  // critical, time-sensitive code ends
  //interrupts();

  timeDistance = timeMeasurement / 2;
  distance = speedOfSound * timeDistance * pow(10, -4); // in cm

  //Serial.printf("Distance %i",sensorNumber);
  //Serial.printf(": %i",distance);
  //Serial.println();

  // calclulate distance in cm
  return distance;
  delayMicroseconds(50);
}

/*******************************************************************************
 * Description    : Trigger for opening / closing the garage door
 * Input          : open / close
 * Output         : 1 = opening, 0 = closing, -1 = invalid command
 *******************************************************************************/
int triggerGarage(String command)
{

  if (garageDoorState > 2 && command == "open")
  {

    Serial.println("Opening garage door ...");

    digitalWrite(garageTrigger, HIGH);
    delay(500);
    digitalWrite(garageTrigger, LOW);

    return 1;
  }
  else if (garageDoorState < 2 && command == "close")
  {

    Serial.println("Closing garage door ...");

    digitalWrite(garageTrigger, HIGH);
    delay(500);
    digitalWrite(garageTrigger, LOW);

    return 0;
  }
  else
  {

    return -1;
  }
}

/*******************************************************************************
 * Description    : Setting mode for automatic opening / closing the garage door
 * Input          : on / off / status
 * Output         : 1 = activated, 0 = deactivated, -1 = invalid command
 *******************************************************************************/
int automaticMode(String command)
{

  if (command == "on")
  {

    bAutomaticDoorOperation = true;
    return 1;
  }
  else if (command == "off")
  {

    bAutomaticDoorOperation = false;
    return 0;
  }
  else if (command == "status")
  {

    if (bAutomaticDoorOperation)
    {

      return 1;
    }
    else
    {

      return 0;
    }
  }
  else
  {

    return -1;
  }
}

/*******************************************************************************
  * Description    : Use of callback_wrapper has been deprecated but left in this example to confirm backwards compatibility.
  * NOTE           : This wrapper is in charge of calling must be defined like this for the lib work
  * Input          : None
  * Output         : None
  *******************************************************************************/
void dht_wrapper()
{

  DHT.isrCallback();
}

/*******************************************************************************
 * Description    : Reads Temperature and Humidity of Sensor DHT22
 * Input          : None
 * Output         : None
 *******************************************************************************/
void readTempHumid()
{

  // Check if we need to start the next sample
  if (millis() > DHTnextSampleTime)
  {
    if (!bDHTstarted)
    { // start the sample
      Serial.print("\n\n");
      Serial.print("Retrieving information from sensor. ");
      DHT.acquire();
      bDHTstarted = true;
    }

    if (!DHT.acquiring())
    { // has sample completed?

      // get DHT status
      int result = DHT.getStatus();

      Serial.print("Read sensor: ");

      switch (result)
      {

      case DHTLIB_OK:
        Serial.println("OK");
        sensorStatus = "OK";
        break;

      case DHTLIB_ERROR_CHECKSUM:
        Serial.println("Error\n\r\tChecksum error");
        sensorStatus = "Error\n\r\tChecksum error";
        break;

      case DHTLIB_ERROR_ISR_TIMEOUT:
        Serial.println("Error\n\r\tISR time out error");
        sensorStatus = "Error\n\r\tISR time out error";
        break;

      case DHTLIB_ERROR_RESPONSE_TIMEOUT:
        Serial.println("Error\n\r\tResponse time out error");
        sensorStatus = "Error\n\r\tResponse time out error";
        break;

      case DHTLIB_ERROR_DATA_TIMEOUT:
        Serial.println("Error\n\r\tData time out error");
        sensorStatus = "Error\n\r\tData time out error";
        break;

      case DHTLIB_ERROR_ACQUIRING:
        Serial.println("Error\n\r\tAcquiring");
        sensorStatus = "Error\n\r\tAcquiring";
        break;

      case DHTLIB_ERROR_DELTA:
        Serial.println("Error\n\r\tDelta time to small");
        sensorStatus = "Error\n\r\tDelta time to small";
        break;

      case DHTLIB_ERROR_NOTSTARTED:
        Serial.println("Error\n\r\tNot started");
        sensorStatus = "Error\n\r\tNot started";
        break;

      default:
        Serial.println("Unknown error");
        sensorStatus = "Unknown error";
        break;
      }

      Serial.print("Humidity (%): ");
      Serial.println(DHT.getHumidity(), 1);
      //Particle.publish("Humidity (%)", String(DHT.getHumidity(), 1));
      dHumidity = (double)(roundf((10.0 * DHT.getHumidity())) / 10.0);

      Serial.print("Temperature (°C): ");
      Serial.println((DHT.getCelsius() - tempOffset), 1);
      //Particle.publish("Temperature (°C)", String((DHT.getCelsius()-tempOffset), 1));
      dTemperature = (double)(roundf((10.0 * (DHT.getCelsius() - tempOffset))) / 10.0);

      //Serial.print("Temperature (°F): ");
      //Serial.println(DHT.getFahrenheit(), 1);

      //Serial.print("Temperature (K): ");
      //Serial.println(DHT.getKelvin(), 1);

      Serial.print("Dew Point (°C): ");
      Serial.println(DHT.getDewPoint());

      Serial.print("Dew Point Slow (°C): ");
      Serial.println(DHT.getDewPointSlow());

      bDHTstarted = false;                                // reset the sample flag so we can take another
      DHTnextSampleTime = millis() + DHT_SAMPLE_INTERVAL; // set the time for next sample
    }

    // Manage alerts for low temperature
    if (dTemperature < 3.0 && bTempAlert == false)
    {

      temperatureNotification();
      bTempAlert = true;
    }
    else if (dTemperature >= 5.0 && bTempAlert == true)
    {

      bTempAlert = false;
    }
  }
}

/*******************************************************************************
 * Description    : Request time synchronization from the Particle Cloud once a day
 * Input          : None
 * Output         : None
 *******************************************************************************/
void updateTime()
{

  if (millis() - lastSync > ONE_DAY_MILLIS)
  {

    // Request time synchronization from the Particle Cloud
    Particle.syncTime();
    Serial.printf("Time updated at %s...", Time.timeStr().c_str());
    lastSync = millis();
  }
}

/*******************************************************************************
 * Description    : Blinks any LED as disired w/o delay
 * Input          : LED Pin defintion, Off-Time (ms), On-Time (ms)
 * Output         : None
 *******************************************************************************/
void blinkLED(int pin, int off, int on)
{

  int blinkPhase = millis() % (off + on);

  if (blinkPhase < off)
  {

    digitalWrite(pin, LOW);
  }
  else
  {

    digitalWrite(pin, HIGH);
  }
}

/*******************************************************************************
 * Description    : Checks if Particle cloud is connected
 * Input          : None
 * Output         : None
 *******************************************************************************/
void checkCloudStatus()
{

  if (WiFi.ready())
  {

    if (Particle.connected())
    {

      // connection to particle backend established
      //blinkLED(statusLEDParticle, 5000, 100);
    }
    else
    {

      // connection to particle backend not established
      blinkLED(statusLEDParticle, 1000, 100);
    }
  }
  else if (!WiFi.ready())
  {

    // wifi connection not established
    blinkLED(statusLEDParticle, 500, 500);
  }
}