  // TrackStar - Interacts with a BLE device (TrackStar app) which sends commands consisting of one letter or symbol 
  // which can be followed by a value to parse. 
  // The command letters and functions are listed here:
  //---------------------------------------------------------------------------------------------------
  // a - Resets tracking to sidereal rate after a slew. Sent when slew button released in TrackStar app
  // b - Slew forward
  // c - Slew backward
  // d - Turn tracking on
  // e - Turn tracking off
  // f - increase trackspeed by 1 step/sec
  // g - decrease trackspeed by -1 step/sec
  // h - start timelapse
  // i - Slew a set angle forward
  // j - Slew a set angle backwards
  // * -
  // # -
  // s -
  // $ - 
  //--------------------------------------------------------------------------------------------------- 
    #include <Arduino.h>
    #include <AccelStepper.h>
    #include <TMCStepper.h>
    #include <BLEDevice.h>
    #include <BLEServer.h>
    #include <BLEUtils.h>
    #include <BLE2902.h>
    #include <multiCameraIrControl.h>
    
    Sony A900(4);


    int j = 0;
    int k = 0;
    int batSum = 0;
    int batAvg = 0;
    int batVal = 0;
    float batVolt = 0;

    const int ledPin = 21;      // the number of the LED pin
    const int ledPin2 = 22;
    const int batPin = 35;
    const int slew1Pin = 25;
    int slew1State = 0;
    const int slew2Pin = 26;
    int slew2State = 0;
    
    //const int cameraled = 4;      // the number of the LED pin
    bool slew = false;
    bool track = true;
    bool timelapse = false;
    bool degreesSet = false;
    bool shotsSet = false;
    bool shutterSet = false;
    int trackspeed = 214;
    //int trackspeed = 10000; //fast trackspeed for debugging
    float shutterSec = 0;
    float shutterMillis = 0;
    float degrees = 0;
    int shots = 0;
    int cameraType = 0;
    long currentmillis = 0;
    long previousmillis = 0;
    // Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
    #define dirPin 12
    #define stepPin 13
    #define motorInterfaceType 1
    

    // TMC2208 serial comms setup
    //#define STALL_VALUE     200 // [0..255], during tracking the motor is probably running too slow to use stall guard or coolstep, it could be useful to set up coolstep for slews but the timing of slew is short enough that you can probably just up the current during slews if the motor struggles
    #define EN_PIN    19  // LOW: Driver enabled. HIGH: Driver disabled
    #define SERIAL_PORT Serial2 // TMC2209 HardwareSerial port
    #define R_SENSE 0.18  // Sense resistor for TMC2209
    #define DRIVER_ADDRESS 0// TMC2209 Driver address according to MS1 and MS2
    TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS); // Driver setup with hardware serial

    using namespace TMC2208_n;
    
    // Create a new instance of the AccelStepper class:
    AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

//BLuetoot setup
//recieve text from the BLE server (android app)
BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;


//std::string rxValue; // Could also make this a global var to access it in loop()

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
       std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++) {
          Serial.print(rxValue[i]);
          }
 
  

        // Do stuff based on the command received from the app - 
        
        // Return to Sideral rate after slew 
        if (rxValue.find("a") != -1) {
          stepper.stop();
          delay(1000);
          slew = false;
          driver.microsteps(256);
          stepper.setSpeed(trackspeed);
          Serial.println("sidereal"); 
          }
        // slew clockwise
        else if (rxValue.find("b") != -1) {
          stepper.setCurrentPosition(0);
          slew = true;
          driver.microsteps(32);
          stepper.moveTo(1000500);
          Serial.println("cw"); 
          }
        //slew counterclockwise
        else if (rxValue.find("c") != -1) {
          stepper.setCurrentPosition(0);
          slew = true;
          driver.microsteps(32);
          stepper.moveTo(-1000500);
          Serial.println("ccw");   
          }
        // turn tracking on  
        else if (rxValue.find("d") != -1) {
          track = true;
          digitalWrite(ledPin2,HIGH);
          driver.microsteps(256);
          stepper.runSpeed();
          Serial.println("track on");
          }
        // turn tracking off
        else if (rxValue.find("e") != -1) {
          track = false;
          digitalWrite(ledPin2, LOW);
          stepper.stop();
          Serial.println("track off");
          }
        // increase tracking speed 
        else if (rxValue.find("f") != -1) {
          trackspeed = trackspeed + 1;
          stepper.setSpeed(trackspeed);
          Serial.println("track speed increased to ");
          Serial.print(trackspeed);
          Serial.println(" steps/sec");
          }
        // decrease tracking speed
        else if (rxValue.find("g") != -1) {
          trackspeed = trackspeed - 1;
          stepper.setSpeed(trackspeed);
          Serial.println("track speed decreased to ");
          Serial.print(trackspeed);
          Serial.println("steps/sec");
          }
        // start timelapse
        else if (rxValue.find("h") != -1) {
          if (degreesSet && shotsSet && shutterSet == true){
            timelapse = true;
          }
          else{
            Serial.println("Easy tiger...Timelapse variables not set!!!");
              }
          }
        //slew by set angle clockwise
        else if (rxValue.find("i") != -1) {
          stepper.setCurrentPosition(0);
          driver.microsteps(64);
          stepper.moveTo(6400);
          while (stepper.distanceToGo() > 0)
          {
            stepper.run();
          }
          
          driver.microsteps(256);
          stepper.setSpeed(trackspeed);
          }
          
          


          else if (rxValue.find("j") != -1) {
          stepper.setCurrentPosition(0);
          driver.microsteps(64);
          stepper.moveTo(-6400);
          while (stepper.distanceToGo() < 0)
          {
            stepper.run();
          }
          
          driver.microsteps(256);
          stepper.setSpeed(trackspeed);
          }
          
      // *x Sets the degrees (x) in timelapse. The number of digits in x is flexible
       else if (rxValue.find("*") != -1) {
          rxValue.erase (rxValue.begin()+0);
          Serial.print("Degrees in timelapse");
          Serial.println(rxValue.c_str());
          degrees = ::atof(rxValue.c_str());
          degreesSet = true;
          }
      // #x Sets the number of shots (x) in timpelapse. The number of digits in x is flexible
       else if (rxValue.find("#") != -1) {
          rxValue.erase (rxValue.begin()+0);
          Serial.print("# of shots in timelapse: ");
          Serial.println(rxValue.c_str());
          shots = ::atoi(rxValue.c_str());
          shotsSet = true;
          }
      // sx sets the time for each timelaps position in seconds (x). the number of digits in x is flexible
       else if (rxValue.find("s") != -1) {
          rxValue.erase (rxValue.begin()+0);
          Serial.print("Shutter interval for timelapse: ");
          Serial.println(rxValue.c_str());
          if (rxValue.find("/") != -1) {
             rxValue.erase (0,2);
             Serial.println(rxValue.c_str());
             float shutterDenominator = ::atof(rxValue.c_str());
             shutterSec = 1 / shutterDenominator;
             }
           else { 
             shutterSec = ::atof(rxValue.c_str());
                }
          shutterMillis = shutterSec * 1000;
          shutterSet = true;
          }

      //$x Set Camera brand, sets the camera brand for the IR shutter release 
       else if (rxValue.find("$") != -1) {
          rxValue.erase (rxValue.begin()+0);
          Serial.print("camera type: ");
          Serial.println(rxValue.c_str());
          cameraType = ::atoi(rxValue.c_str());
    }
    }
}};


 
    void setup() {
      
      // start serial
      Serial.begin(9600);
      Serial2.begin(115200); // SW UART drivers


//BLE setup
      // Create the BLE Device
      BLEDevice::init("TrackStar");

      // Create the BLE Server
      pServer = BLEDevice::createServer();
      pServer->setCallbacks(new MyServerCallbacks());

      // Create the BLE Service
      BLEService *pService = pServer->createService(SERVICE_UUID);

      // Create a BLE Characteristic
      pTxCharacteristic = pService->createCharacteristic(
                    CHARACTERISTIC_UUID_TX,
                    BLECharacteristic::PROPERTY_NOTIFY
                  );
                      
      pTxCharacteristic->addDescriptor(new BLE2902());

      BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
                       CHARACTERISTIC_UUID_RX,
                      BLECharacteristic::PROPERTY_WRITE
                    );

      pRxCharacteristic->setCallbacks(new MyCallbacks());

      // Start the service
      pService->start();

      // Start advertising
      pServer->getAdvertising()->start();
      Serial.println("Waiting a client connection to notify...");
      
  
      
      //internal LED pin
      pinMode(ledPin, OUTPUT);
      pinMode(ledPin2, OUTPUT);
      
      //camera LED pin
      // pinMode(cameraled, OUTPUT);

      //slew buttons
      pinMode(slew1Pin, INPUT);
      pinMode(slew2Pin, INPUT);
      
      //enable stepper pin
      pinMode(EN_PIN, OUTPUT);
      digitalWrite(EN_PIN, LOW);    // Enable driver in hardware
      
      //Standby pin for stepper driver
      pinMode(14, OUTPUT);
      digitalWrite(14,LOW);
      
      // set initial LED state
      digitalWrite(ledPin, LOW);
      digitalWrite(ledPin2, HIGH);
     
     // set initial LED state
     // digitalWrite(cameraled, HIGH);
      
      // Set the maximum speed in steps per second:
      stepper.setMaxSpeed(20000);
      stepper.setAcceleration(40000.0);
      stepper.setSpeed(trackspeed);
      stepper.setPinsInverted(true, false, false);

    // tmcStpper driver settings
    driver.begin();
    driver.toff(4);
    //driver.blank_time(24);
    driver.rms_current(800); // mA
    driver.microsteps(256);
    //driver.TCOOLTHRS(0xFFFFF); // 20bit max // according to TMC website coolstep and stall guard don't work at low speeds so not implemented here
    //driver.semin(5);
    //driver.semax(2);
    //driver.sedn(0b01);
    //driver.SGTHRS(STALL_VALUE);

      

}
      
    
    

//----------------------------------------------------------------------------------------------------------
void loop() {
  
 // read and sum the battery voltage j times
  if (j < 20){
    batVal = analogRead(batPin);
    batSum = batSum + batVal;
    j++;
  }
  
// Send battery voltage over bluetooth and serial once in a while
currentmillis = millis();
  if ((currentmillis - previousmillis) > 1000){ //set time interval for calculating and sending the 
    previousmillis = currentmillis;
    batAvg = batSum / j;
    batVolt = batAvg / 425.00; // calc the battery voltage - this should probably be a nonlinear correction for esp32 ADC but we just want a rough idea. Full battery voltaage is 8.4 volts and onboard voltage divider drops that to about 3
    batSum = 0;                // reset the averging values
    j = 0;
    if (batVolt < 7) digitalWrite(ledPin, HIGH); // low battery indicator light turned on if voltage drops below 7V
    if (batVolt > 7.2) digitalWrite(ledPin, LOW); // low battery turned off if voltage returns to above 7.2 - this gives hysteresis to stop and blinking around the low battery level
     

 //for sending the voltage over serial and graphing with python matplotlib   
    // Serial.print(k);  
    // Serial.print(',');
    // Serial.println(batVolt);
    // Serial.print('\n');
    //k++;

    char batVstring[8]; // char buffer to hold the battery voltage
    dtostrf(batVolt, 1, 2, batVstring); // float_val, min_width, digits_after_decimal, char_buffer -- translate the battery voltage float into a string for sending over BLE
      if (deviceConnected) {  // Send battery voltage over BLE
          pTxCharacteristic->setValue(batVstring);
          pTxCharacteristic->notify();
          }
  }

        // For BLE -- disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // For BLE -- connecting
    if (deviceConnected && !oldDeviceConnected) {
		// do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
    
    //Slew when physical buttons on tracker are pressed
    slew2State = digitalRead(slew2Pin);
    if (slew2State == HIGH)
    {
          stepper.setCurrentPosition(0);
          Serial.println("button");
          driver.microsteps(32);
          stepper.moveTo(-1000500);
           while (slew2State == HIGH)
            {
            stepper.run();
            slew2State = digitalRead(slew2Pin); 
            }
          stepper.stop();
          while (stepper.distanceToGo() != 0)
          {
            stepper.run();
          }
          
          driver.microsteps(256);
          stepper.setSpeed(trackspeed);
    }
     slew1State = digitalRead(slew1Pin);
    if (slew1State == HIGH)
    {
          stepper.setCurrentPosition(0);
          Serial.println("button");
          driver.microsteps(32);
          stepper.moveTo(1000500);
           while (slew1State == HIGH)
            {
            stepper.run();
            slew1State = digitalRead(slew1Pin); 
            }
          stepper.stop();
          while (stepper.distanceToGo() != 0)
          {
            stepper.run();
          }
          
          driver.microsteps(256);
          stepper.setSpeed(trackspeed);
    }
    
    // The following if statements determine the running condition for the stepper which can be slew, track or timelapse
    if (slew == true)  stepper.run();    // run stepper during slews - make a step if it is time - acceleration and deceleration handled by accel stepper lib
    if (slew == false && track == true) stepper.runSpeed();  // run stepper with no acceleration during tracking
    if (timelapse == true && track == false){   // Starts a timelapse when the timelapse button pressed on app if track mode is off. The following interprets the timelapse setting and steps to new positions
       stepper.setCurrentPosition(0);
       driver.microsteps(32);
       int stepsPerShot = degrees / shots * 6400;
       int i = 0;
       while (i <= shots)
       {
         currentmillis = millis(); // tracks time and enters the slew between photos if the elapsed time since last shot is greater than the time at each step or the shuttermillis variable
         if ((currentmillis - previousmillis) > shutterMillis){
          previousmillis = currentmillis;
          // run stepper to new position in timelapse
          stepper.setCurrentPosition(0);
          stepper.moveTo(stepsPerShot);
          while(stepper.distanceToGo() > 0){
            stepper.run();
            }
          delay(shutterMillis/6); // wait 1/6th of the total time at this timelapse position to settle the mount before firing shutter
          if(cameraType == 0) A900.shutterNow();      // fire shutter with IR LED - multicamera library
          if(cameraType == 1) 
          Serial.println(i);
        i++;
        }
       }
      timelapse = false; // resets the timelapse boolean to re-enter track/slew mode
      driver.microsteps(256);
    }
}