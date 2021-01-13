  // TrackStar - Interacts with a BLE device (TrackStar app) which sends commands consisting of one letter or symbol 
  // which can be followed by a value to parse. 
  // The command letters and functions are listed here:
  // a - Resets tracking to sidereal rate after a slew. Sent when slew button released in TrackStar app
  // b - Slew forward
  // c - Slew backward
  // d - Turn tracking on
  // e - Turn tracking off
  // f - 
  // g - 
  // h - 
  // i - Slew a set angle forward
  // j - Slew a set angle backwards
   
    #include <Arduino.h>
    #include <AccelStepper.h>
    #include <TMCStepper.h>
    #include <BLEDevice.h>
    #include <BLEServer.h>
    #include <BLEUtils.h>
    #include <BLE2902.h>
    #include <multiCameraIrControl.h>

    Sony A900(4);

    const int ledPin = 21;      // the number of the LED pin
    //const int cameraled = 4;      // the number of the LED pin
    bool slew = false;
    bool track = true;
    bool timelapse = false;
    bool degreesSet = false;
    bool shotsSet = false;
    bool shutterSet = false;
    int trackspeed = 214;
    float shutterSec = 0;
    float shutterMillis = 0;
    float degrees = 0;
    int shots = 0;
    long currentmillis = 0;
    long previousmillis = 0;
    // Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
    #define dirPin 12
    #define stepPin 13
    #define motorInterfaceType 1
    

    // TMC2208 serial comms setup
 
    #define EN_PIN    19  // LOW: Driver enabled. HIGH: Driver disabled
    #define SERIAL_PORT Serial2 // TMC2209 HardwareSerial port
    #define R_SENSE 0.18  // Sense resistor for TMC2209
    TMC2208Stepper driver = TMC2208Stepper(&SERIAL_PORT, R_SENSE); // Driver setup with hardware serial

    
    // Create a new instance of the AccelStepper class:
    AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

//BLuetoot setup
//recieve text from the BLE server (android app)
BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;


std::string rxValue; // Could also make this a global var to access it in loop()

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
       rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++) {
          Serial.print(rxValue[i]);
          }
 
  

        // Do stuff based on the command received from the app
        
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
          driver.microsteps(256);
          stepper.runSpeed();
          Serial.println("track on");
          }
        // turn tracking off
        else if (rxValue.find("e") != -1) {
          track = false;
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
          if (degreesSet && shotsSet && shutterSet == true){
            timelapse = true;
          }
          }

          else if (rxValue.find("j") != -1) {
          if (degreesSet && shotsSet && shutterSet == true){
            timelapse = true;
          }
          }
      
       else if (rxValue.find("*") != -1) {
          rxValue.erase (rxValue.begin()+0);
          Serial.print("Degrees in timelapse");
          Serial.println(rxValue.c_str());
          degrees = ::atof(rxValue.c_str());
          degreesSet = true;
          }

       else if (rxValue.find("#") != -1) {
          rxValue.erase (rxValue.begin()+0);
          Serial.print("# of shots in timelapse: ");
          Serial.println(rxValue.c_str());
          shots = ::atoi(rxValue.c_str());
          shotsSet = true;
          }

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
    }
    }
};

 
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
      //camera LED pin
     // pinMode(cameraled, OUTPUT);

      //enable stepper pin
      pinMode(EN_PIN, OUTPUT);
      digitalWrite(EN_PIN, LOW);    // Enable driver in hardware

      // set initial LED state
      digitalWrite(ledPin, HIGH);

            // set initial LED state
     // digitalWrite(cameraled, HIGH);
      
      // Set the maximum speed in steps per second:
      stepper.setMaxSpeed(25000);
      stepper.setAcceleration(40000.0);
      stepper.setSpeed(trackspeed);
      stepper.setPinsInverted(true, false, false);
      
      


      
      // Set stepper driver parameters over serial

      driver.begin();               //  SPI: Init CS pins and possible SW SPI pins
      driver.pdn_disable(true);     // Use PDN/UART pin for communication
      driver.I_scale_analog(false); // Use internal voltage reference
      driver.rms_current(650);      // Set motor RMS current
      driver.mstep_reg_select(1);  // necessary for TMC2208 to set microstep register with UART
      driver.microsteps(256);     
      driver.toff(5);               // Enables driver in software
      //driver.en_pwm_mode(true);     // Enable stealthChop
      driver.pwm_autoscale(true);   // Needed for stealthChop

      

}
      
    
    

//----------------------------------------------------------------------------------------------------------
    void loop() {

     if (slew == true)  stepper.run();    // run stepper - make a step if time- accel stepper lib
     if (slew == false && track == true) stepper.runSpeed();  
     if (timelapse == true && track == false){   // Starts a timelapse when the timelapse button pressed on app if track mode is off then resets the timelapse boolean at end
       stepper.setCurrentPosition(0);
       driver.microsteps(32);
       int stepsPerShot = degrees / shots * 6400;
       int i = 0;
       while (i <= shots)
       {
         currentmillis = millis();
         if ((currentmillis - previousmillis) > shutterMillis){
          previousmillis = currentmillis;
          
          stepper.setCurrentPosition(0);
          stepper.moveTo(stepsPerShot);
          while(stepper.distanceToGo() > 0){
            stepper.run();
            }
          delay(shutterMillis/6);
          A900.shutterNow();
          Serial.println(i);
        i++;
        }
       }
      timelapse = false;
      driver.microsteps(256);
    }
    }

