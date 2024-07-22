// Zwift budget version for DC motor
// I felt hobbyist so I redid almost everything but the bluetooth stuff, because I used cheaper alternative parts for the build and much of the code was incompatible
// Original work by pete_ev @ instructables.com https://www.instructables.com/Zwift-Interface-for-Dumb-Turbo-Trainer/
// Miikka Kosonen 2022

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>


// DEFINES
#define BLINKER_DELAY 500
#define MANUAL_DELAY 3000 // How long to wait before accepting command from bluetooth after manual command

// I/O define
#define ENABLE 16 
#define MOTOR_RUN_UP 17
#define MOTOR_RUN_DOWN 4
//Buttons
#define BTN_UP 14
#define BTN_DOWN 12
//Limit switches
#define LIMIT_UP 36
#define LIMIT_DOWN 39
//Status LEDs
#define LED_GREEN 2
#define LED_YELLOW 15
#define LED_RED 13
// pot.meters SWITCHED WAS 32 <> 33
#define POT_DOWN_GRADE_LIMIT 33
#define POT_UP_GRADE_LIMIT 32

int_least16_t oldGrade = 0; //<-- oli grade
int_least16_t setpGrade = 12345; //<-- oli grade
int_least16_t currGrade = 0;    //Current gradient that the trainer is set to
int_least16_t maxGrade = 1000;
int_least16_t minGrade = -500;
bool calibrationDone = false; 

unsigned long moveDelay = 0; // time in ms from command to start moving
unsigned long range = 1000;             //range of travel of the actuator, in millis() from lo to hi
unsigned long currentPos = 0;
float downMultiplier = 1.0; // when going the other direction, speed might be more or less, so the current position is adjusted with multiplier (1000 = 1.000)

unsigned long millisLast, millisNow;
unsigned long manualDelay = MANUAL_DELAY; 

bool blinker = LOW;

BLEServer* pServer = NULL;
BLECharacteristic* pIndoorBike = NULL;
//BLECharacteristic* pResistanceRange = NULL;
//BLECharacteristic* pPowerRange = NULL;
BLECharacteristic* pFeature = NULL;
BLECharacteristic* pControlPoint = NULL;
BLECharacteristic* pStatus = NULL;

BLEAdvertisementData advert;
BLEAdvertisementData scan_response;
BLEAdvertising *pAdvertising;

bool deviceConnected = false;
bool oldDeviceConnected = false;
int value = 0;                   //This is the value sent as "nothing".  We need to send something for some of the charactistics or it won't work.

#define FTMSDEVICE_FTMS_UUID "00001826-0000-1000-8000-00805F9B34FB"
#define FTMSDEVICE_INDOOR_BIKE_CHAR_UUID "00002AD2-0000-1000-8000-00805F9B34FB"
//#define FTMSDEVICE_RESISTANCE_RANGE_CHAR_UUID "00002AD6-0000-1000-8000-00805F9B34FB"
//#define FTMSDEVICE_POWER_RANGE_CHAR_UUID "00002AD8-0000-1000-8000-00805F9B34FB"
#define FTMSDEVICE_FTMS_FEATURE_CHAR_UUID "00002ACC-0000-1000-8000-00805F9B34FB"
#define FTMSDEVICE_FTMS_CONTROL_POINT_CHAR_UUID "00002AD9-0000-1000-8000-00805F9B34FB"
#define FTMSDEVICE_FTMS_STATUS_CHAR_UUID "00002ADA-0000-1000-8000-00805F9B34FB"

//response/acknowledgement to send to the client after writing to control point
uint8_t replyDs[3] = {0x80, 0x00, 0x01};

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Zwift turbo trainer resistance controller");
  Serial.println("Budget version by Miikka Kosonen 2022,");
  Serial.println("original work pete_ev @ instructables.com");
  Serial.println("https://www.instructables.com/Zwift-Interface-for-Dumb-Turbo-Trainer/");

  pinMode(LED_BUILTIN, OUTPUT);

//  pinMode(ENABLE, OUTPUT); // << replace with ledcAttachPin(ledPin, ledChannel) when ready for PWM
  
  pinMode(MOTOR_RUN_UP, OUTPUT);
  pinMode(MOTOR_RUN_DOWN, OUTPUT);

  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);
 
  pinMode(BTN_UP, INPUT);
  pinMode(BTN_DOWN, INPUT);
  
  pinMode(LIMIT_UP, INPUT);
  pinMode(LIMIT_DOWN, INPUT);
  
  setupMotorSpeed();

// BLE BLE BLE

  //Setup BLE
  Serial.println("Creating BLE server...");
  BLEDevice::init("ESP32");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  Serial.println("Define service...");
  BLEService *pService = pServer->createService(FTMSDEVICE_FTMS_UUID);

  // Create BLE Characteristics
  Serial.println("Define characteristics");
  pIndoorBike = pService->createCharacteristic(FTMSDEVICE_INDOOR_BIKE_CHAR_UUID,BLECharacteristic::PROPERTY_NOTIFY);
  pIndoorBike->addDescriptor(new BLE2902());
  pControlPoint = pService->createCharacteristic(FTMSDEVICE_FTMS_CONTROL_POINT_CHAR_UUID,BLECharacteristic::PROPERTY_INDICATE | BLECharacteristic::PROPERTY_WRITE);
  pControlPoint->addDescriptor(new BLE2902());
  pFeature = pService->createCharacteristic(FTMSDEVICE_FTMS_FEATURE_CHAR_UUID,BLECharacteristic::PROPERTY_READ);
  pFeature->addDescriptor(new BLE2902());
  pStatus = pService->createCharacteristic(FTMSDEVICE_FTMS_STATUS_CHAR_UUID,BLECharacteristic::PROPERTY_NOTIFY);
  pStatus->addDescriptor(new BLE2902());

  // Start the service
  Serial.println("Staring BLE service...");
  pService->start();

  // Start advertising
  Serial.println("Define the advertiser...");
  pAdvertising = BLEDevice::getAdvertising();
  
  pAdvertising->setScanResponse(true);  
  pAdvertising->addServiceUUID(FTMSDEVICE_FTMS_UUID);
  pAdvertising->setMinPreferred(0x06);  // set value to 0x00 to not advertise this parameter
  Serial.println("Starting advertiser...");
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");

}


byte calibrate() {
  static byte currentStep = 0;
  static unsigned long calibrateDelay = 0;
    
  bool limit_down = digitalRead(LIMIT_DOWN);
  bool limit_up = digitalRead(LIMIT_UP);
  byte motorCommands = 0;

  calibrateDelay = calibrateDelay + (millisNow - millisLast);        

  switch(currentStep) {
    case(0):
      Serial.println("Calibrating start >> Running to DOWN LIMIT...");
      currentStep++;
      calibrateDelay = 0;
      break;    
      
    case(1): 
      if(!limit_down) {
        motorCommands = 2;
      } else {
        Serial.print("DOWN LIMIT found, took ");
        Serial.println(calibrateDelay);
        motorCommands = 0;
        calibrateDelay = 0;
        moveDelay = 0;
        currentStep++;
        Serial.println("Calculating moveDelay...");
      }
      break;

    case(2): // The actuator is touching the down limit, calculating moveDelay
      if(limit_down) {
        motorCommands = 1;
      } else {
        Serial.print("Movedelay calculated ");
        Serial.println(calibrateDelay);
        moveDelay = calibrateDelay;
        calibrateDelay = 0;
        currentStep++;
        Serial.println("Calculating range (going up)...");
      }
      break;
      
    case(3): //The actuator is just off the low delay, driving until the UP LIMIT to calculate the distance
     if(limit_up) {
        Serial.print("Range calculated ");
        Serial.println(calibrateDelay);
        motorCommands = 0;
        range = calibrateDelay;
        calibrateDelay = 0;
        currentStep++;
        Serial.println("Calculating run down multiplier...");
     } else {
        motorCommands = 1;
     }
      break;

    case(4): // Driving back to low to calculate the multiplier of speed up vs down
      if(limit_down) {
          motorCommands = 0;
          downMultiplier = float(range) / float(calibrateDelay - moveDelay);
          Serial.print("Down multiplier calculated ");
          Serial.println(downMultiplier);
          calibrateDelay = 0;
          currentStep++;
          Serial.println("Calibration done");
      } else {
        motorCommands = 2;
      }
      break;
      
    case(5): // Done
      calibrationDone = true;
      motorCommands = 0;
      break;
  }

  return(motorCommands); 
}

byte aspire_to_grade_setpoint() {
  static byte currentStep = 0;
// A difference of 1% in grade causes action
  switch(currentStep) {
    case(0): //ready
      if((setpGrade + 50) / 100  > (currGrade + 50) / 100 && currGrade < maxGrade) {
        currentStep = 1;
      } else if ((setpGrade + 50) / 100 < (currGrade + 50) / 100 && currGrade > minGrade) {
        currentStep = 2;
      }
      return(0);
      break;
    case(1): //go up
      if(currGrade >= setpGrade) {currentStep = 0; manualDelay = MANUAL_DELAY; return(0); }
      else {return(1);}
      break;
    case(2): //go down
      if(currGrade <= setpGrade) {currentStep = 0; manualDelay = MANUAL_DELAY; return(0); }
      else {return(2);}
      break;
  }
}

byte read_manual_switches(byte motorCommands) {
  static bool manualActive = false;
  bool runUp = digitalRead(BTN_UP);
  bool runDown = digitalRead(BTN_DOWN);
  
  if(runUp) {
    if(!manualActive) {Serial.println("UP BUTTON");}
    motorCommands = bitSet(motorCommands,0);
    motorCommands = bitClear(motorCommands,1);
  } else if(runDown) {
    if(!manualActive) {Serial.println("DOWN BUTTON");}
    motorCommands = bitClear(motorCommands,0);
    motorCommands = bitSet(motorCommands,1);
  }
  
  if(!runUp && !runDown && manualActive) {manualDelay = MANUAL_DELAY; }
  else if (runUp || runDown) {manualDelay = 0; }
  manualActive = runUp || runDown;

  return motorCommands;
}

byte read_limit_sensors(byte motorCommands) {
  bool limitUp = digitalRead(LIMIT_UP);
  bool limitDown = digitalRead(LIMIT_DOWN);
  
  if(limitUp) {
    if(currentPos != range) {Serial.println("UP LIMIT");}
    motorCommands = bitClear(motorCommands,0);
    currentPos = range;
  } else if(limitDown) {
    if(currentPos != 0) {Serial.println("DOWN LIMIT");}
    motorCommands = bitClear(motorCommands,1);
    currentPos = 0;
  }
  
  digitalWrite(LED_RED, limitUp || limitDown ? HIGH : LOW);

  return motorCommands;
}

void read_grade_pots() {
    static int_least16_t oldMin, oldMax;
     minGrade = map(analogRead(POT_DOWN_GRADE_LIMIT), 0, 4095, -10, 0) * 100;   
     maxGrade = map(analogRead(POT_UP_GRADE_LIMIT), 0, 4095, 6, 15) * 100;
    if (minGrade != oldMin || maxGrade != oldMax) {
     Serial.print("SCALING (x0.01) Down:");
     Serial.print(minGrade);
     Serial.print(" Up: ");
     Serial.println(maxGrade);
     oldMin = minGrade; oldMax = maxGrade;
    }
}

void setupMotorSpeed() {
// debug
//  int hz = map(analogRead(POT_DOWN_GRADE_LIMIT), 0, 4095, 10000,20000);
//  int duty = map(analogRead(POT_UP_GRADE_LIMIT), 0, 4095, 50, 255);
//  Serial.print("hz: ");
//  Serial.print(hz);
//  Serial.print(" Duty(255): ");
//  Serial.println(duty);

  int hz = 17500;
  int duty = 209;
  ledcSetup(0, hz, 8);
  ledcAttachPin(ENABLE, 0);
  ledcWrite(0, duty);
}

void read_serial_input() {
  if(Serial.available()) {
    setpGrade = Serial.readStringUntil('\n').toInt() * 100;
    Serial.print("Grade setpoint received from serial (x0.01): ");
    Serial.println(setpGrade);  
  }
}

void bluetoothStuff() {
    char windStr[6];
    char gradeStr[6];
    char rStr[4];
    char wStr[4];
  
      //This is a fiddle to get the machine to show some power data for testing
      uint8_t bikeData[19] = {};  //set all the bytes to zero .  If we're only transmitting power, there's no point transmitting beyond byte 18
      bikeData[0] = 0x41;   //set bit 6 of byte 0 to say power data present
      bikeData[15] = 0x01;  //set bits 15 and 16 for the power (note that this is a signed int16
      bikeData[16] = 0x00; 
      uint8_t flags = bikeData[0];
      uint8_t dataToSend[4] = {flags, bikeData[13], bikeData[14], bikeData[15]};
      pIndoorBike->setValue(dataToSend,4);
      pIndoorBike->notify();

      //configure machine features
      uint8_t feature[8] = {0x80, 0x40, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00};   //2^7 = Resistance level+ 2^14 = power measurement, 2^13 = bike simulation.
      pFeature->setValue(feature,8);
      pStatus->setValue(value);
      pStatus->notify();

      //Get the data written to the control point
      std::string rxValue = pControlPoint->getValue();
      if (rxValue.length() == 0) {
;//        Serial.println("No data received...");
      } else {
       switch(rxValue[0]) {
          case 0x00:  //Request control
            //reply with 0x80, 0x00, 0x01 to say OK
            replyDs[1] = rxValue[0];
            pControlPoint->setValue(replyDs, 3);
            pControlPoint->indicate();
            break;
          case 0x01: //reset
            //reply with 0x80, 0x01, 0x01 to say OK
            replyDs[1] = rxValue[0];
            pControlPoint->setValue(replyDs, 3);
            pControlPoint->indicate();
            break;
          case 0x07:  //Start/resume
            //reply with 0x80, 0x07, 0x01 to say OK
            replyDs[1] = rxValue[0];
            pControlPoint->setValue(replyDs, 3);
            pControlPoint->indicate();
            break; 
         case 0x11:  //receive simulation parameters
              /*
              In the case of Zwift:
              Wind is always constant, even with an aero boost (you just go faster)
              When trainer difficulty is set to maximum, the grade is as per the BLE spec (500 = 5%)
              Rolling resistance only ever seems to be 0 or 255.  255 occurs on mud/off road sections but not always (short sections only, like a patch of sand?)
              Wind coefficient doesn't change during a ride, even with an aero boost.  It may change depending on bike used in the game.*/

              int16_t wind = rxValue[1] << 8;
              wind |= rxValue[0];

              uint8_t Rres = rxValue[4];
              uint8_t Wres = rxValue[5];

              setpGrade = rxValue[4] << 8;
              setpGrade |= rxValue[3];

              if(oldGrade != setpGrade) {Serial.print("Grade setpoint received from bluetooth (x0.01: "); Serial.println(setpGrade); oldGrade = setpGrade;}
              //Pulse LED to show data received
              digitalWrite(LED_GREEN, 0);
              delay(20);
              break;
         }
       }

}

void loop() {
  
// put your main code here, to run repeatedly:
  millisLast = millisNow;
  millisNow = millis();

   // Blinker that can be used anywhere
  static unsigned long blinkerDelay = BLINKER_DELAY;
  blinkerDelay = max(0, int(blinkerDelay - (millisNow - millisLast)));
  if(blinkerDelay == 0) {blinkerDelay = BLINKER_DELAY; blinker = !blinker; }
  digitalWrite(LED_BUILTIN, blinker);  //Blink the onboard LED to show run state
  
    read_grade_pots();
    read_serial_input();

    currGrade = map(currentPos, 0, range, minGrade, maxGrade);

    // ** Motor commands (priority == limit sensors > manual > bluetooth) 
    manualDelay = max(0, int(manualDelay - (millisNow - millisLast)));

    static byte oldCommands = 0;
    byte motorCommands = 0; //b0 = motor run high, b1 = motor run low
    if(!calibrationDone && setpGrade != 12345) {motorCommands = calibrate();}
    if(calibrationDone) {if(!manualDelay) {motorCommands = aspire_to_grade_setpoint();}}
    motorCommands = read_manual_switches(motorCommands);
  
    // Position calculation
    if(oldCommands != motorCommands) { switch(motorCommands) {
      case(0): Serial.print("Stopped at "); Serial.print(currentPos); Serial.print("/"); Serial.print(range); 
               Serial.print(", equals grade (x0.01) "); Serial.print(currGrade); Serial.print(", target grade was: ");
               Serial.println(setpGrade); break;
      case(1): currentPos = max(0,int(currentPos - moveDelay)); break;
      case(2): currentPos = min(range,currentPos + moveDelay); break;
    }; oldCommands = motorCommands;};

    switch(motorCommands) {
      case(1): currentPos = min(range, currentPos + (millisNow - millisLast)); break;
      case(2): currentPos = max(0, int(currentPos) - int(float(millisNow - millisLast) * downMultiplier)); break;
    }

    // Read limit sensors purposefully after position calculation
    // because here we will reset position if it's hitting the limit
    motorCommands = read_limit_sensors(motorCommands);
    digitalWrite(MOTOR_RUN_UP, bitRead(motorCommands,0));
    digitalWrite(MOTOR_RUN_DOWN, bitRead(motorCommands,1));
    digitalWrite(LED_YELLOW, manualDelay ? blinker : motorCommands);
  
   if (deviceConnected) {
       bluetoothStuff();
       digitalWrite(LED_GREEN, HIGH);
   } else {
       digitalWrite(LED_GREEN, blinker);
   }
   
   // disconnecting
   if (!deviceConnected && oldDeviceConnected) {
       delay(300); // give the bluetooth stack the chance to get things ready
       pServer->startAdvertising(); // restart advertising
       Serial.println("Nothing connected, start advertising");
       oldDeviceConnected = deviceConnected;
   }
       // connecting
   if (deviceConnected && !oldDeviceConnected) {
       Serial.println("Trying to connect device...");
       // do stuff here on connecting
       setpGrade = 1;
       oldDeviceConnected = deviceConnected;
   }

 

  delay(50);
}
