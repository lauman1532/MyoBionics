#include <SoftwareSerial.h>
#include <MyoBridge.h>
#include <FingerLib.h>

SoftwareSerial bridgeSerial(13, 14); // For Almond: RX3 = 13 or 52, TX3 = 14 or 51; For Arduino: RX = 2, TX = 3
MyoBridge bridge(bridgeSerial);
Finger finger[5];

// Initialisation of MyoBridge and Almond
void init_myobridge_almond()
{
  int handFlag = RIGHT;
  byte batteryLevel;
  MyoPose unlockPose;

  // Set up serial connections
  MYSERIAL.begin(38400); // MYSERAIL for compability (Serial and Serial USB), defined in FingerLib.h
  bridgeSerial.begin(38400); // baud rate = 38400 for Almond

  MYSERIAL.println("Please ensure Myoband is not connected to anything else and not in standby mode.");
  MYSERIAL.println("Searching for Myoband...");

  bridge.begin();
  MYSERIAL.println("Connected to Almond!");
  pin_assignment(handFlag); // assigns a pin to each finger
  init_motors(); // initialise motors' max, min speed and position

  batteryLevel = bridge.getBatteryLevel(); // gets battery level of Myoband
  MYSERIAL.print("Battery Level: ");
  MYSERIAL.println(batteryLevel); // prints battery level of Myoband
  check_low_battery();

  // Set up data transfer
  bridge.setPoseEventCallBack(handle_pose); // sets a function for handling pose data
  bridge.setEMGDataCallBack(get_EMG_data); // sets a function for handling data
  bridge.setEMGMode(EMG_MODE_NONE); // EMG_MODE_SEND: filtered EMG data; EMG_MODE_RAW: raw data
  bridge.setIMUDataCallBack(get_IMU_data); // sets a function for handling IMU data
  bridge.setIMUMode(IMU_MODE_SEND_DATA); // IMU_MODE_SEND_ALL; IMU_MODE_SEND_DATA: accelerometer, gyroscope, and orientation data; IMU_MODE_SEND_RAW: raw data
  bridge.enablePoseData();

  bridge.vibrate(3);
  bridge.unlockMyo();
  bridge.disableSleep(); // disables sleep mode
}

//-----------data-----------//
// Handle pose data -> Basic hand control using pose data
void handle_pose(MyoPoseData& poseData)
{
  MyoPose pose;
  pose = (MyoPose)poseData.pose; // converts pose data to MyoPose

  MYSERIAL.println(bridge.poseToString(pose));

  switch (pose)
  {
    case MYO_POSE_REST:
    {
      
    }
    
    case MYO_POSE_FINGERS_SPREAD: // When fingers are spread
    {
      openHand(); // opens all fingers
      break;
    }

    case MYO_POSE_DOUBLE_TAP: // When double tap is detected
    {
      stopHand(); //stop motors at desired position
      break;
    }

    case MYO_POSE_WAVE_IN: // When 'wave_in' motion is detected
    {
      tripodGrip(); // Assigns a tripod grip
      break;
    }

    case MYO_POSE_WAVE_OUT: // When 'wave_out' motion is detected
    {
      pointHand(); // Points finger
      break;
    }

    case MYO_POSE_FIST: // When a fist gesture is detected
    {
      closeHand(); // closes fingers
      break;
    }
  }
}

void get_EMG_data(int8_t EMGData[8]) // EMG data is 8-bit signed integers
{
  // not yet implemented
}

void get_IMU_data(MyoIMUData& IMUdata)
{
  // not yet implemented
}

//-----------init-----------//
// Attach each pin on Almond to Ada, so we can recognise the fingers (motors)
void pin_assignment(int& handFlag) // Needed for Almond (FingerLib)
{
  if (handFlag == RIGHT)
  {
    finger[0].attach(13, 4, A5); // attach(PWN pin1, PWN pin2, ADC pin)
    finger[1].attach(3, 6, A1);
    finger[2].attach(7, 8, A2);
    finger[3].attach(10, 9, A3);
    finger[4].attach(11, 12, A4);
  }
  else
  {
    finger[0].attach(5, 2, A0);
    finger[1].attach(11, 12, A4);
    finger[2].attach(10, 9, A3);
    finger[3].attach(7, 8, A2);
    finger[4].attach(3, 6, A1);
  }
}

void init_motors()
{
  int i = 0;
  do
  {
    finger[i].setPosLimits(50, 975);
    finger[i].setSpeedLimits(150, 250); // 0 - 150 leads to no movement
    i++;
  }while(i < 6);
}

//-----------hand-----------//
void openHand() // function from examples of FingerLib.h, opens hand
{
  for (int i = 0; i < 5; i++)
  {
    finger[i].open();
  }
}

void closeHand() // function from examples of FingerLib.h, closes hand
{
  unsigned long startTime = millis();
  unsigned long timeElapsed = 0;
  
  for (int i = 0; i < 5; i++)
  {
    finger[i].close();
  }
  
  while(finger[0].readPosError() > 0 && finger[1].readPosError() > 0 && finger[2].readPosError() > 0
  && finger[3].readPosError() > 0 && finger[4].readPosError() > 0)
  {
    timeElapsed = startTime - millis();
    if(timeElapsed > 5000) // prevent damaging the fingers if they can't be closed
    {
      if(finger[0].readPosError() > 0)
      {
        finger[0].stopMotor();
      }
      if(finger[1].readPosError() > 0)
      {
        finger[1].stopMotor();
      }
      if(finger[2].readPosError() > 0)
      {
        finger[2].stopMotor();
      }
      if(finger[3].readPosError() > 0)
      {
        finger[3].stopMotor();
      }
      if(finger[4].readPosError() > 0)
      {
        finger[4].stopMotor();
      }
    }
  }
}

void stopHand() // stops motors
{
  for (int i = 0; i < 5; i++)
  {
    finger[i].stopMotor();
  }
}

void tripodGrip() // performs a tripod grip
{
  for (int i = 0; i < 3; i++)
  {
    finger[i].close();
  }
}

void pointHand() // points hand
{
  for (int i = 0; i < 5; i++)
  {
    if (i != 1)
    {
      finger[i].close();
    }
  }
}

//------other functions------//
void check_low_battery()
{
  byte batteryLevel = bridge.getBatteryLevel();
  if(batteryLevel < 10)
  {
    MYSERIAL.println("Myoband: low battery!");
  }
}

//-----------main-----------//
void setup()
{
  init_myobridge_almond();
}

void loop()
{
  bridge.update(); // Keep updating data from BLE module
}


