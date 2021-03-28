////////////////////////////////////////////////////////////////////////////////
/// @file   Rioduino.hpp
/// @author David Stalter
///
/// @details
/// This is the class declaration and implementation of functionality on the
/// RIOduino for a FRC robot.  It handles offloaded sensors/peripherals and
/// communicates to the roboRIO via I2C on the MXP port.
///
/// Copyright (c) 2021 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// INCLUDES
#include <Adafruit_Sensor.h>                  // for base sensor support
#include <Adafruit_BNO055.h>                  // for BNO055 library
#include <Wire.h>                             // for I2C communication
#include "RoborioRioduinoSharedData.hpp"      // for shared data structures


////////////////////////////////////////////////////////////////////////////////
/// Class: YtaRobot
///
/// Class that contains the declarations and functions for running the RIOduino.
///
////////////////////////////////////////////////////////////////////////////////
class YtaRioduino
{
public:
  
  // Wrappers for the setup/loop Arduino functions
  static void Initialize();
  static void Run();
  
private:
  
  // Makes for easier use/readability of interacting with I2C commands and data
  typedef RoborioRioduinoSharedData::I2cCommand I2cCommand;
  typedef RoborioRioduinoSharedData::I2cData I2cData;

  // Support function for displaying a message if prints are enabled
  template <typename T>
  inline static void DisplayMessage(T message)
  {
    if (DEBUG_PRINTS)
    {
      Serial.println(message);
    }
  }
  
  // Member functions
  static void InterruptHandler();
  static void I2cOnReceive(int bytesReceived);
  static void I2cOnRequest();
  static void CheckForAndProcessI2cCommand();
  static void BuildI2cData();
  static void GetGyroData();
  static void HeartBeat();
  
  // Member variables
  static Adafruit_BNO055    m_Bno055;
  static I2cCommand         m_I2cCommand;
  static I2cData            m_I2cData;
  static volatile bool      m_bNewI2cCommandAvailable;
  static volatile bool      m_bReadNewGyroCenter;
  static volatile bool      m_bCollectSensorData;
  static volatile bool      m_bI2cDataRead;
  static double             m_RobotAngle;
  static double             m_RobotAbsoluteAngle;
  static double             m_RobotRelativeAngle;
  static double             m_RobotCenterPoint;
  
  // Constants
  static const int          ROBORIO_SIGNAL_PIN          = 2;
  static const int          RIODUINO_SIGNAL_PIN         = 3;
  static const int          HEALTH_LED_PIN              = 5;
  static const int          DEBUG_BLUE_LED_PIN          = 6;
  static const int          DEBUG_GREEN_LED_PIN         = 7;
  static const int          BNO055_SENSOR_ID            = 55;
  static const int          HEART_BEAT_RATE_MS          = 1000;
  static const uint8_t      I2C_BUFFER_MARKER           = 0xBB;
  static constexpr double   ONE_HUNDRED_EIGHTY_DEGREES  = 180.0;
  static constexpr double   THREE_HUNDRED_SIXTY_DEGREES = 360.0;

  static const bool         DEBUG_PRINTS                = false;
  static const bool         DEBUG_I2C_TRANSACTIONS      = false;
  static const bool         DEBUG_GYRO_READINGS         = false;

  // Constructor, destructor, copy, assignment
  YtaRioduino();
  ~YtaRioduino();
  YtaRioduino(const YtaRioduino &);
  YtaRioduino & operator=(const YtaRioduino &);
};

// STATIC MEMBER DATA
Adafruit_BNO055         YtaRioduino::m_Bno055                   = Adafruit_BNO055(BNO055_SENSOR_ID);
YtaRioduino::I2cCommand YtaRioduino::m_I2cCommand;
YtaRioduino::I2cData    YtaRioduino::m_I2cData;
volatile bool           YtaRioduino::m_bNewI2cCommandAvailable  = false;
volatile bool           YtaRioduino::m_bReadNewGyroCenter       = false;
volatile bool           YtaRioduino::m_bCollectSensorData       = false;
volatile bool           YtaRioduino::m_bI2cDataRead             = false;
double                  YtaRioduino::m_RobotAngle               = 0.0;
double                  YtaRioduino::m_RobotAbsoluteAngle       = 0.0;
double                  YtaRioduino::m_RobotRelativeAngle       = 0.0;
double                  YtaRioduino::m_RobotCenterPoint         = 0.0;


////////////////////////////////////////////////////////////////////////////////
/// Method: setup
///
/// Details:  The Arduino initialization function called during controller
///           start up. 
////////////////////////////////////////////////////////////////////////////////
void setup()
{
  YtaRioduino::Initialize();
}


////////////////////////////////////////////////////////////////////////////////
/// Method: loop
///
/// Details:  The continuous Arduino background user loop.
////////////////////////////////////////////////////////////////////////////////
void loop()
{
  YtaRioduino::Run();
}


////////////////////////////////////////////////////////////////////////////////
/// Method: Initialize
///
/// Details:  Initializes data/peripherals for the RIOduino.
////////////////////////////////////////////////////////////////////////////////
void YtaRioduino::Initialize()
{
  // Start up the serial port
  Serial.begin(115200);
  DisplayMessage("FRC 120 RIOduino.");
  
  // Configure debug pins
  pinMode(HEALTH_LED_PIN, OUTPUT);
  pinMode(DEBUG_BLUE_LED_PIN, OUTPUT);
  pinMode(DEBUG_GREEN_LED_PIN, OUTPUT);
  
  // Indicate initialization is occurring
  digitalWrite(HEALTH_LED_PIN, HIGH);
  digitalWrite(DEBUG_BLUE_LED_PIN, HIGH);
  digitalWrite(DEBUG_GREEN_LED_PIN, HIGH);
  
  // Configure communication pins
  pinMode(ROBORIO_SIGNAL_PIN, INPUT);
  pinMode(RIODUINO_SIGNAL_PIN, OUTPUT);
  
  // Connect the interrupt handler
  attachInterrupt(digitalPinToInterrupt(ROBORIO_SIGNAL_PIN), InterruptHandler, RISING);
  
  // Open the I2C port
  Wire.begin(RoborioRioduinoSharedData::I2C_DEVICE_ADDRESS);
  Wire.onReceive(I2cOnReceive);
  Wire.onRequest(I2cOnRequest);
  
  // Clear I2C data and set constant fields
  memset(&m_I2cData, 0U, sizeof(m_I2cData));
  
  // Initialize the 9-axis sensor
  while (!m_Bno055.begin())
  {
    // There was a problem detecting the BNO055 ... check your connections
    DisplayMessage("No BNO055 detected... check wiring or I2C ADDR!");

    // Delay before trying again
    const unsigned int ONE_SECOND_DELAY_MS = 1000;
    delay(ONE_SECOND_DELAY_MS);
  }
  
  m_Bno055.setExtCrystalUse(true);
  
  // Initialization complete, turn visual indication off
  digitalWrite(HEALTH_LED_PIN, LOW);
  digitalWrite(DEBUG_BLUE_LED_PIN, LOW);
  digitalWrite(DEBUG_GREEN_LED_PIN, LOW);
}


////////////////////////////////////////////////////////////////////////////////
/// Method: Run
///
/// Details:  Main loop for the RIOduino program.
////////////////////////////////////////////////////////////////////////////////
void YtaRioduino::Run()
{
  while (true)
  {
    while (!m_bCollectSensorData)
    {
      HeartBeat();
    }
    GetGyroData();
    BuildI2cData();
    m_bCollectSensorData = false;
  }
  
  // This approach is an interurpt based communication mechanism.
  // The roboRIO will interrupt the RIOduino when it wants new data.
  // The loops waiting on a state change will poll for a new I2C
  // command to come through and be processed.
  
  // Wait for the interrupt to come through
  while (!m_bCollectSensorData)
  {
    CheckForAndProcessI2cCommand();
    HeartBeat();
  }
  
  // Get new information from the sensor
  GetGyroData();
  
  // Build an I2C response packet
  BuildI2cData();
  
  // Send a response back to the roboRIO to indicate data is ready
  digitalWrite(RIODUINO_SIGNAL_PIN, HIGH);
  
  // Wait for the roboRIO to read the I2C data
  while (!m_bI2cDataRead)
  {
    CheckForAndProcessI2cCommand();
    HeartBeat();
  }
  
  // The roboRIO has now read the new I2C data, reset control variables
  m_bCollectSensorData = false;
  m_bI2cDataRead = false;
  digitalWrite(RIODUINO_SIGNAL_PIN, LOW);
}


////////////////////////////////////////////////////////////////////////////////
/// Method: InterruptHandler
///
/// Details:  Interrupt handler for when the roboRIO sends a trigger signal.
////////////////////////////////////////////////////////////////////////////////
void YtaRioduino::InterruptHandler()
{
  m_bCollectSensorData = true;
}


////////////////////////////////////////////////////////////////////////////////
/// Method: HeartBeat
///
/// Details:  Support function to show the RIOduino is still executing.
////////////////////////////////////////////////////////////////////////////////
void YtaRioduino::HeartBeat()
{
  static int heartBeat = 1;
  static bool bHealthLedState = false;
  static unsigned int lastHeartBeatTimeMs = 0U;
  unsigned int currentTimeMs = millis();
  
  if ((currentTimeMs - lastHeartBeatTimeMs) > HEART_BEAT_RATE_MS)
  {
    if (DEBUG_PRINTS)
    {
      Serial.print("HeartBeat: ");
      Serial.println(heartBeat++);
    }
    
    // No matter what, toggle the health LED
    digitalWrite(HEALTH_LED_PIN, static_cast<int>(bHealthLedState));
    bHealthLedState = !bHealthLedState;
    
    lastHeartBeatTimeMs = currentTimeMs;
  }
}


////////////////////////////////////////////////////////////////////////////////
/// Method: I2cOnReceive
///
/// Details:  Receives I2C data from the roboRIO.
////////////////////////////////////////////////////////////////////////////////
void YtaRioduino::I2cOnReceive(int bytesReceived)
{
  // Clear the buffer with a marker in case there's a need to debug
  memset(&m_I2cCommand, I2C_BUFFER_MARKER, sizeof(m_I2cCommand));
  
  // Get a pointer to the destination location
  uint8_t * pDest = reinterpret_cast<uint8_t *>(&m_I2cCommand);
  
  // Read the bytes into the buffer
  for (int i = 0; i < bytesReceived; i++)
  {
    *pDest++ = Wire.read();
  }
  
  m_bNewI2cCommandAvailable = true;
  
  if (DEBUG_I2C_TRANSACTIONS)
  {
    Serial.print("On receive: ");
    
    pDest = reinterpret_cast<uint8_t *>(&m_I2cCommand);
    for (int i = 0; i < bytesReceived; i++)
    {
      Serial.print(*pDest++, HEX);
    }
    
    Serial.println();
  }
}


////////////////////////////////////////////////////////////////////////////////
/// Method: I2cOnRequest
///
/// Details:  Sends I2C data to the roboRIO.
////////////////////////////////////////////////////////////////////////////////
void YtaRioduino::I2cOnRequest()
{
  Wire.write(reinterpret_cast<byte *>(&m_I2cData), sizeof(m_I2cData));
  
  m_bI2cDataRead = true;

  if (DEBUG_I2C_TRANSACTIONS)
  {
    Serial.print("On request: ");
    const byte * pData = reinterpret_cast<byte *>(&m_I2cData);
    for (size_t i = 0; i < sizeof(m_I2cData); i++)
    {
      Serial.print(*pData++, HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}


////////////////////////////////////////////////////////////////////////////////
/// Method: CheckForAndProcessI2cCommand
///
/// Details:  Looks for a new I2C command to have arrived and processes it.
////////////////////////////////////////////////////////////////////////////////
void YtaRioduino::CheckForAndProcessI2cCommand()
{
  // Validate the command that was received
  if (m_bNewI2cCommandAvailable)
  {
    // Check the header and footer
    if (m_I2cCommand.m_Header == RoborioRioduinoSharedData::I2C_HEADER_DATA &&
        m_I2cCommand.m_Footer == RoborioRioduinoSharedData::I2C_FOOTER_DATA)
    {
      // Process the command
      switch (m_I2cCommand.m_CommandSelection)
      {
        case RoborioRioduinoSharedData::GYRO_READ_NEW_CENTER:
        {
          m_bReadNewGyroCenter = true;
          break;
        }
        default:
        {
          // Do nothing in case it was a bad packet
          break;
        }
      }
    }
    else
    {
      if (DEBUG_PRINTS)
      {
        Serial.println("Invalid I2C metadata.  Dumping buffer...");
        
        uint8_t * pBuffer = reinterpret_cast<uint8_t *>(&m_I2cCommand);
        for (size_t i = 0; i < sizeof(m_I2cCommand); i++)
        {
          Serial.print(*pBuffer++, HEX);
          Serial.print(" ");
        }
        Serial.println();
        }
    }
    
    // Command was processed, indicate as such
    m_bNewI2cCommandAvailable = false;
  }
}


////////////////////////////////////////////////////////////////////////////////
/// Method: BuildI2cData
///
/// Details:  Builds the data to send over to the roboRIO.
////////////////////////////////////////////////////////////////////////////////
void YtaRioduino::BuildI2cData()
{
  // Clear the buffer with a marker in case there's a need to debug
  memset(&m_I2cData, I2C_BUFFER_MARKER, sizeof(m_I2cData));
  
  // Set the header/footer info
  m_I2cData.m_Header = RoborioRioduinoSharedData::I2C_HEADER_DATA;
  m_I2cData.m_Footer = RoborioRioduinoSharedData::I2C_FOOTER_DATA;
  
  // Indicate gyro data is being sent over
  m_I2cData.m_DataSelection = RoborioRioduinoSharedData::I2cDataSelection::GYRO_DATA;
  
  // Temporary test sending 0 -> 360
  m_I2cData.m_DataBuffer.m_GyroData.m_xAxisInfo.m_bIsNegative = false;
  m_I2cData.m_DataBuffer.m_GyroData.m_xAxisInfo.m_Angle = static_cast<uint16_t>(round(m_RobotAbsoluteAngle));
  return;
  
  // First get the robot angle since it will be manipulated before sending
  double robotAngle = m_RobotAngle;
  
  // Check if it's negative
  if (robotAngle < 0.0)
  {    
    m_I2cData.m_DataBuffer.m_GyroData.m_xAxisInfo.m_bIsNegative = true;
    
    // For simplicity with the different architectures, always send a positive angle
    robotAngle *= -1.0;
  }
  else
  {
    m_I2cData.m_DataBuffer.m_GyroData.m_xAxisInfo.m_bIsNegative = false;
  }
  
  // Set the angle, deliberately converting to an integer value
  m_I2cData.m_DataBuffer.m_GyroData.m_xAxisInfo.m_Angle = static_cast<uint16_t>(round(robotAngle));
}


////////////////////////////////////////////////////////////////////////////////
/// Method: GetGyroData
///
/// Details:  Reads information from the BNO055 9-axis sensor.
////////////////////////////////////////////////////////////////////////////////
void YtaRioduino::GetGyroData()
{
  // This will only be called on demand, no need for time controls
  
  // For normalization to arbitrary angle as center:
  //
  // Defaults to zero
  // double m_CenterSetPointDegrees = 0.0;
  // m_RobotAbsoluteAngle = bnoSensorEvent.orientation.x;
  // m_RobotRelativeAngle = m_RobotAbsoluteAngle - m_CenterSetPointDegrees;
  // if (m_RobotRelativeAngle < 0.0)
  // {
  //   // Negative angles must be normalized from 360 (the addition here is actually subtraction)
  //   m_RobotRelativeAngle += THREE_HUNDRED_SIXTY_DEGREES;
  // }
  
  // Get a new sensor event
  sensors_event_t bnoSensorEvent;
  m_Bno055.getEvent(&bnoSensorEvent);
  
  // Save off a new center if one was requested
  if (m_bReadNewGyroCenter)
  {
    DisplayMessage("Reading new center...");
    m_RobotCenterPoint = bnoSensorEvent.orientation.x;
    m_bReadNewGyroCenter = false;
  }
  
  // The absolute angle is what the sensor reports.
  // The relative angle needs to be computed from the absolute angle.
  m_RobotAbsoluteAngle = bnoSensorEvent.orientation.x;
  m_RobotRelativeAngle = m_RobotAbsoluteAngle - m_RobotCenterPoint;
  
  // The relative angle can be anywhere from -360 to + 360 at this point.
  // We need an angle between 0 -> 360 only.
  // Negative angles must be normalized from 360 (the addition here is actually subtraction).
  if (m_RobotRelativeAngle < 0.0)
  {
    m_RobotRelativeAngle += THREE_HUNDRED_SIXTY_DEGREES;
  }
  
  // Now that we have an angle from 0 -> 360, convert it to -180 -> 180
  if (m_RobotRelativeAngle > ONE_HUNDRED_EIGHTY_DEGREES)
  {
    // convert left half of unit circle to negative
    // old: 360 (top) -> 270 (left) -> 180 (bottom)
    // new: 0 (top) -> -90 (left) -> -180 (bottom)
    m_RobotRelativeAngle -= THREE_HUNDRED_SIXTY_DEGREES;
  }
  
  // Robot angle is the relative angle
  m_RobotAngle = m_RobotRelativeAngle;
  
  // Update the LED debug outputs
  static bool bDebugLedPulseOn = true;
  if (bDebugLedPulseOn)
  {
    if (m_RobotAngle < 0.0)
    {
      digitalWrite(DEBUG_BLUE_LED_PIN, HIGH);
      digitalWrite(DEBUG_GREEN_LED_PIN, LOW);
    }
    else
    {
      digitalWrite(DEBUG_BLUE_LED_PIN, LOW);
      digitalWrite(DEBUG_GREEN_LED_PIN, HIGH);
    }
    
    bDebugLedPulseOn = false;
  }
  else
  {
    digitalWrite(DEBUG_BLUE_LED_PIN, LOW);
    digitalWrite(DEBUG_GREEN_LED_PIN, LOW);
    
    bDebugLedPulseOn = true;
  }
  
  if (DEBUG_GYRO_READINGS)
  {
    Serial.print("Time: ");
    Serial.print(millis());
    Serial.print(", Absolute angle: ");
    Serial.print(m_RobotAbsoluteAngle);
    Serial.print(", Relative angle: ");
    Serial.println(m_RobotRelativeAngle);
  }
}

