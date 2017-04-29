/*
  DJI Naza (v1, v1 Lite, v2) - Minim OSD adapter
  (c) Pawelsky 20150508
  Not for commercial use

  Provides HEARTBEAT, GPS_RAW_INT, ATTITUDE, VFR_HUD, SYS_STATUS and RC_CHANNELS_RAW messages

  When using serial adapter requires:
   - Arduino Pro Mini (16Mhz/5v) board
   - NazaDecoder library v20140628 or newer
   - AltSoftSerial library 
  Refer to naza_decoder_wiring.jpg diagram for proper connection
  Works with Naza v1, v1 Lite and V2
  Check out http://www.rcgroups.com/forums/showthread.php?t=1995704 for more details

  When using CAN adapter requires:
   - Teensy 3.1 board and CAN transceiver (complie with "CPU speed" set to "96MHz (overclock)")
   - NazaCanDecoder library v20140628 or newer
   - FlexCAN library v0.1 beta (or newer if compatible)
  Refer to naza_can_decoder_wiring.jpg diagram for proper connection
  Connections can be greatly simplified using CAN bus and MicroSD shields by Pawelsky
  (see teensy_shields.jpg for installation and naza_can_decoder_wiring_shields.jpg for wiring)
  Works with Naza v1 and V2 with PMU (does not work with v1 Lite)
  Check out http://www.rcgroups.com/forums/showthread.php?t=2071772 for more details
*/

// Select the adapter type you are using (either serial or CAN) by commenting/uncommenting the right includes below
// These two lines below shall be commented out when using the CAN adapter and uncommented when using serial adapter
#include "NazaDecoderLib.h"
#include "AltSoftSerial.h"
// These two lines below shall be commented out when using the serial adapter CAN adapter and uncommented when using CAN adapter
//#include "NazaCanDecoderLib.h"
//#include "FlexCAN.h"

// Number of battery cells (only needed for CAN adapter to show proper battery percentage)
#define BATTERY_CELL_NUM 3

// By default the course over ground from GPS will be used. Uncomment the line below to use magnetometer heading instead (not recommended when using Serial adapter as it will not be tilt compensated).
//#define USE_COMPASS_HEADING

#if defined(__NAZA_DECODER_LIB_H__) && defined(__NAZA_CAN_DECODER_LIB_H__)
  #error "You cannot use both NazaDecoderLib.h and NazaCanDecoderLib.h includes. Uncomment only one of them at the beginning of the file."
#endif

#if !defined(__NAZA_DECODER_LIB_H__) && !defined(__NAZA_CAN_DECODER_LIB_H__)
  #error "You must use either NazaDecoderLib.h + AltSoftSerial.h (for serial adapter) or NazaCanDecoderLib.h + FlexCAN.h (for CAN adapter). Uncomment the right includes at the beginnning of the file."
#endif

#if defined(__NAZA_DECODER_LIB_H__) && !defined(__NAZA_CAN_DECODER_LIB_H__) && defined(__FLEXCAN_H__)
  #error "You shuld not include the FlexCAN library when using serial adapter. Comment out the FlexCAN.h include at the beginning of the file."
#endif

#if !defined(__NAZA_DECODER_LIB_H__) && defined(__NAZA_CAN_DECODER_LIB_H__) && defined(AltSoftSerial_h)
  #error "You shuld not include the AltSoftSerial library when using CAN adapter. Comment out the AltSoftSerial.h include at the beginning of the file."
#endif

#ifdef __NAZA_CAN_DECODER_LIB_H__
  #define SERIAL_PORT Serial1
  #define DECODER NazaCanDecoder
  #define DECODER_LIB NazaCanDecoderLib
  #define MAVLINK_MSG_NO 6
  #define ALTITUDE_FUN getAlt  // use barometric altitude for the CAN adapter (more precise)
  #define VSI_FUN getVsi       // use barometric VSI for the CAN adapter (more precise)
  #ifdef USE_COMPASS_HEADING
    #define HEADING_FUN getHeading
  #else
    #define HEADING_FUN getCog
  #endif  
  #ifndef __MK20DX256__
    #error "You need to use Teensy 3.1 board to use CAN adapter"
  #endif
#else
  #define SERIAL_PORT OsdSerial
  #define DECODER NazaDecoder
  #define DECODER_LIB NazaDecoderLib
  #define MAVLINK_MSG_NO 4
  #define ALTITUDE_FUN getGpsAlt  // no barometric altitude available for serial adapter - use GPS altitude (less precise)
  #define VSI_FUN getGpsVsi       // no barometric VSI available for serial adapter - use GPS VSI (less precise)
  #ifdef USE_COMPASS_HEADING
    #define HEADING_FUN getHeadingNc // No tilt compensated heading for serial adapter yet
  #else
    #define HEADING_FUN getCog
  #endif  
  AltSoftSerial SERIAL_PORT;
#endif


#define MAVLINK_START_OF_FRAME 0xFE
#define MAVLINK_HEADER_LEN     5
#define MAVLINK_CHECKSUM_LEN   2

#define MAVLINK_SYSTEM_ID    0xAA
#define MAVLINK_COMPONENT_ID 0x00

#define BATTERY_VOLT_MAX BATTERY_CELL_NUM * 4200
#define BATTERY_VOLT_MIN BATTERY_CELL_NUM * 3300

uint8_t seqNumber = 1;
uint8_t mavlinkMsg[64];
uint8_t mavlinkCrc[2] = { 0x00, 0x00 };

#define MAVLINK_MSG_IDX_HEARTBEAT       0
#define MAVLINK_MSG_IDX_GPS_RAW_INT     1
#define MAVLINK_MSG_IDX_ATTITUDE        2
#define MAVLINK_MSG_IDX_VFR_HUD         3
#ifdef __NAZA_CAN_DECODER_LIB_H__
  #define MAVLINK_MSG_IDX_SYS_STATUS      4
  #define MAVLINK_MSG_IDX_RC_CHANNELS_RAW 5
#endif

// Keeps message length, message ID and magic CRC number for each message
uint8_t mavlinkMsgDef[MAVLINK_MSG_NO][3] = { 
                                               { 9,  0,  50  }  // HEARTBEAT
                                             , { 30, 24, 24  }  // GPS_RAW_INT
                                             , { 28, 30, 39  }  // ATTITUDE
                                             , { 20, 74, 20  }  // VFR_HUD
#ifdef __NAZA_CAN_DECODER_LIB_H__
                                             , { 31, 1,  124 }  // SYS_STATUS
                                             , { 22, 35, 244 }  // RC_CHANNELS_RAW
#endif
                                           };

uint32_t currTime, heartbeatTime, gpsTime, attiTime;
#ifdef __NAZA_CAN_DECODER_LIB_H__
  uint32_t batteryTime, rcChannelsTime;
  
  uint8_t getBatteryPercent()
  {
    return constrain(100.0 * (DECODER.getBattery() - BATTERY_VOLT_MIN) / (BATTERY_VOLT_MAX - BATTERY_VOLT_MIN), 0, 100);
  }
#endif

void encodeByte(uint8_t idx, uint8_t b)
{
  mavlinkMsg[MAVLINK_HEADER_LEN + idx] = b;
}

void encodeShort(uint8_t idx, uint16_t s)
{
  union { uint8_t b[2]; uint16_t s; } val;
  val.s = s;
  for(int i = 0; i < 2; i++) mavlinkMsg[MAVLINK_HEADER_LEN + idx + i] = val.b[i];
}

void encodeLong(uint8_t idx, uint32_t l)
{
  union { uint8_t b[4]; uint32_t l; } val;
  val.l = l;
  for(int i = 0; i < 4; i++) mavlinkMsg[MAVLINK_HEADER_LEN + idx + i] = val.b[i];
}

void encodeFloat(uint8_t idx, float f)
{
  union { uint8_t b[4]; float f; } val;
  val.f = f;
  for(int i = 0; i < 4; i++) mavlinkMsg[MAVLINK_HEADER_LEN + idx + i] = val.b[i];
}

void sendMavlink(uint8_t idx)
{
  // Fill header with data
  mavlinkMsg[0] = mavlinkMsgDef[idx][0];
  mavlinkMsg[1] = seqNumber++;
  mavlinkMsg[2] = MAVLINK_SYSTEM_ID;
  mavlinkMsg[3] = MAVLINK_COMPONENT_ID;
  mavlinkMsg[4] = mavlinkMsgDef[idx][1];

  uint8_t msgLen = mavlinkMsg[0] + MAVLINK_HEADER_LEN;

  // Calculate CRC
  uint16_t crc = 0xFFFF;
  mavlinkMsg[msgLen] = mavlinkMsgDef[idx][2]; // store magic CRC value just after the payload
  for(int i = 0; i < msgLen + 1; i++)         // add 1 to message len to include magic value in CRC calculation
  {
    uint8_t tmp;
    tmp = mavlinkMsg[i] ^ (uint8_t)(crc & 0xff);
    tmp ^= (tmp << 4);
    crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
  }
  mavlinkCrc[0] = (uint8_t)(crc & 0xff);
  mavlinkCrc[1] = (uint8_t)(crc >> 8);

  // Send mavlink messge
  SERIAL_PORT.write(MAVLINK_START_OF_FRAME);
  SERIAL_PORT.write(mavlinkMsg, msgLen);
  SERIAL_PORT.write(mavlinkCrc, MAVLINK_CHECKSUM_LEN);
}

void sendMavlinkHeartbeat()
{
  uint32_t mode = 0;  // 0 = STAB (default when no data from controller)
#ifdef __NAZA_CAN_DECODER_LIB_H__
  switch(DECODER.getMode())
  {
    case DECODER_LIB::MANUAL:    mode = 1; break; // 1 = ACRO
    case DECODER_LIB::ATTI:      mode = 0; break; // 0 = STAB
    case DECODER_LIB::GPS:       mode = 8; break; // 8 = POSI
    case DECODER_LIB::FAILSAFE:  mode = 6; break; // 6 = RETL
  }
#endif
  encodeLong(0, mode); // Extended mode flags (0)
  encodeByte(4, 0x02); // System type (quadrotor)
  encodeByte(5, 0x00); // Autopilot type (generic)
  encodeByte(6, 0xC0); // Basic mode flags (armed + manual input enabled)
  encodeByte(7, 0x04); // State (active)
  encodeByte(8, 0x03); // Protocol version (3 -> 1.0)

  sendMavlink(MAVLINK_MSG_IDX_HEARTBEAT);
}

void sendMavlinkGpsRawInt()
{
  encodeLong(0,   (uint32_t)currTime);
  encodeLong(4,   (uint32_t)0);
  encodeLong(8,   (int32_t)(DECODER.getLat() * 10000000));
  encodeLong(12,  (int32_t)(DECODER.getLon() * 10000000));
  encodeLong(16,  (int32_t)(DECODER.ALTITUDE_FUN() * 1000));
  encodeShort(20, (uint16_t)0xFFFF);
  encodeShort(22, (uint16_t)0xFFFF);
  encodeShort(24, (uint16_t)(DECODER.getSpeed()));
  encodeShort(26, (uint16_t)DECODER.HEADING_FUN());
  encodeByte(28,  DECODER.getFixType());
  encodeByte(29,  DECODER.getNumSat());

  sendMavlink(MAVLINK_MSG_IDX_GPS_RAW_INT);
}

void sendMavlinkAttitude()
{
  encodeLong(0,   (uint32_t)currTime / 1000);
  encodeFloat(4,  (float)(DECODER.getRoll() * M_PI / 180));
  encodeFloat(8,  (float)(DECODER.getPitch() * M_PI / 180));
  encodeFloat(12, (float)0.0);
  encodeFloat(16, (float)0.0);
  encodeFloat(20, (float)0.0);
  encodeFloat(24, (float)0.0);

  sendMavlink(MAVLINK_MSG_IDX_ATTITUDE);
}

void sendMavlinkVfrHud()
{
  encodeFloat(0, 0.0);
  encodeFloat(4,  (float)DECODER.getSpeed());
  encodeFloat(8,  (float)DECODER.ALTITUDE_FUN());
  encodeFloat(12, -(uint16_t)DECODER.VSI_FUN());
  encodeShort(16, (uint16_t)DECODER.HEADING_FUN());
#ifdef __NAZA_CAN_DECODER_LIB_H__
  encodeShort(18, (uint16_t)map(constrain(DECODER.getRcIn(DECODER_LIB::RC_T), -1000, 1000), -1000, 1000, 0, 100));
#else
  encodeShort(18, 0);
#endif

  sendMavlink(MAVLINK_MSG_IDX_VFR_HUD);
}

#ifdef __NAZA_CAN_DECODER_LIB_H__
  void sendMavlinkSysStatus()
  {
    encodeLong(0,   (uint32_t)0); // Sensors present (0)
    encodeLong(4,   (uint32_t)0); // Sensors enabled (0)
    encodeLong(8,   (uint32_t)0); // Sensors health (0)
    encodeShort(12, (uint16_t)0); // Load (0)
    encodeShort(14, (uint16_t)DECODER.getBattery()); // Battery voltage
    encodeShort(16, (int16_t)0);  // Battery current (0)
    encodeShort(18, (uint16_t)0); // Communication drops in percent (0)
    encodeShort(20, (uint16_t)0); // Communication errors (0)
    encodeShort(22, (uint16_t)0); // Autopilot specific errors 1
    encodeShort(24, (uint16_t)0); // Autopilot specific errors 2
    encodeShort(26, (uint16_t)0); // Autopilot specific errors 3
    encodeShort(28, (uint16_t)0); // Autopilot specific errors 4
    encodeByte(30,  (int8_t)getBatteryPercent()); // Remaining battery energy
  
    sendMavlink(MAVLINK_MSG_IDX_SYS_STATUS);
  }

  void sendMavlinkRcChannelsRaw()
  {
    encodeLong(0,   (uint32_t)currTime);
    encodeShort(4,  (uint16_t)map(constrain(DECODER.getRcIn(DECODER_LIB::RC_A), -1000, 1000), -1000, 1000, 1000, 2000));
    encodeShort(6,  (uint16_t)map(constrain(DECODER.getRcIn(DECODER_LIB::RC_E), -1000, 1000), -1000, 1000, 1000, 2000));
    encodeShort(8,  (uint16_t)map(constrain(DECODER.getRcIn(DECODER_LIB::RC_T), -1000, 1000), -1000, 1000, 1000, 2000));
    encodeShort(10, (uint16_t)map(constrain(DECODER.getRcIn(DECODER_LIB::RC_R), -1000, 1000), -1000, 1000, 1000, 2000));
    encodeShort(12, (uint16_t)map(constrain(DECODER.getRcIn(DECODER_LIB::RC_X1), -1000, 1000), -1000, 1000, 1000, 2000));
    encodeShort(14, (uint16_t)map(constrain(DECODER.getRcIn(DECODER_LIB::RC_X2), -1000, 1000), -1000, 1000, 1000, 2000));
    encodeShort(16, (uint16_t)map(constrain(DECODER.getRcIn(DECODER_LIB::RC_U), -1000, 1000), -1000, 1000, 1000, 2000));
    encodeShort(18, (uint16_t)UINT16_MAX);
    encodeByte(20,  (uint8_t)1);  // first and only port
    encodeByte(21,  (uint8_t)255);  // invalid/unknonw
  
    sendMavlink(MAVLINK_MSG_IDX_RC_CHANNELS_RAW);
  }
#endif

void setup()
{
  #ifdef __NAZA_CAN_DECODER_LIB_H__
    DECODER.begin();
  #else
    Serial.begin(115200);
  #endif
  SERIAL_PORT.begin(57600);
}

void loop()
{
  #ifdef __NAZA_CAN_DECODER_LIB_H__
    DECODER.decode();
  #else
    if(Serial.available()) DECODER.decode(Serial.read());
  #endif

  currTime = millis(); 

  if(currTime > heartbeatTime)
  {
    heartbeatTime = currTime + 1000; // 1Hz = every 1000 milliseconds
    sendMavlinkHeartbeat();
  }
  
  if(currTime > gpsTime)
  {
    gpsTime = currTime + 500; // 2Hz = every 500 milliseconds
    sendMavlinkGpsRawInt();
    sendMavlinkVfrHud();
  }

  if(currTime > attiTime)
  {
    attiTime = currTime + 200; // 5Hz = every 200 milliseconds
    sendMavlinkAttitude();
  }

#ifdef __NAZA_CAN_DECODER_LIB_H__
  if(currTime > batteryTime)
  {
    batteryTime = currTime + 1000; // 1Hz = every 1000 milliseconds
    sendMavlinkSysStatus();
  }

  if(currTime > rcChannelsTime)
  {
    rcChannelsTime = currTime + 200; // 5Hz = every 200 milliseconds
    sendMavlinkRcChannelsRaw();
  }

  DECODER.heartbeat();
#endif
}
