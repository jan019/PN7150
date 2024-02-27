/**
 * NXP PN7150 Driver
 * Porting authors:
 *        Salvador Mendoza - @Netxing - salmg.net
 *        Andres Sabas - Electronic Cats - electroniccats.com
 *        Francisco Torres - Electronic Cats - electroniccats.com
 *
 *  August 2023
 *
 * This code is beerware; if you see me (or any other collaborator
 * member) at the local, and you've found our code helpful,
 * please buy us a round!
 * Distributed as-is; no warranty is given.
 *
 * Some methods and ideas were extracted from https://github.com/Strooom/PN7150
 */

#ifndef PN7150_H
#define PN7150_H

#include <Arduino.h>  // Gives us access to all typical Arduino types and functions
                      // The HW interface between The PN7150 and the DeviceHost is I2C, so we need the I2C library.library
#include "Mode.h"
#include "NdefMessage.h"
#include "NdefRecord.h"
#include "P2P_NDEF.h"
#include "RemoteDevice.h"
#include "T4T_NDEF_emu.h"

#include <Wire.h>

#define NO_PN7150_RESET_PIN 255
/* Following definitions specifies which settings will apply when NxpNci_ConfigureSettings()
 * API is called from the application
 */
#define NXP_CORE_CONF 1
#define NXP_CORE_STANDBY 1
#define NXP_CORE_CONF_EXTN 1
#define NXP_CLK_CONF 1   // 1=Xtal, 2=PLL
#define NXP_TVDD_CONF 1  // 1=CFG1, 2=CFG2; CFG1: Vbat is used to generate the VDD(TX) through TXLDO; CFG2: external 5V is used to generate the VDD(TX) through TXLDO
#define NXP_RF_CONF 1

#define NFC_FACTORY_TEST 1

#define NFC_SUCCESS 0
#define NFC_ERROR 1
#define SUCCESS NFC_SUCCESS
#define ERROR NFC_ERROR
constexpr uint16_t MAX_NCI_FRAME_SIZE = 258;

/*
 * Flag definition used for NFC library configuration
 */
#define MODE_CARDEMU (1 << 0)
#define MODE_P2P (1 << 1)
#define MODE_RW (1 << 2)

constexpr uint8_t MaxPayloadSize = 255;  // See NCI specification V1.0, section 3.1
constexpr uint8_t MsgHeaderSize = 3;

/***** Factory Test dedicated APIs *********************************************/
#ifdef NFC_FACTORY_TEST

/*
 * Definition of technology types
 */
typedef enum {
  NFC_A,
  NFC_B,
  NFC_F
} NxpNci_TechType_t;

/*
 * Definition of bitrate
 */
typedef enum {
  BR_106,
  BR_212,
  BR_424,
  BR_848
} NxpNci_Bitrate_t;
#endif

/*
 * Definition of operations handled when processing Reader mode
 */
typedef enum {
#ifndef NO_NDEF_SUPPORT
  READ_NDEF,
  WRITE_NDEF,
#endif
  PRESENCE_CHECK
} RW_Operation_t;

class PN7150 : public Mode {
 private:
  bool _hasBeenInitialized;
  uint8_t _IRQpin, _VENpin, _I2Caddress;
  TwoWire *_wire;
  RfIntf_t dummyRfInterface;
  uint8_t rxBuffer[MaxPayloadSize + MsgHeaderSize];  // buffer where we store bytes received until they form a complete message
  unsigned long timeOut;
  unsigned long timeOutStartTime;
  uint32_t rxMessageLength;  // length of the last message received. As these are not 0x00 terminated, we need to remember the length
  uint8_t gNfcController_fw_version[3] = {0};
  void setTimeOut(unsigned long);  // set a timeOut for an expected next event, eg reception of Response after sending a Command
  bool isTimeOut() const;
  uint8_t wakeupNCI();
  bool getMessage(uint16_t timeout = 5);  // 5 miliseconds as default to wait for interrupt responses

 public:
  PN7150(uint8_t IRQpin, uint8_t VENpin, uint8_t I2Caddress, TwoWire *wire = &Wire);
  uint8_t begin(void);
  uint8_t end(void);
  RemoteDevice remoteDevice;
  Protocol protocol;
  Tech tech;
  ModeTech modeTech;
  Interface interface;
  bool hasMessage() const;
  uint8_t writeData(const uint8_t data[], uint32_t dataLength) const;  // write data from DeviceHost to PN7150. Returns success (0) or Fail (> 0)
  uint32_t readData(uint8_t data[]) const;                       // read data from PN7150, returns the amount of bytes read
  int getFirmwareVersion();
  uint8_t connectNCI();
  uint8_t ConfigMode(uint8_t modeSE);  // Deprecated, use configMode(void) instead
  uint8_t configMode(void);
  bool setReaderWriterMode();
  bool setEmulationMode();
  bool setP2PMode();
  bool configureSettings(void);
  uint8_t startDiscovery();
  uint8_t StartDiscovery(uint8_t modeSE);  // Deprecated, use startDiscovery() instead
  bool stopDiscovery();
  bool WaitForDiscoveryNotification(RfIntf_t *pRfIntf, uint16_t tout = 0);  // Deprecated, use isTagDetected() instead
  bool isTagDetected(uint16_t tout = 500);
  bool cardModeSend(unsigned char *pData, unsigned char DataSize);
  bool cardModeReceive(unsigned char *pData, unsigned char *pDataSize);
  void handleCardEmulation();
  void ProcessCardMode(RfIntf_t RfIntf);                              // Deprecated, use handleCardEmulation() instead
  void processReaderMode(RfIntf_t RfIntf, RW_Operation_t Operation);  // Deprecated, use waitForTagRemoval(), readNdefMessage() or writeNdefMessage() and readNdefMessage() instead
  void processP2pMode(RfIntf_t RfIntf);                               // TODO: rename it
  void presenceCheck(RfIntf_t RfIntf);                                // Deprecated, use waitForTagRemoval() instead
  void waitForTagRemoval();
  bool readerTagCmd(unsigned char *pCommand, unsigned char CommandSize, unsigned char *pAnswer, unsigned char *pAnswerSize);
  bool readerReActivate();
  bool activateNextTagDiscovery();
  bool ReaderActivateNext(RfIntf_t *pRfIntf);        // Deprecated, use activateNextTagDiscovery() instead
  void readNdef(RfIntf_t RfIntf);  // TODO: remove it
  void readNdefMessage();
  void writeNdef(RfIntf_t RfIntf);  // TODO: remove it
  void writeNdefMessage();
  bool nciFactoryTestPrbs(NxpNci_TechType_t type, NxpNci_Bitrate_t bitrate);
  bool nciFactoryTestRfOn();
  bool reset();
  void setReadMsgCallback(std::function<void()> function);
  void setSendMsgCallback(CustomCallback_t function);
  bool isReaderDetected();
  void closeCommunication();
  void sendMessage();
};

#endif
