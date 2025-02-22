#include "PN7150.h"

uint8_t gNextTag_Protocol = PROT_UNDETERMINED;

uint8_t NCIStartDiscovery_length = 0;
const std::size_t maxNCIStartDiscoveryLength = 30;
uint8_t NCIStartDiscovery[maxNCIStartDiscoveryLength];

unsigned char DiscoveryTechnologiesCE[] = { // Emulation
    MODE_LISTEN | MODE_POLL};

unsigned char DiscoveryTechnologiesRW[] = { // Read & Write
    MODE_POLL | TECH_PASSIVE_NFCA,
    MODE_POLL | TECH_PASSIVE_NFCF,
    MODE_POLL | TECH_PASSIVE_NFCB,
    MODE_POLL | TECH_PASSIVE_15693};

unsigned char DiscoveryTechnologiesP2P[] = { // P2P
    MODE_POLL | TECH_PASSIVE_NFCA,
    MODE_POLL | TECH_PASSIVE_NFCB,
    MODE_POLL | TECH_PASSIVE_NFCF,

    MODE_POLL | TECH_ACTIVE_NFCA,
    // MODE_POLL | TECH_ACTIVE_NFCF,

    MODE_LISTEN | TECH_PASSIVE_NFCA,
    MODE_LISTEN | TECH_PASSIVE_NFCB,
    MODE_LISTEN | TECH_PASSIVE_NFCF,

    MODE_LISTEN | TECH_ACTIVE_NFCA,
    MODE_LISTEN | TECH_ACTIVE_NFCF};

const uint8_t NCICoreReset[] = {0x20, 0x00, 0x01, 0x00}; // CORE_RESET_CMD - 0x00: Reset NCI (keep configuration), 0x01: Reset NCI (restore default configuration), JG: command checked, ok.
const uint8_t NCICoreInit[] = {0x20, 0x01, 0x00};        // CORE_INIT_CMD - empty payload; JG: command checked, ok.

PN7150::PN7150(uint8_t IRQpin, uint8_t VENpin,
               uint8_t I2Caddress, TwoWire *wire) : _IRQpin(IRQpin), _VENpin(VENpin), _I2Caddress(I2Caddress), _wire(wire)
{
    pinMode(_IRQpin, INPUT);
    if (_VENpin != 255)
        pinMode(_VENpin, OUTPUT);

    this->_hasBeenInitialized = false;
}

uint8_t PN7150::begin()
{
    _wire->begin();
    if (_VENpin != 255)
    {
        digitalWrite(_VENpin, HIGH);
        delay(1);
        digitalWrite(_VENpin, LOW);
        delay(1);
        digitalWrite(_VENpin, HIGH);
        delay(3);
    }

    if (connectNCI())
    {
        return ERROR;
    }

    if (configureSettings())
    {
        return ERROR;
    }

    if (configMode())
    {
        return ERROR;
    }

    if (startDiscovery())
    {
        return ERROR;
    }

    this->_hasBeenInitialized = true;

    return SUCCESS;
}

uint8_t PN7150::end()
{
    _wire->end();
    digitalWrite(_VENpin, LOW);
    delay(3);

    this->_hasBeenInitialized = false;

    return SUCCESS;
}

bool PN7150::isTimeOut() const
{
    return ((millis() - timeOutStartTime) >= timeOut);
}

void PN7150::setTimeOut(unsigned long theTimeOut)
{
    timeOutStartTime = millis();
    timeOut = theTimeOut;
}

uint8_t PN7150::wakeupNCI()
{ // the device has to wake up using a core reset
    // uint8_t NCICoreReset[] = {0x20, 0x00, 0x01, 0x01}; // TODO: I now defined NCICoreReset at global scope, it is however with 0x00 (only reset, keep config). Why was this with reset config, makes no sense?
    uint16_t NbBytes = 0;

    // Reset RF settings restauration flag
    (void)writeData(NCICoreReset, sizeof(NCICoreReset));
    getMessage(15);
    NbBytes = rxMessageLength;
    if ((NbBytes == 0) || (rxBuffer[0] != 0x40) || (rxBuffer[1] != 0x00))
    {
        return ERROR;
    }
    getMessage();
    NbBytes = rxMessageLength;
    if (NbBytes != 0)
    {
        // NCI_PRINT_BUF("NCI << ", Answer, NbBytes);
        //  Is CORE_GENERIC_ERROR_NTF ?
        if ((rxBuffer[0] == 0x60) && (rxBuffer[1] == 0x07))
        {
            /* Is PN7150B0HN/C11004 Anti-tearing recovery procedure triggered ? */
            // if ((rxBuffer[3] == 0xE6)) gRfSettingsRestored_flag = true;
        }
        else
        {
            return ERROR;
        }
    }
    return SUCCESS;
}

// TODO: the magic number 1337 sets and inifite loop (called in one place), this is not accceptable. Remove it.
// this implemntation must be improved
bool PN7150::getMessage(uint16_t timeout)
{ // check for message using timeout, 5 milisec as default
    setTimeOut(timeout);
    rxMessageLength = 0;
    while (!isTimeOut())
    {
        rxMessageLength = readData(rxBuffer, rxBufferSize);
        if (rxMessageLength == 0xFFFFFFFF) // buffer overflow
        {
            rxMessageLength = 0; // hacky solution for now because library does not check return of getMessage, it works of rxMessageLength value 0 or 1
            return false;
        }

        if (rxMessageLength)
            return true;
        else if (timeout == 1337)
            setTimeOut(timeout);
            // this creates and inifite loop and must be removed
    }

    return rxMessageLength;
}

bool PN7150::hasMessage() const
{
    return (HIGH == digitalRead(_IRQpin)); // PN7150 indicates it has data by driving IRQ signal HIGH
}

// not a safe implementation, but if all occurenes of call are checked and the size is always given as sizeof, thenn it should be fine
uint8_t PN7150::writeData(const uint8_t txBuffer[], uint32_t txBufferLevel) const
{
    uint32_t nmbrBytesWritten = 0;
    _wire->beginTransmission(_I2Caddress);
    nmbrBytesWritten = _wire->write(txBuffer, (size_t)(txBufferLevel));
    delay(10);
#ifdef DEBUG2
    Serial.println("[DEBUG] written bytes = 0x" + String(nmbrBytesWritten, HEX));
#endif
    if (nmbrBytesWritten == txBufferLevel)
    {
        byte resultCode;
        resultCode = _wire->endTransmission();
        delay(10);
#ifdef DEBUG2
        Serial.println("[DEBUG] write data code = 0x" + String(resultCode, HEX));
#endif
        return resultCode;
    }
    else
    {
        return 4; // Could not properly copy data to I2C buffer, so treat as other error, see i2c_t3
    }
}

uint32_t PN7150::readData(uint8_t rxBuffer[], uint16_t bufferSize) const
{
    uint32_t bytesReceived; // keeps track of how many bytes we actually received
    if (hasMessage())
    {                                                                // only try to read something if the PN7150 indicates it has something
        bytesReceived = _wire->requestFrom(_I2Caddress, (uint8_t)3); // first reading the header, as this contains how long the payload will be
        delay(10);
#ifdef DEBUG2
        Serial.println("[DEBUG] bytesReceived = 0x" + String(bytesReceived, HEX));
#endif
        int byte0 = _wire->read();
        int byte1 = _wire->read();
        int byte2 = _wire->read();

        if (byte0 == -1 || byte1 == -1 || byte2 == -1)
        {
            delay(10);
#ifdef DEBUG2
            Serial.println("[DEBUG] Error reading bytes");
            Serial.println("[DEBUG] byte0: 0x" + String(byte0, HEX));
            Serial.println("[DEBUG] byte1: 0x" + String(byte1, HEX));
            Serial.println("[DEBUG] byte2: 0x" + String(byte2, HEX));
#endif
            return 0;
        }

        rxBuffer[0] = byte0;
        rxBuffer[1] = byte1;
        rxBuffer[2] = byte2;
        delay(10);
#ifdef DEBUG2
        for (int i = 0; i < 3; i++)
        {
            Serial.println("[DEBUG] Byte[" + String(i) + "] = 0x" + String(rxBuffer[i], HEX));
        }
#endif
        uint8_t payloadLength = rxBuffer[2];
        if (payloadLength >= bufferSize)
        {
            return 0xFFFFFFFF; // buffer overflow
        }

        if (payloadLength > 0)
        {
            bytesReceived += _wire->requestFrom(_I2Caddress, payloadLength); // then reading the payload, if any
            delay(10);

            if (bytesReceived >= bufferSize)
            {
                return 0xFFFFFFFF; // buffer overflow
            }

#ifdef DEBUG2
            Serial.println("[DEBUG] payload bytes = 0x" + String(bytesReceived - 3, HEX));
#endif
            uint32_t index = 3;
            while (index < bytesReceived)
            {
                int byte = _wire->read();
                if (byte == -1)
                {
#ifdef DEBUG2
                    Serial.println("[DEBUG] Error reading byte");
                    Serial.println("[DEBUG] byte: 0x" + String(byte, HEX));
#endif
                    return 0;
                }
                rxBuffer[index] = byte;
                delay(10);
#ifdef DEBUG2
                Serial.println("[DEBUG] payload[" + String(index) + "] = 0x" + String(rxBuffer[index], HEX));
#endif
                index++;
            }
            index = 0;
        }
    }
    else
    {
        bytesReceived = 0;
    }
    return bytesReceived;
}

int PN7150::getFirmwareVersion()
{
    return ((gNfcController_fw_version[0] & 0xFF) << 16) | ((gNfcController_fw_version[1] & 0xFF) << 8) | (gNfcController_fw_version[2] & 0xFF);
}

uint8_t PN7150::connectNCI()
{
    uint8_t i = 2;
    uint8_t NCICoreInit[] = {0x20, 0x01, 0x00};

    // Check if begin function has been called
    if (this->_hasBeenInitialized)
    {
        return SUCCESS;
    }

    // Open connection to NXPNCI
    _wire->begin();
    if (_VENpin != 255)
    {
        digitalWrite(_VENpin, HIGH);
        delay(1);
        digitalWrite(_VENpin, LOW);
        delay(1);
        digitalWrite(_VENpin, HIGH);
        delay(3);
    }

    // Loop until NXPNCI answers
    while (wakeupNCI() != SUCCESS)
    {
        if (i-- == 0)
            return ERROR;
        delay(500);
    }

    (void)writeData(NCICoreInit, sizeof(NCICoreInit));
    getMessage();
    if ((rxBuffer[0] != 0x40) || (rxBuffer[1] != 0x01) || (rxBuffer[3] != 0x00))
        return ERROR;

    // Retrieve NXP-NCI NFC Controller generation
    std::size_t romCodePos = 17 + rxBuffer[8];
    std::size_t majorFwVersion = 18 + rxBuffer[8];
    std::size_t minorFwversion = 19 + rxBuffer[8];
    if (minorFwversion >= rxBufferSize)
        return ERROR; // buffer overflow

    if (rxBuffer[romCodePos] != 0x10)
        return ERROR; // Not supported

    // Retrieve NXP-NCI NFC Controller FW version
    gNfcController_fw_version[0] = rxBuffer[romCodePos]; // 0xROM_CODE_V
    gNfcController_fw_version[1] = rxBuffer[majorFwVersion]; // 0xFW_MAJOR_NO
    gNfcController_fw_version[2] = rxBuffer[minorFwversion]; // 0xFW_MINOR_NO
#ifdef DEBUG
    Serial.println("0xROM_CODE_V: " + String(gNfcController_fw_version[0], HEX));
    Serial.println("FW_MAJOR_NO: " + String(gNfcController_fw_version[1], HEX));
    Serial.println("0xFW_MINOR_NO: " + String(gNfcController_fw_version[2], HEX));
#endif

    return SUCCESS;
}

/// @brief Update the internal mode, stop discovery, and build the command to configure the PN7150 chip based on the input mode
/// @param modeSE
/// @return SUCCESS or ERROR
uint8_t PN7150::ConfigMode(uint8_t modeSE)
{
    unsigned mode = (modeSE == 1 ? MODE_RW : modeSE == 2 ? MODE_CARDEMU
                                                         : MODE_P2P);

    // Update internal mode
    if (!PN7150::setMode(modeSE))
    {
        return ERROR; // Invalid mode, out of range
    }

    PN7150::stopDiscovery();

    uint8_t Command[MAX_NCI_FRAME_SIZE];

    uint8_t Item = 0;
    uint8_t NCIDiscoverMap[] = {0x21, 0x00};

    // Emulation mode
    const uint8_t DM_CARDEMU[] = {0x4, 0x2, 0x2};
    const uint8_t R_CARDEMU[] = {0x1, 0x3, 0x0, 0x1, 0x4};

    // RW Mode
    const uint8_t DM_RW[] = {0x1, 0x1, 0x1, 0x2, 0x1, 0x1, 0x3, 0x1, 0x1, 0x4, 0x1, 0x2, 0x80, 0x01, 0x80};
    uint8_t NCIPropAct[] = {0x2F, 0x02, 0x00};

    // P2P Support
    const uint8_t DM_P2P[] = {0x5, 0x3, 0x3};
    const uint8_t R_P2P[] = {0x1, 0x3, 0x0, 0x1, 0x5};
    uint8_t NCISetConfig_NFC[] = {0x20, 0x02, 0x1F, 0x02, 0x29, 0x0D, 0x46, 0x66, 0x6D, 0x01, 0x01, 0x11, 0x03, 0x02, 0x00, 0x01, 0x04, 0x01, 0xFA, 0x61, 0x0D, 0x46, 0x66, 0x6D, 0x01, 0x01, 0x11, 0x03, 0x02, 0x00, 0x01, 0x04, 0x01, 0xFA};

    uint8_t NCIRouting[] = {0x21, 0x01, 0x07, 0x00, 0x01};
    uint8_t NCISetConfig_NFCA_SELRSP[] = {0x20, 0x02, 0x04, 0x01, 0x32, 0x01, 0x00};

    if (mode == 0)
        return SUCCESS;

    /* Enable Proprietary interface for T4T card presence check procedure */
    if (modeSE == 1)
    {
        if (mode == MODE_RW)
        {
            (void)writeData(NCIPropAct, sizeof(NCIPropAct));
            getMessage(10);

            if ((rxBuffer[0] != 0x4F) || (rxBuffer[1] != 0x02) || (rxBuffer[3] != 0x00))
                return ERROR;
        }
    }

    //* Building Discovery Map command
    Item = 0;

    if ((mode & MODE_CARDEMU and modeSE == 2) || (mode & MODE_P2P and modeSE == 3))
    {
        std::size_t copyStartPos = 4 + (3 * Item);
        if ((copyStartPos + sizeof((modeSE == 2 ? DM_CARDEMU : DM_P2P))) >= MAX_NCI_FRAME_SIZE)
        {
            return ERROR; // buffer overflow
        }
        memcpy(&Command[copyStartPos], (modeSE == 2 ? DM_CARDEMU : DM_P2P), sizeof((modeSE == 2 ? DM_CARDEMU : DM_P2P)));
        Item++;
    }
    if (mode & MODE_RW and modeSE == 1)
    {
        std::size_t copyStartPos = 4 + (3 * Item);
        if ((copyStartPos + sizeof(DM_RW)) >= MAX_NCI_FRAME_SIZE)
        {
            return ERROR; // buffer overflow
        }
        memcpy(&Command[copyStartPos], DM_RW, sizeof(DM_RW));
        Item += sizeof(DM_RW) / 3;
    }
    if (Item != 0)
    {
        if (sizeof(NCIDiscoverMap) >= MAX_NCI_FRAME_SIZE)
        {
            return ERROR; // buffer overflow
        }

        memcpy(Command, NCIDiscoverMap, sizeof(NCIDiscoverMap));
        Command[2] = 1 + (Item * 3);
        Command[3] = Item;

        std::size_t writeSize = 3 + Command[2];
        if ((writeSize) >= MAX_NCI_FRAME_SIZE)
        {
            return ERROR; // buffer overflow
        }

        (void)writeData(Command, writeSize);
        getMessage(10);
        if ((rxBuffer[0] != 0x41) || (rxBuffer[1] != 0x00) || (rxBuffer[3] != 0x00))
        {
            return ERROR;
        }
    }

    // Configuring routing
    Item = 0;

    if (modeSE == 2 || modeSE == 3)
    { // Emulation or P2P
        std::size_t copyStartPos = 5 + (5 * Item);
        if ((copyStartPos + sizeof((modeSE == 2 ? R_CARDEMU : R_P2P))) >= MAX_NCI_FRAME_SIZE)
        {
            return ERROR; // buffer overflow
        }

        memcpy(&Command[copyStartPos], (modeSE == 2 ? R_CARDEMU : R_P2P), sizeof((modeSE == 2 ? R_CARDEMU : R_P2P)));
        Item++;

        if (Item != 0)
        {
            if (sizeof(NCIRouting) >= MAX_NCI_FRAME_SIZE)
            {
                return ERROR; // buffer overflow
            }

            memcpy(Command, NCIRouting, sizeof(NCIRouting));
            Command[2] = 2 + (Item * 5);
            Command[4] = Item;

            std::size_t writeSize = 3 + Command[2];
            if ((writeSize) >= MAX_NCI_FRAME_SIZE)
            {
                return ERROR; // buffer overflow
            }

            (void)writeData(Command, writeSize);
            getMessage(10);
            if ((rxBuffer[0] != 0x41) || (rxBuffer[1] != 0x01) || (rxBuffer[3] != 0x00))
                return ERROR;
        }
        NCISetConfig_NFCA_SELRSP[6] += (modeSE == 2 ? 0x20 : 0x40);

        if (NCISetConfig_NFCA_SELRSP[6] != 0x00)
        {
            (void)writeData(NCISetConfig_NFCA_SELRSP, sizeof(NCISetConfig_NFCA_SELRSP));
            getMessage(10);

            if ((rxBuffer[0] != 0x40) || (rxBuffer[1] != 0x02) || (rxBuffer[3] != 0x00))
                return ERROR;
            else
                return SUCCESS;
        }

        if (mode & MODE_P2P and modeSE == 3)
        {
            (void)writeData(NCISetConfig_NFC, sizeof(NCISetConfig_NFC));
            getMessage(10);

            if ((rxBuffer[0] != 0x40) || (rxBuffer[1] != 0x02) || (rxBuffer[3] != 0x00))
                return ERROR;
        }
    }
    return SUCCESS;
}

uint8_t PN7150::configMode()
{
    int mode = PN7150::getMode();
    return PN7150::ConfigMode(mode);
}

bool PN7150::configureSettings(void)
{
#if NXP_CORE_CONF
    /* NCI standard dedicated settings
     * Refer to NFC Forum NCI standard for more details
     */
    uint8_t NxpNci_CORE_CONF[] = {
        0x20, 0x02, 0x05, 0x01, /* CORE_SET_CONFIG_CMD */
        0x00, 0x02, 0x00, 0x01  /* TOTAL_DURATION */
        // JG: command checked, ok. TOTAL_DURATION is set to 256 ms, which is the RF discovery loop length (actualy for PN7150 its the listening phase duration, the poll phase is about 20 ms)
    };
#endif

#if NXP_CORE_CONF_EXTN
    /* NXP-NCI extension dedicated setting
     * Refer to NFC controller User Manual for more details
     */
    // this is the configuration for LPCD (low power card detection). For now we don't use it, instead we completely turn off the chip.
    // however, it would speed up the process of reading a chip dramatically, and so it might be worth to use it in the future.
    // TODO: Decide after testing
    uint8_t NxpNci_CORE_CONF_EXTN[] = {
        0x20, 0x02, 0x0D, 0x03, /* CORE_SET_CONFIG_CMD */
        0xA0, 0x40, 0x01, 0x00, /* TAG_DETECTOR_CFG - default value, agc detection disabled, trace mode disabled*/
        0xA0, 0x41, 0x01, 0x04, /* TAG_DETECTOR_THRESHOLD_CFG - default, value, detection level, not sure what it does*/
        0xA0, 0x43, 0x01, 0x00  /* TAG_DETECTOR_FALLBACK_CNT_CFG - not default, param used to configure Hybrid mode to insert a regular polling cycle ever N pulses generated by the LPCD. Disabled.*/
        // JG: command checked, ok.
    };
#endif

#if NXP_CORE_STANDBY
    /* NXP-NCI standby enable setting
     * Refer to NFC controller User Manual for more details
     */
    uint8_t NxpNci_CORE_STANDBY[] = {
        0x2F, 0x00, 0x01, 0x01 /* CORE_SET_POWER_MODE_CMD - 0x01 standy mode enabled - drastically reduces current consumption in listening mode*/
        // JG: command checked, ok.
    };
#endif

#if NXP_TVDD_CONF
    /* NXP-NCI TVDD configuration
     * Refer to NFC controller Hardware Design Guide document for more details
     */
    /* RF configuration related to 2nd generation of NXP-NCI controller (e.g PN7150)*/
#if (NXP_TVDD_CONF == 1)
    /* PMU_CFG - Configuration of the Power Management Unit (PMU)*/
    /* CFG1: Vbat is used to generate the VDD(TX) through TXLDO */
    uint8_t NxpNci_TVDD_CONF[] = {0x20, 0x02, 0x07, 0x01, 0xA0, 0x0E, 0x03, 0x02, 0x09, 0x00};
#else
    /* CFG2: external 5V is used to generate the VDD(TX) through TXLDO */
    uint8_t NxpNci_TVDD_CONF[] = {0x20, 0x02, 0x07, 0x01, 0xA0, 0x0E, 0x03, 0x06, 0x64, 0x00}; /* PMU_CFG */
#endif
    // JG: command checked, ok.
#endif

#if NXP_RF_CONF
    /* NXP-NCI RF configuration
     * Refer to NFC controller Antenna Design and Tuning Guidelines document for more details
     */
    /* RF configuration related to 2nd generation of NXP-NCI controller (e.g PN7150)*/
    /* Following configuration relates to performance optimization of OM5578/PN7150 NFC Controller demo kit */
    /* Avoid mixing RF Transition parameters with other parameters (not starting with address 0xA00D) in a same CORE_SET_CONFIG_CMD command */
    uint8_t NxpNci_RF_CONF[] = {
        0x20, 0x02, 0x94, 0x11,
        0xA0, 0x0D, 0x06, 0x04, 0x35, 0x90, 0x01, 0xF4, 0x01, /* RF_CLIF_CFG_INITIATOR        CLIF_AGC_INPUT_REG */
        0xA0, 0x0D, 0x06, 0x06, 0x30, 0x01, 0x90, 0x03, 0x00, /* RF_CLIF_CFG_TARGET           CLIF_SIGPRO_ADCBCM_THRESHOLD_REG */
        0xA0, 0x0D, 0x06, 0x06, 0x42, 0x02, 0x00, 0xFF, 0xFF, /* RF_CLIF_CFG_TARGET           CLIF_ANA_TX_AMPLITUDE_REG */
        0xA0, 0x0D, 0x06, 0x20, 0x42, 0x88, 0x00, 0xFF, 0xFF, /* RF_CLIF_CFG_TECHNO_I_TX15693 CLIF_ANA_TX_AMPLITUDE_REG */
        0xA0, 0x0D, 0x04, 0x22, 0x44, 0x23, 0x00,             /* RF_CLIF_CFG_TECHNO_I_RX15693 CLIF_ANA_RX_REG */
        0xA0, 0x0D, 0x06, 0x22, 0x2D, 0x50, 0x34, 0x0C, 0x00, /* RF_CLIF_CFG_TECHNO_I_RX15693 CLIF_SIGPRO_RM_CONFIG1_REG */
        0xA0, 0x0D, 0x06, 0x32, 0x42, 0xF8, 0x00, 0xFF, 0xFF, /* RF_CLIF_CFG_BR_106_I_TXA     CLIF_ANA_TX_AMPLITUDE_REG */
        0xA0, 0x0D, 0x06, 0x34, 0x2D, 0x24, 0x37, 0x0C, 0x00, /* RF_CLIF_CFG_BR_106_I_RXA_P   CLIF_SIGPRO_RM_CONFIG1_REG */
        0xA0, 0x0D, 0x06, 0x34, 0x33, 0x86, 0x80, 0x00, 0x70, /* RF_CLIF_CFG_BR_106_I_RXA_P   CLIF_AGC_CONFIG0_REG */
        0xA0, 0x0D, 0x04, 0x34, 0x44, 0x22, 0x00,             /* RF_CLIF_CFG_BR_106_I_RXA_P   CLIF_ANA_RX_REG */
        0xA0, 0x0D, 0x06, 0x42, 0x2D, 0x15, 0x45, 0x0D, 0x00, /* RF_CLIF_CFG_BR_848_I_RXA     CLIF_SIGPRO_RM_CONFIG1_REG */
        0xA0, 0x0D, 0x04, 0x46, 0x44, 0x22, 0x00,             /* RF_CLIF_CFG_BR_106_I_RXB     CLIF_ANA_RX_REG */
        0xA0, 0x0D, 0x06, 0x46, 0x2D, 0x05, 0x59, 0x0E, 0x00, /* RF_CLIF_CFG_BR_106_I_RXB     CLIF_SIGPRO_RM_CONFIG1_REG */
        0xA0, 0x0D, 0x06, 0x44, 0x42, 0x88, 0x00, 0xFF, 0xFF, /* RF_CLIF_CFG_BR_106_I_TXB     CLIF_ANA_TX_AMPLITUDE_REG */
        0xA0, 0x0D, 0x06, 0x56, 0x2D, 0x05, 0x9F, 0x0C, 0x00, /* RF_CLIF_CFG_BR_212_I_RXF_P   CLIF_SIGPRO_RM_CONFIG1_REG */
        0xA0, 0x0D, 0x06, 0x54, 0x42, 0x88, 0x00, 0xFF, 0xFF, /* RF_CLIF_CFG_BR_212_I_TXF     CLIF_ANA_TX_AMPLITUDE_REG */
        0xA0, 0x0D, 0x06, 0x0A, 0x33, 0x80, 0x86, 0x00, 0x70  /* RF_CLIF_CFG_I_ACTIVE         CLIF_AGC_CONFIG0_REG */
    };
    // JG: The settings of these parameters need to be changed according to measurements. This will be done for the final release, not the testing
#endif

#if NXP_CLK_CONF
    /* NXP-NCI CLOCK configuration
     * Refer to NFC controller Hardware Design Guide document for more details
     */
#if (NXP_CLK_CONF == 1)
    /* Xtal configuration */
    uint8_t NxpNci_CLK_CONF[] = {
        0x20, 0x02, 0x05, 0x01, /* CORE_SET_CONFIG_CMD */
        0xA0, 0x03, 0x01, 0x08  /* CLOCK_SEL_CFG */
        // JG: command checked, ok.
    };

#else
    /* PLL configuration */
    uint8_t NxpNci_CLK_CONF[] = {
        0x20, 0x02, 0x09, 0x02, /* CORE_SET_CONFIG_CMD */
        0xA0, 0x03, 0x01, 0x11, /* CLOCK_SEL_CFG */
        0xA0, 0x04, 0x01, 0x01  /* CLOCK_TO_CFG */
    };
#endif
#endif

    bool gRfSettingsRestored_flag = false;

#if (NXP_TVDD_CONF | NXP_RF_CONF)
    uint8_t *NxpNci_CONF;
    uint16_t NxpNci_CONF_size = 0;
#endif
#if (NXP_CORE_CONF_EXTN | NXP_CLK_CONF | NXP_TVDD_CONF | NXP_RF_CONF)
    const std::size_t timestampSize = 32;
    const std::size_t headerSize = 7;
    uint8_t currentTS[timestampSize] = __TIMESTAMP__;
    uint8_t NCIReadTS[] = {0x20, 0x03, 0x03, 0x01, 0xA0, 0x14};              // CORE_GET_CONFIG_CMD - (NXP extended) read EEPROM; DH_EEPROM_AREA_2 - 32-Byte EEPROM area dedicated to the DH to store/retrieve non-volatile data. JG: command checked, ok.
    uint8_t NCIWriteTS[headerSize + timestampSize] = {0x20, 0x02, 0x24, 0x01, 0xA0, 0x14, 0x20}; // CORE_SET_CONFIG_CMD - (NXP extended) write 32 bytes (0x20) to EEPROM; DH_EEPROM_AREA_2
                                                                             // 7 + 32 because 7 is the length of the command and 32 is the length of the timestamp
                                                                             // JG: both command checked, ok.
#endif
    bool isResetRequired = false;

    /* Apply settings */
#if NXP_CORE_CONF
    if (sizeof(NxpNci_CORE_CONF) != 0)
    {
        isResetRequired = true;
        (void)writeData(NxpNci_CORE_CONF, sizeof(NxpNci_CORE_CONF));
        getMessage(10);
        if ((rxBuffer[0] != 0x40) || (rxBuffer[1] != 0x02) || (rxBuffer[3] != 0x00) || (rxBuffer[4] != 0x00))
        {
#ifdef DEBUG
            Serial.println("NxpNci_CORE_CONF");
#endif
            return ERROR;
        }
    }
#endif

#if NXP_CORE_STANDBY
    if (sizeof(NxpNci_CORE_STANDBY) != 0)
    {
        (void)(writeData(NxpNci_CORE_STANDBY, sizeof(NxpNci_CORE_STANDBY)));
        getMessage(10);
        if ((rxBuffer[0] != 0x4F) || (rxBuffer[1] != 0x00) || (rxBuffer[3] != 0x00))
        {
#ifdef DEBUG
            Serial.println("NxpNci_CORE_STANDBY");
#endif
            return ERROR;
        }
    }
#endif

    /* All further settings are not versatile, so configuration only applied if there are changes (application build timestamp)
       or in case of PN7150B0HN/C11004 Anti-tearing recovery procedure inducing RF setings were restored to their default value */
#if (NXP_CORE_CONF_EXTN | NXP_CLK_CONF | NXP_TVDD_CONF | NXP_RF_CONF)
    /* First read timestamp stored in NFC Controller */
    (void)writeData(NCIReadTS, sizeof(NCIReadTS));
    getMessage(10);
    if ((rxBuffer[0] != 0x40) || (rxBuffer[1] != 0x03) || (rxBuffer[3] != 0x00))
    {
#ifdef DEBUG
        Serial.println("read timestamp ");
#endif
        return ERROR;
    }
    /* Then compare with current build timestamp, and check RF setting restauration flag */
    // TODO: This was commented out, why?
    if (!memcmp(&rxBuffer[8], currentTS, sizeof(currentTS)) && (gRfSettingsRestored_flag == false))
    {
        // No change, nothing to do
    }
    else
    {

        /* Apply settings */
#if NXP_CORE_CONF_EXTN
        if (sizeof(NxpNci_CORE_CONF_EXTN) != 0)
        {
            (void)writeData(NxpNci_CORE_CONF_EXTN, sizeof(NxpNci_CORE_CONF_EXTN));
            getMessage(10);
            if ((rxBuffer[0] != 0x40) || (rxBuffer[1] != 0x02) || (rxBuffer[3] != 0x00) || (rxBuffer[4] != 0x00))
            {
#ifdef DEBUG
                Serial.println("NxpNci_CORE_CONF_EXTN");
#endif
                return ERROR;
            }
        }
#endif

#if NXP_CLK_CONF
        if (sizeof(NxpNci_CLK_CONF) != 0)
        {
            isResetRequired = true;

            (void)writeData(NxpNci_CLK_CONF, sizeof(NxpNci_CLK_CONF));
            getMessage(10);
            if ((rxBuffer[0] != 0x40) || (rxBuffer[1] != 0x02) || (rxBuffer[3] != 0x00) || (rxBuffer[4] != 0x00))
            {
#ifdef DEBUG
                Serial.println("NxpNci_CLK_CONF");
#endif
                return ERROR;
            }
        }
#endif

#if NXP_TVDD_CONF
        if (NxpNci_CONF_size != 0)
        {
            (void)writeData(NxpNci_TVDD_CONF, sizeof(NxpNci_TVDD_CONF));
            getMessage(10);
            if ((rxBuffer[0] != 0x40) || (rxBuffer[1] != 0x02) || (rxBuffer[3] != 0x00) || (rxBuffer[4] != 0x00))
            {
#ifdef DEBUG
                Serial.println("NxpNci_CONF_size");
#endif
                return ERROR;
            }
        }
#endif

#if NXP_RF_CONF
        if (NxpNci_CONF_size != 0)
        {
            (void)writeData(NxpNci_RF_CONF, sizeof(NxpNci_RF_CONF));
            getMessage(10);
            if ((rxBuffer[0] != 0x40) || (rxBuffer[1] != 0x02) || (rxBuffer[3] != 0x00) || (rxBuffer[4] != 0x00))
            {
#ifdef DEBUG
                Serial.println("NxpNci_CONF_size");
#endif
                return ERROR;
            }
        }
#endif
        /* Store curent timestamp to NFC Controller memory for further checks */
        memcpy(&NCIWriteTS[headerSize], currentTS, sizeof(currentTS));
        (void)writeData(NCIWriteTS, sizeof(NCIWriteTS));
        getMessage(10);
        if ((rxBuffer[0] != 0x40) || (rxBuffer[1] != 0x02) || (rxBuffer[3] != 0x00) || (rxBuffer[4] != 0x00))
        {
#ifdef DEBUG
            Serial.println("NFC Controller memory");
#endif
            return ERROR;
        }
    }
#endif

    if (isResetRequired)
    {
        /* Reset the NFC Controller to insure new settings apply */
        (void)writeData(NCICoreReset, sizeof(NCICoreReset));
        getMessage();
        if ((rxBuffer[0] != 0x40) || (rxBuffer[1] != 0x00) || (rxBuffer[3] != 0x00))
        {
#ifdef DEBUG
            Serial.println("insure new settings apply");
#endif
            return ERROR;
        }

        (void)writeData(NCICoreInit, sizeof(NCICoreInit));
        getMessage();
        if ((rxBuffer[0] != 0x40) || (rxBuffer[1] != 0x01) || (rxBuffer[3] != 0x00))
        {
#ifdef DEBUG
            Serial.println("insure new settings apply 2");
#endif
            return ERROR;
        }
    }
    return SUCCESS;
}

uint8_t PN7150::StartDiscovery(uint8_t modeSE)
{
    int mode = PN7150::getMode();
    if (mode != modeSE)
    {
        PN7150::setMode(modeSE);
        PN7150::configMode();
    }

    unsigned char TechTabSize = (modeSE == 1 ? sizeof(DiscoveryTechnologiesRW) : modeSE == 2 ? sizeof(DiscoveryTechnologiesCE)
                                                                                             : sizeof(DiscoveryTechnologiesP2P));

    NCIStartDiscovery_length = 0;
    NCIStartDiscovery[0] = 0x21;
    NCIStartDiscovery[1] = 0x03;
    NCIStartDiscovery[2] = (TechTabSize * 2) + 1;
    NCIStartDiscovery[3] = TechTabSize;
    for (uint8_t i = 0; i < TechTabSize; i++)
    {
        NCIStartDiscovery[(i * 2) + 4] = (modeSE == 1 ? DiscoveryTechnologiesRW[i] : modeSE == 2 ? DiscoveryTechnologiesCE[i]
                                                                                                 : DiscoveryTechnologiesP2P[i]);

        NCIStartDiscovery[(i * 2) + 5] = 0x01;
    }

    NCIStartDiscovery_length = (TechTabSize * 2) + 4;
    if (NCIStartDiscovery_length >= maxNCIStartDiscoveryLength)
    {
        return ERROR; // buffer overflow
    }

    (void)writeData(NCIStartDiscovery, NCIStartDiscovery_length);
    getMessage();

    if ((rxBuffer[0] != 0x41) || (rxBuffer[1] != 0x03) || (rxBuffer[3] != 0x00))
        return ERROR;
    else
        return SUCCESS;
}

uint8_t PN7150::startDiscovery()
{
    int mode = PN7150::getMode();
    return PN7150::StartDiscovery(mode);
}

bool PN7150::stopDiscovery()
{
    uint8_t NCIStopDiscovery[] = {0x21, 0x06, 0x01, 0x00};

    (void)writeData(NCIStopDiscovery, sizeof(NCIStopDiscovery));
    getMessage(10);

    return SUCCESS;
}

bool PN7150::WaitForDiscoveryNotification(RfIntf_t *pRfIntf, uint16_t tout)
{
    uint8_t NCIRfDiscoverSelect[] = {0x21, 0x04, 0x03, 0x01, protocol.ISODEP, interface.ISODEP};

    // P2P Support
    uint8_t NCIStopDiscovery[] = {0x21, 0x06, 0x01, 0x00};
    uint8_t NCIRestartDiscovery[] = {0x21, 0x06, 0x01, 0x03};
    uint8_t saved_NTF[7];

    gNextTag_Protocol = PROT_UNDETERMINED;
    bool getFlag = false;
wait:
    do
    {
        getFlag = getMessage(
            tout > 0 ? tout : 1337); // Infinite loop, waiting for response
    } while (((rxBuffer[0] != 0x61) || ((rxBuffer[1] != 0x05) && (rxBuffer[1] != 0x03))) && (getFlag == true));
    gNextTag_Protocol = PROT_UNDETERMINED;

    /* Is RF_INTF_ACTIVATED_NTF ? */
    if (rxBuffer[1] == 0x05)
    {
        pRfIntf->Interface = rxBuffer[4];
        remoteDevice.setInterface(rxBuffer[4]);
        pRfIntf->Protocol = rxBuffer[5];
        remoteDevice.setProtocol(rxBuffer[5]);
        pRfIntf->ModeTech = rxBuffer[6];
        remoteDevice.setModeTech(rxBuffer[6]);
        pRfIntf->MoreTags = false;
        remoteDevice.setMoreTagsAvailable(false);
        remoteDevice.setInfo(pRfIntf, &rxBuffer[10]);

        // P2P
        /* Verifying if not a P2P device also presenting T4T emulation */
        if ((pRfIntf->Interface == INTF_ISODEP) && (pRfIntf->Protocol == PROT_ISODEP) && ((pRfIntf->ModeTech & MODE_LISTEN) != MODE_LISTEN))
        {
            memcpy(saved_NTF, rxBuffer, sizeof(saved_NTF));
            while (1)
            {
                /* Restart the discovery loop */
                (void)writeData(NCIRestartDiscovery, sizeof(NCIRestartDiscovery));
                getMessage();
                getMessage(100);
                /* Wait for discovery */
                do
                {
                    getMessage(1000); // Infinite loop, waiting for response
                } while ((rxMessageLength == 4) && (rxBuffer[0] == 0x60) && (rxBuffer[1] == 0x07));

                if ((rxMessageLength != 0) && (rxBuffer[0] == 0x61) && (rxBuffer[1] == 0x05))
                {
                    /* Is same device detected ? */
                    if (memcmp(saved_NTF, rxBuffer, sizeof(saved_NTF)) == 0)
                        break;
                    /* Is P2P detected ? */
                    if (rxBuffer[5] == PROT_NFCDEP)
                    {
                        pRfIntf->Interface = rxBuffer[4];
                        remoteDevice.setInterface(rxBuffer[4]);
                        pRfIntf->Protocol = rxBuffer[5];
                        remoteDevice.setProtocol(rxBuffer[5]);
                        pRfIntf->ModeTech = rxBuffer[6];
                        remoteDevice.setModeTech(rxBuffer[6]);
                        pRfIntf->MoreTags = false;
                        remoteDevice.setMoreTagsAvailable(false);
                        remoteDevice.setInfo(pRfIntf, &rxBuffer[10]);
                        break;
                    }
                }
                else
                {
                    if (rxMessageLength != 0)
                    {
                        /* Flush any other notification  */
                        while (rxMessageLength != 0)
                            getMessage(100);

                        /* Restart the discovery loop */
                        (void)writeData(NCIRestartDiscovery, sizeof(NCIRestartDiscovery));
                        getMessage();
                        getMessage(100);
                    }
                    goto wait;
                }
            }
        }
    }
    else
    { /* RF_DISCOVER_NTF */
        pRfIntf->Interface = INTF_UNDETERMINED;
        remoteDevice.setInterface(interface.UNDETERMINED);
        pRfIntf->Protocol = rxBuffer[4];
        remoteDevice.setProtocol(rxBuffer[4]);
        pRfIntf->ModeTech = rxBuffer[5];
        remoteDevice.setModeTech(rxBuffer[5]);
        pRfIntf->MoreTags = true;
        remoteDevice.setMoreTagsAvailable(true);

        /* Get next NTF for further activation */
        do
        {
            if (!getMessage(100))
                return ERROR;
        } while ((rxBuffer[0] != 0x61) || (rxBuffer[1] != 0x03));
        gNextTag_Protocol = rxBuffer[4];

        /* Remaining NTF ? */
        if (rxMessageLength >= rxBufferSize)
        {
            return ERROR;
        }

        while (rxBuffer[rxMessageLength - 1] == 0x02)
            getMessage(100);

        /* In case of multiple cards, select the first one */
        NCIRfDiscoverSelect[4] = remoteDevice.getProtocol();
        if (remoteDevice.getProtocol() == protocol.ISODEP)
            NCIRfDiscoverSelect[5] = interface.ISODEP;
        else if (remoteDevice.getProtocol() == protocol.NFCDEP)
            NCIRfDiscoverSelect[5] = interface.NFCDEP;
        else if (remoteDevice.getProtocol() == protocol.MIFARE)
            NCIRfDiscoverSelect[5] = interface.TAGCMD;
        else
            NCIRfDiscoverSelect[5] = interface.FRAME;

        (void)writeData(NCIRfDiscoverSelect, sizeof(NCIRfDiscoverSelect));
        getMessage(100);

        if ((rxBuffer[0] == 0x41) || (rxBuffer[1] == 0x04) || (rxBuffer[3] == 0x00))
        {
            if (rxMessageLength >= rxBufferSize)
            {
                return ERROR; // buffer overflow
            }

            (void)writeData(rxBuffer, rxMessageLength);
            getMessage(100);

            if ((rxBuffer[0] == 0x61) || (rxBuffer[1] == 0x05))
            {
                pRfIntf->Interface = rxBuffer[4];
                remoteDevice.setInterface(rxBuffer[4]);
                pRfIntf->Protocol = rxBuffer[5];
                remoteDevice.setProtocol(rxBuffer[5]);
                pRfIntf->ModeTech = rxBuffer[6];
                remoteDevice.setModeTech(rxBuffer[6]);
                remoteDevice.setInfo(pRfIntf, &rxBuffer[10]);
            }

            /* In case of P2P target detected but lost, inform application to restart discovery */
            else if (remoteDevice.getProtocol() == protocol.NFCDEP)
            {
                /* Restart the discovery loop */
                (void)writeData(NCIStopDiscovery, sizeof(NCIStopDiscovery));
                getMessage();
                getMessage(100);

                if (NCIStartDiscovery_length >= maxNCIStartDiscoveryLength)
                {
                    return ERROR; // buffer overflow
                }

                (void)writeData(NCIStartDiscovery, NCIStartDiscovery_length);
                getMessage();

                goto wait;
            }
        }
    }

    /* In case of unknown target align protocol information */
    if (remoteDevice.getInterface() == interface.UNDETERMINED)
    {
        pRfIntf->Protocol = PROT_UNDETERMINED;
        remoteDevice.setProtocol(protocol.UNDETERMINED);
    }

    return SUCCESS;
}

bool PN7150::isTagDetected(uint16_t tout)
{
    return !PN7150::WaitForDiscoveryNotification(&this->dummyRfInterface, tout);
}

bool PN7150::cardModeSend(unsigned char *pData, unsigned char DataSize)
{
    bool status;
    uint8_t Cmd[MAX_NCI_FRAME_SIZE];

    /* Compute and send DATA_PACKET */
    Cmd[0] = 0x00;
    Cmd[1] = 0x00;
    Cmd[2] = DataSize;

    if (DataSize >= MAX_NCI_FRAME_SIZE - 3) // -3 because the first 3 bytes are used for the header
    {
        return false; // buffer overflow
    }
    memcpy(&Cmd[3], pData, DataSize);
    (void)writeData(Cmd, DataSize + 3);
    return status;
}

bool PN7150::cardModeReceive(unsigned char *pData, unsigned char *pDataSize)
{
#ifdef DEBUG2
    Serial.println("[DEBUG] cardModeReceive exec");
#endif

    delay(1);

    bool status = NFC_ERROR;
    uint8_t Ans[MAX_NCI_FRAME_SIZE];

    (void)writeData(Ans, MAX_NCI_FRAME_SIZE - 3);
    getMessage(2000);

    /* Is data packet ? */
    if ((rxBuffer[0] == 0x00) && (rxBuffer[1] == 0x00))
    {
#ifdef DEBUG2
        Serial.println(rxBuffer[2]);
#endif
        if (rxBuffer[2] >= rxBufferSize)
        {
            return false; // buffer overflow
        }

        *pDataSize = rxBuffer[2];
        memcpy(pData, &rxBuffer[3], *pDataSize);
        status = NFC_SUCCESS;
    }
    else
    {
        status = NFC_ERROR;
    }
    return status;
}


void PN7150::ProcessCardMode(RfIntf_t RfIntf)
{
    uint8_t Answer[MAX_NCI_FRAME_SIZE];

    uint8_t NCIStopDiscovery[] = {0x21, 0x06, 0x01, 0x00};
    bool FirstCmd = true;

    /* Reset Card emulation state */
    T4T_NDEF_EMU_Reset();

    getMessage(2000);

    while (rxMessageLength > 0)
    {
        getMessage(2000);
        /* is RF_DEACTIVATE_NTF ? */
        if ((rxBuffer[0] == 0x61) && (rxBuffer[1] == 0x06))
        {
            if (FirstCmd)
            {
                /* Restart the discovery loop */
                (void)writeData(NCIStopDiscovery, sizeof(NCIStopDiscovery));
                getMessage();
                do
                {
                    if ((rxBuffer[0] == 0x41) && (rxBuffer[1] == 0x06))
                        break;
                    getMessage(100);
                } while (rxMessageLength != 0);

                if (NCIStartDiscovery_length >= maxNCIStartDiscoveryLength)
                {
                    return; // buffer overflow
                }

                (void)writeData(NCIStartDiscovery, NCIStartDiscovery_length);
                getMessage();
            }
            /* Come back to discovery state */
        }
        /* is DATA_PACKET ? */
        else if ((rxBuffer[0] == 0x00) && (rxBuffer[1] == 0x00))
        {
            /* DATA_PACKET */
            uint8_t Cmd[MAX_NCI_FRAME_SIZE];
            uint16_t CmdSize;

            T4T_NDEF_EMU_Next(&rxBuffer[3], rxBuffer[2], &Cmd[3], (unsigned short *)&CmdSize);

            Cmd[0] = 0x00;
            Cmd[1] = (CmdSize & 0xFF00) >> 8;
            Cmd[2] = CmdSize & 0x00FF;

            if (CmdSize >= MAX_NCI_FRAME_SIZE - 3)
            {
                return; // buffer overflow
            }
            (void)writeData(Cmd, CmdSize + 3);
            getMessage();
        }
        FirstCmd = false;
    }
}

void PN7150::handleCardEmulation()
{
    PN7150::ProcessCardMode(this->dummyRfInterface);
}

void PN7150::processReaderMode(RfIntf_t RfIntf, RW_Operation_t Operation)
{
    switch (Operation)
    {
    case READ_NDEF:
        readNdef(RfIntf);
        break;
    case WRITE_NDEF:
        writeNdef(RfIntf);
        break;
    case PRESENCE_CHECK:
        presenceCheck(RfIntf);
        break;
    default:
        break;
    }
}

void PN7150::processP2pMode(RfIntf_t RfIntf)
{
    uint8_t status = ERROR;
    bool restart = false;
    uint8_t NCILlcpSymm[] = {0x00, 0x00, 0x02, 0x00, 0x00};
    uint8_t NCIRestartDiscovery[] = {0x21, 0x06, 0x01, 0x03};

    /* Reset P2P_NDEF state */
    P2P_NDEF_Reset();

    /* Is Initiator mode ? */
    if ((RfIntf.ModeTech & MODE_LISTEN) != MODE_LISTEN)
    {
        /* Initiate communication (SYMM PDU) */
        (void)writeData(NCILlcpSymm, sizeof(NCILlcpSymm));
        getMessage();

        /* Save status for discovery restart */
        restart = true;
    }
    status = ERROR;
    getMessage(2000);
    if (rxMessageLength > 0)
        status = SUCCESS;

    /* Get frame from remote peer */
    while (status == SUCCESS)
    {
        /* is DATA_PACKET ? */
        if ((rxBuffer[0] == 0x00) && (rxBuffer[1] == 0x00))
        {
            uint8_t Cmd[MAX_NCI_FRAME_SIZE];
            uint16_t CmdSize;
            /* Handle P2P communication */
            P2P_NDEF_Next(&rxBuffer[3], rxBuffer[2], &Cmd[3], (unsigned short *)&CmdSize);
            /* Compute DATA_PACKET to answer */
            Cmd[0] = 0x00;
            Cmd[1] = (CmdSize & 0xFF00) >> 8;
            Cmd[2] = CmdSize & 0x00FF;
            status = ERROR;

            if (CmdSize >= MAX_NCI_FRAME_SIZE - 3)
            {
                return; // buffer overflow
            }
            (void)writeData(Cmd, CmdSize + 3);
            getMessage();
            if (rxMessageLength > 0)
                status = SUCCESS;
        }
        /* is CORE_INTERFACE_ERROR_NTF ?*/
        else if ((rxBuffer[0] == 0x60) && (rxBuffer[1] == 0x08))
        {
            /* Come back to discovery state */
            break;
        }
        /* is RF_DEACTIVATE_NTF ? */
        else if ((rxBuffer[0] == 0x61) && (rxBuffer[1] == 0x06))
        {
            /* Come back to discovery state */
            break;
        }
        /* is RF_DISCOVERY_NTF ? */
        else if ((rxBuffer[0] == 0x61) && ((rxBuffer[1] == 0x05) || (rxBuffer[1] == 0x03)))
        {
            do
            {
                if ((rxBuffer[0] == 0x61) && ((rxBuffer[1] == 0x05) || (rxBuffer[1] == 0x03)))
                {
                    if ((rxBuffer[6] & MODE_LISTEN) != MODE_LISTEN)
                        restart = true;
                    else
                        restart = false;
                }
                status = ERROR;

                if (rxMessageLength >= rxBufferSize)
                {
                    return; // buffer overflow
                }

                (void)writeData(rxBuffer, rxMessageLength);
                getMessage();
                if (rxMessageLength > 0)
                    status = SUCCESS;
            } while (rxMessageLength != 0);
            /* Come back to discovery state */
            break;
        }

        /* Wait for next frame from remote P2P, or notification event */
        status = ERROR;

        if (rxMessageLength >= rxBufferSize)
        {
            return; // buffer overflow
        }
        (void)writeData(rxBuffer, rxMessageLength);
        getMessage();
        if (rxMessageLength > 0)
            status = SUCCESS;
    }

    /* Is Initiator mode ? */
    if (restart)
    {
        /* Communication ended, restart discovery loop */
        (void)writeData(NCIRestartDiscovery, sizeof(NCIRestartDiscovery));
        getMessage();
        getMessage(100);
    }
}

void PN7150::presenceCheck(RfIntf_t RfIntf)
{
    bool status;
    uint8_t i;

    uint8_t NCIPresCheckT1T[] = {0x00, 0x00, 0x07, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t NCIPresCheckT2T[] = {0x00, 0x00, 0x02, 0x30, 0x00};
    uint8_t NCIPresCheckT3T[] = {0x21, 0x08, 0x04, 0xFF, 0xFF, 0x00, 0x01};
    uint8_t NCIPresCheckIsoDep[] = {0x2F, 0x11, 0x00};
    uint8_t NCIPresCheckIso15693[] = {0x00, 0x00, 0x0B, 0x26, 0x01, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t NCIDeactivate[] = {0x21, 0x06, 0x01, 0x01};
    uint8_t NCISelectMIFARE[] = {0x21, 0x04, 0x03, 0x01, 0x80, 0x80};

    switch (remoteDevice.getProtocol())
    {
    case PROT_T1T:
        do
        {
            delay(500);
            (void)writeData(NCIPresCheckT1T, sizeof(NCIPresCheckT1T));
            getMessage();
            getMessage(100);
        } while ((rxBuffer[0] == 0x00) && (rxBuffer[1] == 0x00));
        break;

    case PROT_T2T:
        do
        {
            delay(500);
            (void)writeData(NCIPresCheckT2T, sizeof(NCIPresCheckT2T));
            getMessage();
            getMessage(100);
        } while ((rxBuffer[0] == 0x00) && (rxBuffer[1] == 0x00) && (rxBuffer[2] == 0x11));
        break;

    case PROT_T3T:
        do
        {
            delay(500);
            (void)writeData(NCIPresCheckT3T, sizeof(NCIPresCheckT3T));
            getMessage();
            getMessage(100);
        } while ((rxBuffer[0] == 0x61) && (rxBuffer[1] == 0x08) && ((rxBuffer[3] == 0x00) || (rxBuffer[4] > 0x00)));
        break;

    case PROT_ISODEP:
        do
        {
            delay(500);
            (void)writeData(NCIPresCheckIsoDep, sizeof(NCIPresCheckIsoDep));
            getMessage();
            getMessage(100);
        } while ((rxBuffer[0] == 0x6F) && (rxBuffer[1] == 0x11) && (rxBuffer[2] == 0x01) && (rxBuffer[3] == 0x01));
        break;

    case PROT_ISO15693:
        do
        {
            delay(500);
            for (i = 0; i < 8; i++)
            {
                NCIPresCheckIso15693[i + 6] = remoteDevice.getID()[7 - i];
            }
            (void)writeData(NCIPresCheckIso15693, sizeof(NCIPresCheckIso15693));
            getMessage();
            getMessage(100);
            status = ERROR;
            if (rxMessageLength >= rxBufferSize)
            {
                return; // buffer overflow, TODO: change return type of fcn?
            }

            if (rxMessageLength)
                status = SUCCESS;
        } while ((status == SUCCESS) && (rxBuffer[0] == 0x00) && (rxBuffer[1] == 0x00) && (rxBuffer[rxMessageLength - 1] == 0x00));
        break;

    case PROT_MIFARE:
        do
        {
            delay(500);
            /* Deactivate target */
            (void)writeData(NCIDeactivate, sizeof(NCIDeactivate));
            getMessage();
            getMessage(100);

            /* Reactivate target */
            (void)writeData(NCISelectMIFARE, sizeof(NCISelectMIFARE));
            getMessage();
            getMessage(100);
        } while ((rxBuffer[0] == 0x61) && (rxBuffer[1] == 0x05));
        break;

    default:
        /* Nothing to do */
        break;
    }
}

void PN7150::waitForTagRemoval()
{
    PN7150::presenceCheck(this->dummyRfInterface);
}

bool PN7150::readerTagCmd(unsigned char *pCommand, unsigned char CommandSize, unsigned char *pAnswer, unsigned char *pAnswerSize)
{
    bool status = ERROR;
    uint8_t Cmd[MAX_NCI_FRAME_SIZE];

    /* Compute and send DATA_PACKET */
    Cmd[0] = 0x00;
    Cmd[1] = 0x00;
    Cmd[2] = CommandSize;
    if (CommandSize >= MAX_NCI_FRAME_SIZE - 3)
    {
        return ERROR; // buffer overflow
    }
    memcpy(&Cmd[3], pCommand, CommandSize);

    (void)writeData(Cmd, CommandSize + 3);
    getMessage();
    getMessage(1000);
    /* Wait for Answer 1S */

    if ((rxBuffer[0] == 0x0) && (rxBuffer[1] == 0x0))
        status = SUCCESS;

    if (*pAnswerSize >= rxBufferSize -3)
    {
        return ERROR; // buffer overflow - reading past rxBuffer alocated memory would occur
    }

    *pAnswerSize = rxBuffer[2];
    memcpy(pAnswer, &rxBuffer[3], *pAnswerSize);

    return status;
}

bool PN7150::readerReActivate()
{
    uint8_t NCIDeactivate[] = {0x21, 0x06, 0x01, 0x01};
    uint8_t NCIActivate[] = {0x21, 0x04, 0x03, 0x01, 0x00, 0x00};

    /* First de-activate the target */
    (void)writeData(NCIDeactivate, sizeof(NCIDeactivate));
    getMessage();
    getMessage(100);

    /* Then re-activate the target */
    NCIActivate[4] = remoteDevice.getProtocol();
    NCIActivate[5] = remoteDevice.getInterface();

    (void)writeData(NCIDeactivate, sizeof(NCIDeactivate));
    getMessage();
    getMessage(100);

    if ((rxBuffer[0] != 0x61) || (rxBuffer[1] != 0x05))
        return ERROR;
    return SUCCESS;
}

bool PN7150::ReaderActivateNext(RfIntf_t *pRfIntf)
{
    uint8_t NCIStopDiscovery[] = {0x21, 0x06, 0x01, 0x01};
    uint8_t NCIRfDiscoverSelect[] = {0x21, 0x04, 0x03, 0x02, PROT_ISODEP, INTF_ISODEP};

    bool status = ERROR;

    pRfIntf->MoreTags = false;
    remoteDevice.setMoreTagsAvailable(false);

    if (gNextTag_Protocol == protocol.UNDETERMINED)
    {
        pRfIntf->Interface = INTF_UNDETERMINED;
        remoteDevice.setInterface(interface.UNDETERMINED);
        pRfIntf->Protocol = PROT_UNDETERMINED;
        remoteDevice.setProtocol(protocol.UNDETERMINED);
        return ERROR;
    }

    /* First disconnect current tag */
    (void)writeData(NCIStopDiscovery, sizeof(NCIStopDiscovery));
    getMessage();

    if ((rxBuffer[0] != 0x41) && (rxBuffer[1] != 0x06) && (rxBuffer[3] != 0x00))
        return ERROR;
    getMessage(100);

    if ((rxBuffer[0] != 0x61) && (rxBuffer[1] != 0x06))
        return ERROR;

    NCIRfDiscoverSelect[4] = gNextTag_Protocol;
    if (gNextTag_Protocol == PROT_ISODEP)
        NCIRfDiscoverSelect[5] = INTF_ISODEP;
    else if (gNextTag_Protocol == PROT_ISODEP)
        NCIRfDiscoverSelect[5] = INTF_NFCDEP;
    else if (gNextTag_Protocol == PROT_MIFARE)
        NCIRfDiscoverSelect[5] = INTF_TAGCMD;
    else
        NCIRfDiscoverSelect[5] = INTF_FRAME;

    (void)writeData(NCIRfDiscoverSelect, sizeof(NCIRfDiscoverSelect));
    getMessage();

    if ((rxBuffer[0] == 0x41) && (rxBuffer[1] == 0x04) && (rxBuffer[3] == 0x00))
    {
        getMessage(100);
        if ((rxBuffer[0] == 0x61) || (rxBuffer[1] == 0x05))
        {
            pRfIntf->Interface = rxBuffer[4];
            remoteDevice.setInterface(rxBuffer[4]);
            pRfIntf->Protocol = rxBuffer[5];
            remoteDevice.setProtocol(rxBuffer[5]);
            pRfIntf->ModeTech = rxBuffer[6];
            remoteDevice.setModeTech(rxBuffer[6]);
            remoteDevice.setInfo(pRfIntf, &rxBuffer[10]);
            status = SUCCESS;
        }
    }

    return status;
}

bool PN7150::activateNextTagDiscovery()
{
    return !PN7150::ReaderActivateNext(&this->dummyRfInterface);
}

void PN7150::readNdef(RfIntf_t RfIntf)
{
    uint8_t Cmd[MAX_NCI_FRAME_SIZE];
    uint16_t CmdSize = 0;

    RW_NDEF_Reset(remoteDevice.getProtocol());

    while (1)
    {
        RW_NDEF_Read_Next(&rxBuffer[3], rxBuffer[2], &Cmd[3], (unsigned short *)&CmdSize);
        if (CmdSize == 0)
        {
            /// End of the Read operation
            break;
        }
        else
        {
            // Compute and send DATA_PACKET
            Cmd[0] = 0x00;
            Cmd[1] = (CmdSize & 0xFF00) >> 8;
            Cmd[2] = CmdSize & 0x00FF;

            if (CmdSize >= MAX_NCI_FRAME_SIZE - 3)
            {
                return; // buffer overflow
            }

            (void)writeData(Cmd, CmdSize + 3);
            getMessage();
            getMessage(1000);

            // Manage chaining in case of T4T
            if (remoteDevice.getInterface() == INTF_ISODEP && rxBuffer[0] == 0x10)
            {
                uint8_t tmp[MAX_NCI_FRAME_SIZE];
                uint8_t tmpSize = 0;
                while (rxBuffer[0] == 0x10)
                {
                    if (tmpSize + rxBuffer[2] >= MAX_NCI_FRAME_SIZE)
                    {
                        return; // buffer overflow
                    }

                    memcpy(&tmp[tmpSize], &rxBuffer[3], rxBuffer[2]);
                    tmpSize += rxBuffer[2];
                    getMessage(100);
                }

                if (tmpSize + rxBuffer[2] >= MAX_NCI_FRAME_SIZE)
                {
                    return; // buffer overflow
                }

                memcpy(&tmp[tmpSize], &rxBuffer[3], rxBuffer[2]);
                tmpSize += rxBuffer[2];
                //* Compute all chained frame into one unique answer

                if (tmpSize >= rxBufferSize - 3)
                {
                    return; // buffer overflow
                }
                memcpy(&rxBuffer[3], tmp, tmpSize);
                rxBuffer[2] = tmpSize;
            }
        }
    }
}

void PN7150::readNdefMessage(void)
{
    PN7150::readNdef(this->dummyRfInterface);
}

void PN7150::writeNdef(RfIntf_t RfIntf)
{
    uint8_t Cmd[MAX_NCI_FRAME_SIZE];
    uint16_t CmdSize = 0;

    RW_NDEF_Reset(remoteDevice.getProtocol());

    while (1)
    {
        RW_NDEF_Write_Next(&rxBuffer[3], rxBuffer[2], &Cmd[3], (unsigned short *)&CmdSize);
        if (CmdSize == 0)
        {
            // End of the Write operation
            break;
        }
        else
        {
            // Compute and send DATA_PACKET
            Cmd[0] = 0x00;
            Cmd[1] = (CmdSize & 0xFF00) >> 8;
            Cmd[2] = CmdSize & 0x00FF;

            if (CmdSize >= MAX_NCI_FRAME_SIZE - 3)
            {
                return; // buffer overflow
            }
            (void)writeData(Cmd, CmdSize + 3);
            getMessage();
            getMessage(2000);
        }
    }
}

void PN7150::writeNdefMessage(void)
{
    PN7150::writeNdef(this->dummyRfInterface);
}

bool PN7150::nciFactoryTestPrbs(NxpNci_TechType_t type, NxpNci_Bitrate_t bitrate)
{
    uint8_t NCIPrbs[] = {0x2F, 0x30, 0x06, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01};
    uint8_t *NxpNci_cmd;
    uint16_t NxpNci_cmd_size = 0;

    NxpNci_cmd = NCIPrbs;
    NxpNci_cmd_size = sizeof(NCIPrbs);
    NxpNci_cmd[5] = type;
    NxpNci_cmd[6] = bitrate;

    if (NxpNci_cmd_size != 0)
    {
        (void)writeData(NxpNci_cmd, sizeof(NxpNci_cmd));
        getMessage();
        if ((rxBuffer[0] != 0x4F) || (rxBuffer[1] != 0x30) || (rxBuffer[3] != 0x00))
            return ERROR;
    }
    else
    {
        return ERROR;
    }

    return SUCCESS;
}

bool PN7150::nciFactoryTestRfOn()
{
    uint8_t NCIRfOn[] = {0x2F, 0x3D, 0x02, 0x20, 0x01};

    (void)writeData(NCIRfOn, sizeof(NCIRfOn));
    getMessage();
    if ((rxBuffer[0] != 0x4F) || (rxBuffer[1] != 0x3D) || (rxBuffer[3] != 0x00))
        return ERROR;

    return SUCCESS;
}

bool PN7150::reset()
{
    if (PN7150::stopDiscovery())
    {
        return false;
    }

    // Configure settings only if we have not detected a tag yet
    if (remoteDevice.getProtocol() == protocol.UNDETERMINED)
    {
        if (PN7150::configureSettings())
        {
            return false;
        }
    }

    if (PN7150::configMode())
    {
        return false;
    }

    if (PN7150::startDiscovery())
    {
        return false;
    }

    return true;
}

bool PN7150::setReaderWriterMode()
{
    PN7150::setMode(mode.READER_WRITER);
    if (!PN7150::reset())
    {
        return false;
    }
    return true;
}

bool PN7150::setEmulationMode()
{
    PN7150::setMode(mode.EMULATION);
    if (!PN7150::reset())
    {
        return false;
    }
    return true;
}

bool PN7150::setP2PMode()
{
    PN7150::setMode(mode.P2P);
    if (!PN7150::reset())
    {
        return false;
    }
    return true;
}

void PN7150::setReadMsgCallback(std::function<void()> function)
{
    registerNdefReceivedCallback(function);
}

void PN7150::setSendMsgCallback(CustomCallback_t function)
{
    T4T_NDEF_EMU_SetCallback(function);
}

bool PN7150::isReaderDetected()
{
    static unsigned char STATUSOK[] = {0x90, 0x00}, Cmd[256], CmdSize;
    bool status = false;

    if (cardModeReceive(Cmd, &CmdSize) == 0)
    { // Data in buffer?
        if ((CmdSize >= 2) && (Cmd[0] == 0x00))
        { // Expect at least two bytes
            if (Cmd[1] == 0xA4)
            {
                status = true;
            }
            PN7150::closeCommunication();
        }
    }

    return status;
}

void PN7150::closeCommunication()
{
    unsigned char STATUSOK[] = {0x90, 0x00};
    PN7150::cardModeSend(STATUSOK, sizeof(STATUSOK));
}

void PN7150::sendMessage()
{
    PN7150::handleCardEmulation();
    PN7150::closeCommunication();
}
