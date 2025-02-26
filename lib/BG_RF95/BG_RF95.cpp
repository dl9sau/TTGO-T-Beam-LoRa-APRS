// BG_RF95.cpp
//
// Copyright (C) 2011 Mike McCauley
// $Id: BG_RF95.cpp,v 1.11 2016/04/04 01:40:12 mikem Exp $

#ifdef HAS_SX127X
#include <BG_RF95.h>

byte _lastSNR = 0;

// Interrupt vectors for the 3 Arduino interrupt pins
// Each interrupt can be handled by a different instance of BG_RF95, allowing you to have
// 2 or more LORAs per Arduino
BG_RF95* BG_RF95::_deviceForInterrupt[BG_RF95_NUM_INTERRUPTS] = {0, 0, 0};
uint8_t BG_RF95::_interruptCount = 0; // Index into _deviceForInterrupt for next device

// These are indexed by the values of ModemConfigChoice
// Stored in flash (program) memory to save SRAM
PROGMEM static const BG_RF95::ModemConfig MODEM_CONFIG_TABLE[] =
{
    //  1d,     1e,      26
    { 0x72,   0x74,    0x00}, // Bw125Cr45Sf128 (the chip default)
    { 0x92,   0x74,    0x00}, // Bw500Cr45Sf128
    { 0x48,   0x94,    0x00}, // Bw31_25Cr48Sf512
    { 0x78,   0xc7,    0x08}, // Bw125Cr48Sf4096
    { 0x76,   0xc7,    0x08}, // Bw125Cr47Sf4096
    { 0x74,   0xc7,    0x08}, // Bw125Cr46Sf4096
    { 0x72,   0xc7,    0x08}, // Bw125Cr45Sf4096
    { 0x72,   0xb4,    0x00}, // Bw125Cr45Sf2048 <= M0IGA messup speed
    { 0x76,   0x94,    0x04}, // Bw125Cr47Sf512 <= corrected 1200baud
    { 0x78,   0xa4,    0x00}, // Bw125Cr48Sf1024
};

BG_RF95::BG_RF95(uint8_t slaveSelectPin, uint8_t interruptPin, RHGenericSPI& spi)
    :
    RHSPIDriver(slaveSelectPin, spi),
    _rxBufValid(0)
{
    _interruptPin = interruptPin;
    _myInterruptIndex = 0xff; // Not allocated yet
}

bool BG_RF95::init()
{
    if (!RHSPIDriver::init())
	return false;
    //Serial.println("RHSPIDriver::init completed");
    // Determine the interrupt number that corresponds to the interruptPin
    int interruptNumber = digitalPinToInterrupt(_interruptPin);
    if (interruptNumber == NOT_AN_INTERRUPT)
	return false;
#ifdef RH_ATTACHINTERRUPT_TAKES_PIN_NUMBER
    interruptNumber = _interruptPin;
#endif
    //Serial.println("Attach Interrupt completed");

    // No way to check the device type :-(

    // Set sleep mode, so we can also set LORA mode:
    spiWrite(BG_RF95_REG_01_OP_MODE, BG_RF95_MODE_SLEEP | BG_RF95_LONG_RANGE_MODE);
    delay(10); // Wait for sleep mode to take over from say, CAD
    // Check we are in sleep mode, with LORA set
    if (spiRead(BG_RF95_REG_01_OP_MODE) != (BG_RF95_MODE_SLEEP | BG_RF95_LONG_RANGE_MODE))
    {
	   //Serial.println(spiRead(BG_RF95_REG_01_OP_MODE), HEX);
	   return false; // No device present?
    }

    // Add by Adrien van den Bossche <vandenbo@univ-tlse2.fr> for Teensy
    // ARM M4 requires the below. else pin interrupt doesn't work properly.
    // On all other platforms, its innocuous, belt and braces
    pinMode(_interruptPin, INPUT);

    // Set up interrupt handler
    // Since there are a limited number of interrupt glue functions isr*() available,
    // we can only support a limited number of devices simultaneously
    // ON some devices, notably most Arduinos, the interrupt pin passed in is actuallt the
    // interrupt number. You have to figure out the interruptnumber-to-interruptpin mapping
    // yourself based on knwledge of what Arduino board you are running on.
    if (_myInterruptIndex == 0xff)
    {
	// First run, no interrupt allocated yet
	if (_interruptCount <= BG_RF95_NUM_INTERRUPTS)
	    _myInterruptIndex = _interruptCount++;
	else
	    return false; // Too many devices, not enough interrupt vectors
    }
    _deviceForInterrupt[_myInterruptIndex] = this;
    if (_myInterruptIndex == 0)
	attachInterrupt(interruptNumber, isr0, RISING);
    else if (_myInterruptIndex == 1)
	attachInterrupt(interruptNumber, isr1, RISING);
    else if (_myInterruptIndex == 2)
	attachInterrupt(interruptNumber, isr2, RISING);
    else
    {
        //Serial.println("Interrupt vector too many vectors");
        return false; // Too many devices, not enough interrupt vectors
    }

    // Set up FIFO
    // We configure so that we can use the entire 256 byte FIFO for either receive
    // or transmit, but not both at the same time
    spiWrite(BG_RF95_REG_0E_FIFO_TX_BASE_ADDR, 0);
    spiWrite(BG_RF95_REG_0F_FIFO_RX_BASE_ADDR, 0);

    // Packet format is preamble + explicit-header + payload + crc
    // Explicit Header Mode
    // payload is TO + FROM + ID + FLAGS + message data
    // RX mode is implmented with RXCONTINUOUS
    // max message data length is 255 - 4 = 251 octets

    setModeIdle();

    // Set up default configuration
    // No Sync Words in LORA mode.
    setModemConfig(Bw125Cr45Sf128); // Radio default
//    setModemConfig(Bw125Cr48Sf4096); // slow and reliable?
    setPreambleLength(8); // Default is 8
    // An innocuous ISM frequency, same as RF22's
    setFrequency(433.800);
    // Lowish power
    setTxPower(20);

    return true;
}

// C++ level interrupt handler for this instance
// LORA is unusual in that it has several interrupt lines, and not a single, combined one.
// On MiniWirelessLoRa, only one of the several interrupt lines (DI0) from the RFM95 is usefuly
// connnected to the processor.
// We use this to get RxDone and TxDone interrupts
void BG_RF95::handleInterrupt()
{
    // Read the interrupt register
    //Serial.println("HandleInterrupt");
    uint8_t irq_flags = spiRead(BG_RF95_REG_12_IRQ_FLAGS);
    if (_mode == RHModeRx && irq_flags & (BG_RF95_RX_TIMEOUT | BG_RF95_PAYLOAD_CRC_ERROR))
    {
	_rxBad++;
    }
    else if (_mode == RHModeRx && irq_flags & BG_RF95_RX_DONE)
    {
	// Have received a packet
	uint8_t len = spiRead(BG_RF95_REG_13_RX_NB_BYTES);

	// Reset the fifo read ptr to the beginning of the packet
	spiWrite(BG_RF95_REG_0D_FIFO_ADDR_PTR, spiRead(BG_RF95_REG_10_FIFO_RX_CURRENT_ADDR));
	spiBurstRead(BG_RF95_REG_00_FIFO, _buf, len);
	_bufLen = len;
	spiWrite(BG_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags

	// Remember the RSSI of this packet
	// this is according to the doc, but is it really correct?
	// weakest receiveable signals are reported RSSI at about -66
	_lastRssi = spiRead(BG_RF95_REG_1A_PKT_RSSI_VALUE) - 137;

	_lastSNR = spiRead(BG_RF95_REG_19_PKT_SNR_VALUE);

	// We have received a message.
	validateRxBuf();
	if (_rxBufValid)
	    setModeIdle(); // Got one
    }
    else if (_mode == RHModeTx && irq_flags & BG_RF95_TX_DONE)
    {
	_txGood++;
	setModeIdle();
    }

    spiWrite(BG_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags
}

// These are low level functions that call the interrupt handler for the correct
// instance of BG_RF95.
// 3 interrupts allows us to have 3 different devices
void BG_RF95::isr0()
{
    if (_deviceForInterrupt[0])
	_deviceForInterrupt[0]->handleInterrupt();
}
void BG_RF95::isr1()
{
    if (_deviceForInterrupt[1])
	_deviceForInterrupt[1]->handleInterrupt();
}
void BG_RF95::isr2()
{
    if (_deviceForInterrupt[2])
	_deviceForInterrupt[2]->handleInterrupt();
}

// Check whether the latest received message is complete and uncorrupted
void BG_RF95::validateRxBuf()
{
 _promiscuous = 1;
   if (_bufLen < 4)
	   return; // Too short to be a real message
    // Extract the 4 headers
    //Serial.println("validateRxBuf >= 4");
    _rxHeaderTo    = _buf[0];
    _rxHeaderFrom  = _buf[1];
    _rxHeaderId    = _buf[2];
    _rxHeaderFlags = _buf[3];
    if (_promiscuous ||
	_rxHeaderTo == _thisAddress ||
	_rxHeaderTo == RH_BROADCAST_ADDRESS)
    {
	_rxGood++;
	_rxBufValid = true;
    }
}

bool BG_RF95::available()
{
    if (_mode == RHModeTx)
	return false;
    setModeRx();
    return _rxBufValid; // Will be set by the interrupt handler when a good message is received
}

void BG_RF95::clearRxBuf()
{
    ATOMIC_BLOCK_START;
    _rxBufValid = false;
    _bufLen = 0;
    ATOMIC_BLOCK_END;
}


// BG 3 Byte header
bool BG_RF95::recvAPRS(uint8_t* buf, uint8_t* len)
{
    if (!available())
	return false;
    if (buf && len)
    {
	ATOMIC_BLOCK_START;
	// Skip the 4 headers that are at the beginning of the rxBuf
	if (*len > _bufLen-BG_RF95_HEADER_LEN)
	    *len = _bufLen-(BG_RF95_HEADER_LEN-1);
	memcpy(buf, _buf+(BG_RF95_HEADER_LEN-1), *len);    //  BG only 3 Byte header (-1)
	ATOMIC_BLOCK_END;
    }
    clearRxBuf(); // This message accepted and cleared
    return true;
}

bool BG_RF95::recv(uint8_t* buf, uint8_t* len)
{
    if (!available())
	return false;
    if (buf && len)
    {
	ATOMIC_BLOCK_START;
	// Skip the 4 headers that are at the beginning of the rxBuf
	if (*len > _bufLen-BG_RF95_HEADER_LEN)
	    *len = _bufLen-BG_RF95_HEADER_LEN;
	memcpy(buf, _buf+BG_RF95_HEADER_LEN, *len);
	ATOMIC_BLOCK_END;
    }
    clearRxBuf(); // This message accepted and cleared
    return true;
}

uint8_t BG_RF95::lastSNR()
{
	return(_lastSNR);
}


bool BG_RF95::send(const uint8_t* data, uint8_t len)
{
    if (len > BG_RF95_MAX_MESSAGE_LEN)
	return false;

    waitPacketSent(); // Make sure we dont interrupt an outgoing message
    setModeIdle();

    // Position at the beginning of the FIFO
    spiWrite(BG_RF95_REG_0D_FIFO_ADDR_PTR, 0);
    // The headers
    spiWrite(BG_RF95_REG_00_FIFO, _txHeaderTo);
    spiWrite(BG_RF95_REG_00_FIFO, _txHeaderFrom);
    spiWrite(BG_RF95_REG_00_FIFO, _txHeaderId);
    spiWrite(BG_RF95_REG_00_FIFO, _txHeaderFlags);
    // The message data
    spiBurstWrite(BG_RF95_REG_00_FIFO, data, len);
    spiWrite(BG_RF95_REG_22_PAYLOAD_LENGTH, len + BG_RF95_HEADER_LEN);

    setModeTx(); // Start the transmitter
    // when Tx is done, interruptHandler will fire and radio mode will return to STANDBY
    return true;
}

bool BG_RF95::sendAPRS(const uint8_t* data, uint8_t len)
{
    if (len > BG_RF95_MAX_MESSAGE_LEN)
	return false;

    waitPacketSent(); // Make sure we dont interrupt an outgoing message
    setModeIdle();

    // Position at the beginning of the FIFO
    spiWrite(BG_RF95_REG_0D_FIFO_ADDR_PTR, 0);
    // The headers   for APRS
    spiWrite(BG_RF95_REG_00_FIFO, '<');
    spiWrite(BG_RF95_REG_00_FIFO, _txHeaderFrom);
    spiWrite(BG_RF95_REG_00_FIFO, 0x1 );
    //spiWrite(BG_RF95_REG_00_FIFO, _txHeaderFlags);
    // The message data
    spiBurstWrite(BG_RF95_REG_00_FIFO, data, len);
    spiWrite(BG_RF95_REG_22_PAYLOAD_LENGTH, len + BG_RF95_HEADER_LEN -1 );   // only 3 Byte header  BG

    setModeTx(); // Start the transmitter
    // when Tx is done, interruptHandler will fire and radio mode will return to STANDBY
    return true;
}

bool BG_RF95::printRegisters()
{
#ifdef RH_HAVE_SERIAL
    uint8_t registers[] = { 0x01, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x014, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x4d };

    uint8_t i;
    for (i = 0; i < sizeof(registers); i++)
    {
	Serial.print(registers[i], HEX);
	Serial.print(": ");
	Serial.println(spiRead(registers[i]), HEX);
    }
#endif
    return true;
}

uint8_t BG_RF95::maxMessageLength()
{
    return BG_RF95_MAX_MESSAGE_LEN;
}

bool BG_RF95::setFrequency(float centre)
{
    // Frf = FRF / FSTEP
    uint32_t frf = (centre * 1000000.0) / BG_RF95_FSTEP;
    spiWrite(BG_RF95_REG_06_FRF_MSB, (frf >> 16) & 0xff);
    spiWrite(BG_RF95_REG_07_FRF_MID, (frf >> 8) & 0xff);
    spiWrite(BG_RF95_REG_08_FRF_LSB, frf & 0xff);

    return true;
}

void BG_RF95::setModeIdle()
{
    if (_mode != RHModeIdle)
    {
	spiWrite(BG_RF95_REG_01_OP_MODE, BG_RF95_MODE_STDBY);
	_mode = RHModeIdle;
    }
}


bool BG_RF95::sleep()
{
    if (_mode != RHModeSleep)
    {
	spiWrite(BG_RF95_REG_01_OP_MODE, BG_RF95_MODE_SLEEP);
	_mode = RHModeSleep;
    }
    return true;
}

void BG_RF95::setModeRx()
{
    if (_mode != RHModeRx)
    {
       //Serial.println("SetModeRx");
       _mode = RHModeRx;
	   spiWrite(BG_RF95_REG_01_OP_MODE, BG_RF95_MODE_RXCONTINUOUS);
	   spiWrite(BG_RF95_REG_40_DIO_MAPPING1, 0x00); // Interrupt on RxDone
    }
}

void BG_RF95::setModeTx()
{
    if (_mode != RHModeTx)
    {
    _mode = RHModeTx;       // set first to avoid possible race condition
	spiWrite(BG_RF95_REG_01_OP_MODE, BG_RF95_MODE_TX);
	spiWrite(BG_RF95_REG_40_DIO_MAPPING1, 0x40); // Interrupt on TxDone
    }
}

void BG_RF95::setTxPower(int8_t power, bool useRFO)
{
    // Sigh, different behaviours depending on whther the module use PA_BOOST or the RFO pin
    // for the transmitter output
    if (useRFO)
    {
	if (power > 14)  power = 14;
	if (power < -1)  power = -1;
	spiWrite(BG_RF95_REG_09_PA_CONFIG, BG_RF95_MAX_POWER | (power + 1));
    } else {
	if (power > 23)  power = 23;
	if (power < 5) 	 power = 5;

	// For BG_RF95_PA_DAC_ENABLE, manual says '+20dBm on PA_BOOST when OutputPower=0xf'
	// BG_RF95_PA_DAC_ENABLE actually adds about 3dBm to all power levels. We will us it
	// for 21, 22 and 23dBm -= 3;
	}
	if (power > 20) {
		spiWrite(BG_RF95_REG_0B_OCP, ( BG_RF95_OCP_ON | BG_RF95_OCP_TRIM ) );   // Trim max current tp 240mA
	    	spiWrite(BG_RF95_REG_4D_PA_DAC, BG_RF95_PA_DAC_ENABLE);
	    //power -= 3;
		power = 20;  // and set PA_DAC_ENABLE

	} else {
	    spiWrite(BG_RF95_REG_4D_PA_DAC, BG_RF95_PA_DAC_DISABLE);
	}

	// RFM95/96/97/98 does not have RFO pins connected to anything. Only PA_BOOST
	// pin is connected, so must use PA_BOOST
	// Pout = 2 + OutputPower.
	// The documentation is pretty confusing on this topic: PaSelect says the max power is 20dBm,
	// but OutputPower claims it would be 17dBm.
	// My measurements show 20dBm is correct
	//spiWrite(BG_RF95_REG_09_PA_CONFIG, (BG_RF95_PA_SELECT | (power-5)) );
	spiWrite(BG_RF95_REG_09_PA_CONFIG, (BG_RF95_PA_SELECT | BG_RF95_MAX_POWER | (power-5)) );

    //}
}

// Sets registers from a canned modem configuration structure
void BG_RF95::setModemRegisters(const ModemConfig* config)
{
    spiWrite(BG_RF95_REG_1D_MODEM_CONFIG1,       config->reg_1d);
    spiWrite(BG_RF95_REG_1E_MODEM_CONFIG2,       config->reg_1e);
    spiWrite(BG_RF95_REG_26_MODEM_CONFIG3,       config->reg_26);
}

// Set one of the canned FSK Modem configs
// Returns true if its a valid choice
bool BG_RF95::setModemConfig(ModemConfigChoice index)
{
    if (index > (signed int)(sizeof(MODEM_CONFIG_TABLE) / sizeof(ModemConfig)))
        return false;

    ModemConfig cfg;
    memcpy_P(&cfg, &MODEM_CONFIG_TABLE[index], sizeof(BG_RF95::ModemConfig));
    setModemRegisters(&cfg);

    return true;
}

void BG_RF95::setPreambleLength(uint16_t bytes)
{
    spiWrite(BG_RF95_REG_20_PREAMBLE_MSB, bytes >> 8);
    spiWrite(BG_RF95_REG_21_PREAMBLE_LSB, bytes & 0xff);
}

bool BG_RF95::SignalDetected(void)
{
  return ((spiRead(BG_RF95_REG_18_MODEM_STAT) & 0x01) == 0x01);
}
#endif
