// ============================================================
//  DaliController.cpp
//
//  Hardware implementation of the DALI (IEC 62386) library.
// ============================================================

#include "DaliController.h"

// ============================================================
//  DALI Part 1 (IEC 62386-102) command byte constants
// ============================================================
static const uint8_t CMD_OFF              = 0x00;
static const uint8_t CMD_STEP_UP          = 0x01;
static const uint8_t CMD_STEP_DOWN        = 0x02;
static const uint8_t CMD_RECALL_MAX       = 0x05;
static const uint8_t CMD_RECALL_MIN       = 0x06;
static const uint8_t CMD_RESET            = 0x20;
static const uint8_t CMD_STORE_ACT_DTR    = 0x22;
static const uint8_t CMD_SET_FADE_TIME    = 0x2E;
static const uint8_t CMD_SET_FADE_RATE    = 0x2F;
static const uint8_t CMD_QUERY_STATUS     = 0x90;
static const uint8_t CMD_QUERY_DEV_TYPE   = 0x99;
static const uint8_t CMD_QUERY_ACT_LVL    = 0xA0;
static const uint8_t CMD_QUERY_MAX_LVL    = 0xA1;
static const uint8_t CMD_QUERY_MIN_LVL    = 0xA2;
static const uint8_t CMD_QUERY_GROUPS_0_7 = 0xC0;
static const uint8_t CMD_QUERY_GROUPS_8_15= 0xC1;

// DTR broadcast opcodes (first byte of a special 2-byte frame)
static const uint8_t DTR0_OPCODE          = 0xA3;
static const uint8_t DTR1_OPCODE          = 0xC3;

// Read Memory Location command byte
static const uint8_t CMD_READ_MEMORY      = 0xC5;

// ============================================================
//  Constructor
// ============================================================
DaliController::DaliController()
    : _tx_ch(RMT_CHANNEL_0),
      _rx_ch(RMT_CHANNEL_4),
      _rx_ringbuf(nullptr),
      _ready(false),
      _transitionCount(0)
{
    memset(_receivedBits,     0, sizeof(_receivedBits));
    memset(_transitionTimes,  0, sizeof(_transitionTimes));
    memset(_transitionValues, 0, sizeof(_transitionValues));
}

// ============================================================
//  begin()  -  configure RMT TX + RX
// ============================================================
bool DaliController::begin(int tx_pin, int rx_pin,
                           rmt_channel_t tx_ch,
                           rmt_channel_t rx_ch)
{
    _tx_ch = tx_ch;
    _rx_ch = rx_ch;

    // ---- TX channel ----
    rmt_config_t tx_cfg = RMT_DEFAULT_CONFIG_TX((gpio_num_t)tx_pin, _tx_ch);
    tx_cfg.clk_div                  = CLK_DIV;            // 1 tick = 1 us
    tx_cfg.tx_config.idle_level     = RMT_IDLE_LEVEL_LOW; // DALI bus idles LOW
    tx_cfg.tx_config.idle_output_en = true;
    tx_cfg.tx_config.carrier_en     = false;
    tx_cfg.tx_config.loop_en        = false;

    if (rmt_config(&tx_cfg) != ESP_OK) {
        Serial.println(F("[DALI] RMT TX config FAILED"));
        return false;
    }
    if (rmt_driver_install(_tx_ch, 0, 0) != ESP_OK) {
        Serial.println(F("[DALI] RMT TX driver install FAILED"));
        return false;
    }

    // ---- RX channel ----
    rmt_config_t rx_cfg = RMT_DEFAULT_CONFIG_RX((gpio_num_t)rx_pin, _rx_ch);
    rx_cfg.clk_div                       = CLK_DIV;
    rx_cfg.rx_config.idle_threshold      = RX_IDLE_THRESH;  // 2000 us gap -> end of frame
    rx_cfg.rx_config.filter_ticks_thresh = 100;             // ignore glitches < 100 us
    rx_cfg.rx_config.filter_en           = true;

    if (rmt_config(&rx_cfg) != ESP_OK) {
        Serial.println(F("[DALI] RMT RX config FAILED"));
        return false;
    }
    if (rmt_driver_install(_rx_ch,
                           MAX_RMT_SYMBOLS * sizeof(rmt_item32_t), 0) != ESP_OK) {
        Serial.println(F("[DALI] RMT RX driver install FAILED"));
        return false;
    }

    rmt_get_ringbuf_handle(_rx_ch, &_rx_ringbuf);

    _ready = true;
    Serial.println(F("[DALI] RMT TX+RX initialised (1 us/tick, legacy driver)"));
    return true;
}

// ============================================================
//  buildFrame()  -  static helper
//
//  Constructs a standard 17-bit DALI command frame.
//
//  frame[0]     = 1  (start bit)
//  frame[1]     = 0 address / 1 group+broadcast
//  frame[2..7]  = 6-bit address/group (MSB first)
//  frame[8]     = 0 DAPC / 1 command
//  frame[9..16] = 8-bit command or level (MSB first)
// ============================================================
void DaliController::buildFrame(uint8_t frame[17],
                                uint8_t addrField,
                                bool    isGroup,
                                uint8_t cmdByte,
                                bool    isCommand)
{
    frame[0] = 1;
    frame[1] = isGroup ? 1 : 0;

    // 6-bit address/group field - MSB first into frame[2..7]
    for (int k = 0; k < 6; k++) {
        frame[2 + k] = (addrField >> (5 - k)) & 0x01;
    }

    frame[8] = isCommand ? 1 : 0;

    // 8-bit command/level - MSB first into frame[9..16]
    for (int i = 0; i < 8; i++) {
        frame[9 + i] = (cmdByte >> (7 - i)) & 0x01;
    }
}

// ============================================================
//  sendRawCommand()  -  Manchester-encode 17 bits via RMT TX
//
//  DALI Manchester (IEC 62386):
//    bit '1' -> HIGH half-period then LOW  half-period
//    bit '0' -> LOW  half-period then HIGH half-period
//  Stop: hold LOW for STOP_US, then end-of-TX (duration1 = 0).
// ============================================================
void DaliController::sendRawCommand(const uint8_t command[17])
{
    if (!_ready) {
        Serial.println(F("[DALI] sendRawCommand: not initialised"));
        return;
    }

    const int NUM_SYMS = NUM_BITS + 1;   // 17 data + 1 stop symbol
    rmt_item32_t symbols[NUM_SYMS];

    for (int i = 0; i < NUM_BITS; i++) {
        if (command[i] == 1) {
            symbols[i].duration0 = HALF_BIT_US;  symbols[i].level0 = 1;
            symbols[i].duration1 = HALF_BIT_US;  symbols[i].level1 = 0;
        } else {
            symbols[i].duration0 = HALF_BIT_US;  symbols[i].level0 = 0;
            symbols[i].duration1 = HALF_BIT_US;  symbols[i].level1 = 1;
        }
    }

    // Stop symbol: hold LOW for STOP_US, duration1 = 0 signals end-of-TX
    symbols[NUM_BITS].duration0 = STOP_US;  symbols[NUM_BITS].level0 = 0;
    symbols[NUM_BITS].duration1 = 0;        symbols[NUM_BITS].level1 = 0;

    rmt_write_items(_tx_ch, symbols, NUM_SYMS, true /*blocking*/);
}

// ============================================================
//  receiveResponse()
//
//  Starts RMT RX, waits up to RX_TIMEOUT_MS for a DALI answer
//  frame, decodes Manchester, and returns the 8-bit payload.
//  Returns -1 on timeout or decode failure.
// ============================================================
int DaliController::receiveResponse()
{
    if (!_ready || _rx_ringbuf == nullptr) return -1;

    _transitionCount = 0;
    memset(_receivedBits, 0, sizeof(_receivedBits));

    rmt_rx_start(_rx_ch, true);

    size_t rx_size = 0;
    rmt_item32_t *items = (rmt_item32_t *)xRingbufferReceive(
        _rx_ringbuf, &rx_size, pdMS_TO_TICKS(RX_TIMEOUT_MS));

    rmt_rx_stop(_rx_ch);

    if (items == nullptr || rx_size == 0) return -1;

    size_t num_symbols = rx_size / sizeof(rmt_item32_t);

    // ----------------------------------------------------------
    //  Convert RMT symbols -> transition edge arrays.
    //  Each rmt_item32_t holds two consecutive pulses:
    //    pulse A: level0 for duration0 us
    //    pulse B: level1 for duration1 us
    //  We record only transitions (level changes).
    // ----------------------------------------------------------
    unsigned long t = 0;
    int lastLevel   = -1;

    for (size_t i = 0; i < num_symbols && _transitionCount < 20; i++) {
        // First pulse of symbol
        if (items[i].duration0 > 0) {
            int lvl = (int)items[i].level0;
            if (lvl != lastLevel) {
                _transitionTimes[_transitionCount]  = t;
                _transitionValues[_transitionCount] = lvl;
                _transitionCount++;
                lastLevel = lvl;
            }
            t += items[i].duration0;
        }

        // Second pulse of symbol
        if (items[i].duration1 > 0 && _transitionCount < 20) {
            int lvl = (int)items[i].level1;
            if (lvl != lastLevel) {
                _transitionTimes[_transitionCount]  = t;
                _transitionValues[_transitionCount] = lvl;
                _transitionCount++;
                lastLevel = lvl;
            }
            t += items[i].duration1;
        }

        // duration1 == 0 is the RMT end-of-frame marker
        if (items[i].duration1 == 0) break;
    }

    vRingbufferReturnItem(_rx_ringbuf, (void *)items);

    if (_transitionCount < 2) return -1;
    if (!manchesterDecode())  return -1;

    // Assemble 8-bit value from bits[1..8]  (bits[0] is the start bit)
    uint8_t value = 0;
    for (int i = 1; i < 9; i++) {
        value |= (_receivedBits[i] << (8 - i));
    }
    return (int)value;
}

// ============================================================
//  manchesterDecode()
//
//  Operates on _transitionTimes / _transitionValues populated
//  by receiveResponse().  Fills _receivedBits[0..8].
//
//  DALI Manchester mid-bit transition rule:
//    Falling edge at mid-bit -> '1'
//    Rising  edge at mid-bit -> '0'
// ============================================================
bool DaliController::manchesterDecode()
{
    memset(_receivedBits, 0, sizeof(_receivedBits));
    if (_transitionCount < 2) return false;

    // Find the start of data: first transition after a gap > 2000 us.
    // For RMT RX (no TX echo) data typically starts at index 0.
    int dataStartIndex = 0;
    for (int i = 1; i < _transitionCount; i++) {
        if ((_transitionTimes[i] - _transitionTimes[i - 1]) > 2000) {
            dataStartIndex = i;
            break;
        }
    }

    int bitIndex           = 0;
    int transIndex         = dataStartIndex;
    unsigned long bitStart = _transitionTimes[dataStartIndex];

    while (bitIndex < 9 && transIndex < _transitionCount) {
        unsigned long bitEnd = bitStart + 833;
        unsigned long bitMid = bitStart + 416;

        int midDir = 0;  // -1 = falling = '1',  +1 = rising = '0'

        for (int i = transIndex; i < _transitionCount; i++) {
            if (_transitionTimes[i] >= bitStart &&
                _transitionTimes[i] <= bitEnd) {
                // Is this transition close to the mid-bit point?
                if (abs((long)(_transitionTimes[i] - bitMid)) < 150) {
                    if (i > 0) {
                        if      (_transitionValues[i-1]==1 && _transitionValues[i]==0)
                            midDir = -1;  // falling -> '1'
                        else if (_transitionValues[i-1]==0 && _transitionValues[i]==1)
                            midDir =  1;  // rising  -> '0'
                    }
                }
            }
            if (_transitionTimes[i] > bitEnd) {
                transIndex = i;
                break;
            }
        }

        if      (midDir == -1) _receivedBits[bitIndex] = 1;
        else if (midDir ==  1) _receivedBits[bitIndex] = 0;
        else if (bitIndex > 0) _receivedBits[bitIndex] = _receivedBits[bitIndex - 1];

        bitIndex++;
        bitStart = bitEnd;
    }

    return true;
}

// ============================================================
//  DTR helpers
//
//  These use a special broadcast frame format:
//    frame[0]     = 1 (start bit)
//    frame[1..8]  = opcode (0xA3 for DTR0, 0xC3 for DTR1)
//    frame[9..16] = value
// ============================================================
void DaliController::setDTR0(uint8_t value)
{
    uint8_t frame[17];
    frame[0] = 1;
    for (int i = 0; i < 8; i++) frame[1 + i] = (DTR0_OPCODE >> (7 - i)) & 0x01;
    for (int i = 0; i < 8; i++) frame[9 + i] = (value       >> (7 - i)) & 0x01;
    sendRawCommand(frame);
    delay(15);
}

void DaliController::setDTR1(uint8_t value)
{
    uint8_t frame[17];
    frame[0] = 1;
    for (int i = 0; i < 8; i++) frame[1 + i] = (DTR1_OPCODE >> (7 - i)) & 0x01;
    for (int i = 0; i < 8; i++) frame[9 + i] = (value       >> (7 - i)) & 0x01;
    sendRawCommand(frame);
    delay(15);
}

// ============================================================
//  readMemoryLocation()
//
//  Sends the "Read Memory Location" command to a specific device.
//  DTR0 (byte address) and DTR1 (bank) must already be set.
// ============================================================
void DaliController::readMemoryLocation(uint8_t device_address)
{
    uint8_t frame[17];
    // Addressed command frame: address byte = (addr << 1) | 0x01
    uint8_t addressByte = (device_address << 1) | 0x01;
    frame[0] = 1;
    for (int i = 0; i < 8; i++) frame[1 + i] = (addressByte   >> (7 - i)) & 0x01;
    for (int i = 0; i < 8; i++) frame[9 + i] = (CMD_READ_MEMORY >> (7 - i)) & 0x01;
    sendRawCommand(frame);
}

// ============================================================
//  validateDTR0Range()
// ============================================================
bool DaliController::validateDTR0Range(uint8_t bank, uint8_t address) const
{
    switch (bank) {
        case 0:   return address <= 0x1A;
        case 1:   return address <= 0x77;
        case 202: return address <= 0x0F;
        case 204: return address <= 0x0F;
        case 205: return address <= 0x1C;
        case 206: return address <= 0x20;
        default:  return false;
    }
}

// ============================================================
//  readMemoryBank()
// ============================================================
uint8_t DaliController::readMemoryBank(uint8_t device_address,
                                       uint8_t bank,
                                       uint8_t address)
{
    if (device_address > 63)               return 0xFF;
    if (!validateDTR0Range(bank, address)) return 0xFF;

    setDTR1(bank);
    setDTR0(address);
    readMemoryLocation(device_address);

    int result = receiveResponse();
    return (result < 0) ? 0xFF : (uint8_t)result;
}

// ============================================================
//  Basic on / off
// ============================================================
void DaliController::turnOnAddress(int address)
{
    uint8_t frame[17];
    buildFrame(frame, (uint8_t)address, false, CMD_RECALL_MAX, true);
    sendRawCommand(frame);
}

void DaliController::turnOnGroup(int group)
{
    uint8_t frame[17];
    buildFrame(frame, (uint8_t)group, true, CMD_RECALL_MAX, true);
    sendRawCommand(frame);
    delay(15);
}

void DaliController::turnOnAll()
{
    // Broadcast: all bits in the address byte set to 1 (0xFF address field)
    uint8_t frame[17];
    for (int k = 0; k < 9; k++) frame[k] = 1;
    for (int i = 0; i < 8; i++) frame[9 + i] = (CMD_RECALL_MAX >> (7 - i)) & 0x01;
    sendRawCommand(frame);
    delay(15);
}

void DaliController::turnOffAddress(int address)
{
    uint8_t frame[17];
    buildFrame(frame, (uint8_t)address, false, CMD_OFF, true);
    sendRawCommand(frame);
}

void DaliController::turnOffGroup(int group)
{
    uint8_t frame[17];
    buildFrame(frame, (uint8_t)group, true, CMD_OFF, true);
    sendRawCommand(frame);
    delay(15);
}

void DaliController::turnOffAll()
{
    uint8_t frame[17];
    for (int k = 0; k < 9; k++) frame[k] = 1;
    for (int i = 0; i < 8; i++) frame[9 + i] = (CMD_OFF >> (7 - i)) & 0x01;
    sendRawCommand(frame);
    delay(15);
}

// ============================================================
//  Brightness (DAPC)
// ============================================================
void DaliController::setBrightness(uint8_t address, uint8_t percent)
{
    if (percent > 100 || address > 63) return;
    uint8_t level = (uint8_t)map(percent, 0, 100, 0, 254);
    uint8_t frame[17];
    buildFrame(frame, address, false, level, false /*DAPC*/);
    sendRawCommand(frame);
}

void DaliController::setGroupBrightness(uint8_t group, uint8_t percent)
{
    if (percent > 100 || group > 15) return;
    uint8_t level = (uint8_t)map(percent, 0, 100, 110, 254);
    uint8_t frame[17];
    buildFrame(frame, group, true, level, false /*DAPC*/);
    sendRawCommand(frame);
}

void DaliController::setAllBrightness(uint8_t percent)
{
    if (percent > 100) return;
    uint8_t level = (uint8_t)map(percent, 0, 100, 110, 254);
    uint8_t frame[17];
    for (int k = 0; k < 9; k++) frame[k] = 1;  // broadcast address
    frame[8] = 0;                                 // DAPC mode
    for (int i = 0; i < 8; i++) frame[9 + i] = (level >> (7 - i)) & 0x01;
    sendRawCommand(frame);
}

// ============================================================
//  Step Up / Down
// ============================================================
void DaliController::stepUpAddress(int address)
{
    uint8_t frame[17];
    buildFrame(frame, (uint8_t)address, false, CMD_STEP_UP, true);
    sendRawCommand(frame);
}

void DaliController::stepDownAddress(int address)
{
    uint8_t frame[17];
    buildFrame(frame, (uint8_t)address, false, CMD_STEP_DOWN, true);
    sendRawCommand(frame);
}

void DaliController::stepUpGroup(int group)
{
    uint8_t frame[17];
    buildFrame(frame, (uint8_t)group, true, CMD_STEP_UP, true);
    sendRawCommand(frame);
    delay(15);
}

void DaliController::stepDownGroup(int group)
{
    uint8_t frame[17];
    buildFrame(frame, (uint8_t)group, true, CMD_STEP_DOWN, true);
    sendRawCommand(frame);
    delay(15);
}

// ============================================================
//  Recall Max / Min Level
// ============================================================
void DaliController::recallMaxLevelAddress(int address)
{
    uint8_t frame[17];
    buildFrame(frame, (uint8_t)address, false, CMD_RECALL_MAX, true);
    sendRawCommand(frame);
}

void DaliController::recallMinLevelAddress(int address)
{
    uint8_t frame[17];
    buildFrame(frame, (uint8_t)address, false, CMD_RECALL_MIN, true);
    sendRawCommand(frame);
}

void DaliController::recallMaxLevelGroup(int group)
{
    uint8_t frame[17];
    buildFrame(frame, (uint8_t)group, true, CMD_RECALL_MAX, true);
    sendRawCommand(frame);
    delay(15);
}

void DaliController::recallMinLevelGroup(int group)
{
    uint8_t frame[17];
    buildFrame(frame, (uint8_t)group, true, CMD_RECALL_MIN, true);
    sendRawCommand(frame);
    delay(15);
}

// ============================================================
//  Scenes
// ============================================================
void DaliController::setScene(int address, uint8_t scene, uint8_t level)
{
    // Store level in DTR0 first, then send Store DTR As Scene (twice - config cmd)
    setDTR0(level);
    delay(15);

    uint8_t frame[17];
    uint8_t cmd = 0x40 | (scene & 0x0F);  // Store DTR as Scene N
    buildFrame(frame, (uint8_t)address, false, cmd, true);
    sendRawCommand(frame);
    delay(75);
    sendRawCommand(frame);  // config commands must be sent twice
}

void DaliController::recallSceneAddress(int address, uint8_t scene)
{
    uint8_t frame[17];
    uint8_t cmd = 0x10 | (scene & 0x0F);  // Go To Scene N
    buildFrame(frame, (uint8_t)address, false, cmd, true);
    sendRawCommand(frame);
}

void DaliController::recallSceneGroup(int group, uint8_t scene)
{
    uint8_t frame[17];
    uint8_t cmd = 0x10 | (scene & 0x0F);  // Go To Scene N
    buildFrame(frame, (uint8_t)group, true, cmd, true);
    sendRawCommand(frame);
}

// ============================================================
//  Group membership
//  Add/Remove group are configuration commands - must be sent twice
// ============================================================
void DaliController::addDeviceToGroup(int address, int group)
{
    uint8_t frame[17];
    uint8_t cmd = 0x60 | (group & 0x0F);  // Add To Group N
    buildFrame(frame, (uint8_t)address, false, cmd, true);
    sendRawCommand(frame);
    delay(75);
    sendRawCommand(frame);
}

void DaliController::removeDeviceFromGroup(int address, int group)
{
    uint8_t frame[17];
    uint8_t cmd = 0x70 | (group & 0x0F);  // Remove From Group N
    buildFrame(frame, (uint8_t)address, false, cmd, true);
    sendRawCommand(frame);
    delay(75);
    sendRawCommand(frame);
}

// ============================================================
//  Fade time / rate
//  Fade commands require DTR0 pre-loaded and must be sent twice
// ============================================================
void DaliController::setFadeTime(int address, uint8_t fadeTime)
{
    setDTR0(fadeTime);
    delay(50);

    uint8_t frame[17];
    buildFrame(frame, (uint8_t)address, false, CMD_SET_FADE_TIME, true);
    sendRawCommand(frame);
    delay(75);
    sendRawCommand(frame);
}

void DaliController::setFadeRate(int address, uint8_t fadeRate)
{
    setDTR0(fadeRate);
    delay(50);

    uint8_t frame[17];
    buildFrame(frame, (uint8_t)address, false, CMD_SET_FADE_RATE, true);
    sendRawCommand(frame);
    delay(75);
    sendRawCommand(frame);
}

// ============================================================
//  Reset / store
//  Reset is a configuration command - must be sent twice
// ============================================================
void DaliController::resetAddress(int address)
{
    uint8_t frame[17];
    buildFrame(frame, (uint8_t)address, false, CMD_RESET, true);
    sendRawCommand(frame);
    delay(100);
    sendRawCommand(frame);
}

void DaliController::storeActualLevelInDTR(int address)
{
    uint8_t frame[17];
    buildFrame(frame, (uint8_t)address, false, CMD_STORE_ACT_DTR, true);
    sendRawCommand(frame);
}

// ============================================================
//  Query commands
//  All return the raw 8-bit response byte, or -1 on timeout
// ============================================================
int DaliController::queryStatus(int address)
{
    uint8_t frame[17];
    buildFrame(frame, (uint8_t)address, false, CMD_QUERY_STATUS, true);
    sendRawCommand(frame);
    return receiveResponse();
}

int DaliController::queryActualLevel(int address)
{
    uint8_t frame[17];
    buildFrame(frame, (uint8_t)address, false, CMD_QUERY_ACT_LVL, true);
    sendRawCommand(frame);
    return receiveResponse();
}

int DaliController::queryMaxLevel(int address)
{
    uint8_t frame[17];
    buildFrame(frame, (uint8_t)address, false, CMD_QUERY_MAX_LVL, true);
    sendRawCommand(frame);
    return receiveResponse();
}

int DaliController::queryMinLevel(int address)
{
    uint8_t frame[17];
    buildFrame(frame, (uint8_t)address, false, CMD_QUERY_MIN_LVL, true);
    sendRawCommand(frame);
    return receiveResponse();
}

int DaliController::queryDeviceType(int address)
{
    uint8_t frame[17];
    buildFrame(frame, (uint8_t)address, false, CMD_QUERY_DEV_TYPE, true);
    sendRawCommand(frame);
    return receiveResponse();
}

// ============================================================
//  queryGroups0_7()  /  queryGroups8_15()
//
//  Returns an 8-bit bitmask:
//    queryGroups0_7:   bit0=group0, bit1=group1, ..., bit7=group7
//    queryGroups8_15:  bit0=group8, bit1=group9, ..., bit7=group15
//  Returns -1 on timeout / no response.
// ============================================================
int DaliController::queryGroups0_7(int address)
{
    uint8_t frame[17];
    buildFrame(frame, (uint8_t)address, false, CMD_QUERY_GROUPS_0_7, true);
    sendRawCommand(frame);
    return receiveResponse();
}

int DaliController::queryGroups8_15(int address)
{
    uint8_t frame[17];
    buildFrame(frame, (uint8_t)address, false, CMD_QUERY_GROUPS_8_15, true);
    sendRawCommand(frame);
    return receiveResponse();
}

void DaliController::Wait(unsigned long ms) {
    unsigned long start = millis();
    while (millis() - start < ms) {
        yield();
    }
}

// ============================================================
//  isDevicePresent()
// ============================================================
bool DaliController::isDevicePresent(int address)
{
    return queryStatus(address) >= 0;
}

// ============================================================
//  Address Management
// ============================================================
bool DaliController::changeShortAddress(int oldAddress, int newAddress)
{
    // Validate inputs
    if (oldAddress < 0 || oldAddress > 63 || newAddress < 0 || newAddress > 63) {
        Serial.println(F("[DALI] Address out of range. Must be 0-63."));
        return false;
    }

    Serial.print(F("[DALI] Changing short address from "));
    Serial.print(oldAddress);
    Serial.print(F(" to "));
    Serial.println(newAddress);

    // Step 1: Set DTR0 with the encoded new address value
    uint8_t dtrValue = (uint8_t)((newAddress << 1) | 0x01);
    setDTR0(dtrValue);
    
    // Give the device a brief moment to process the DTR broadcast
    Wait(50); 

    // Step 2: Program Short Address from DTR (Command 128 / 0x80)
    // Send command to oldAddress. MUST be sent twice within 100ms.
    uint8_t frame[17];
    buildFrame(frame, (uint8_t)oldAddress, false, 0x80, true);
    
    sendRawCommand(frame);
    Wait(75);
    sendRawCommand(frame);
    
    // Wait for the address change to take effect in the hardware
    Wait(100);

    // Step 3: Verify the device now answers at the newAddress
    bool success = isDevicePresent(newAddress);

    if (success) {
        Serial.print(F("[DALI] Success! Device now responds at address "));
        Serial.println(newAddress);
    } else {
        Serial.println(F("[DALI] FAILED. Device did not respond at new address."));
    }

    return success;
}
