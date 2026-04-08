#ifndef DALI_CONTROLLER_H
#define DALI_CONTROLLER_H

// ============================================================
//  Clean C++ DALI (IEC 62386) library for ESP32 RMT.
//  All query/read functions return raw values.
// ============================================================

#include <Arduino.h>
#include <driver/rmt.h>

class DaliController {
public:

    DaliController();

    /**
     * Initialise RMT TX and RX channels.
     * Call once from setup() before any DALI operations.
     *
     * @param tx_pin  GPIO -> DALI bus driver input
     * @param rx_pin  GPIO <- DALI bus receiver output
     * @param tx_ch   RMT TX channel (default RMT_CHANNEL_0)
     * @param rx_ch   RMT RX channel (default RMT_CHANNEL_4)
     * @return true on success
     */
    bool begin(int tx_pin, int rx_pin,
               rmt_channel_t tx_ch = RMT_CHANNEL_0,
               rmt_channel_t rx_ch = RMT_CHANNEL_4);

    // --------------------------------------------------------
    //  Basic on / off
    // --------------------------------------------------------
    void turnOnAddress(int address);
    void turnOnGroup(int group);
    void turnOnAll();

    void turnOffAddress(int address);
    void turnOffGroup(int group);
    void turnOffAll();

    // --------------------------------------------------------
    //  Brightness (DAPC)
    // --------------------------------------------------------
    void setBrightness(uint8_t address, uint8_t percent);
    void setGroupBrightness(uint8_t group, uint8_t percent);
    void setAllBrightness(uint8_t percent);

    // --------------------------------------------------------
    //  Step / Recall
    // --------------------------------------------------------
    void stepUpAddress(int address);
    void stepDownAddress(int address);
    void stepUpGroup(int group);
    void stepDownGroup(int group);

    void recallMaxLevelAddress(int address);
    void recallMinLevelAddress(int address);
    void recallMaxLevelGroup(int group);
    void recallMinLevelGroup(int group);

    // --------------------------------------------------------
    //  Scenes
    // --------------------------------------------------------
    void setScene(int address, uint8_t scene, uint8_t level);
    void recallSceneAddress(int address, uint8_t scene);
    void recallSceneGroup(int group, uint8_t scene);

    // --------------------------------------------------------
    //  Group membership
    // --------------------------------------------------------
    void addDeviceToGroup(int address, int group);
    void removeDeviceFromGroup(int address, int group);

    // --------------------------------------------------------
    //  Fade
    // --------------------------------------------------------
    void setFadeTime(int address, uint8_t fadeTime);
    void setFadeRate(int address, uint8_t fadeRate);

    // --------------------------------------------------------
    //  Reset / store
    // --------------------------------------------------------
    void resetAddress(int address);
    void storeActualLevelInDTR(int address);

    // --------------------------------------------------------
    //  Query commands
    //  Return raw byte (0-255), or -1 on timeout / no response
    // --------------------------------------------------------
    int queryStatus(int address);
    int queryActualLevel(int address);
    int queryMaxLevel(int address);
    int queryMinLevel(int address);
    int queryDeviceType(int address);

    /**
     * Query group membership bitmasks.
     * Each bit in the returned byte corresponds to one group.
     * Returns -1 on timeout / no response.
     *
     * queryGroups0_7:  bit0=group0, bit1=group1, ..., bit7=group7
     * queryGroups8_15: bit0=group8, bit1=group9, ..., bit7=group15
     */
    int queryGroups0_7(int address);
    int queryGroups8_15(int address);

    /** Returns true if the device responded to a Query Status. */
    bool isDevicePresent(int address);

    // --------------------------------------------------------
    //  Memory bank access
    //  Returns 0xFF on error or timeout
    // --------------------------------------------------------
    uint8_t readMemoryBank(uint8_t device_address,
                           uint8_t bank,
                           uint8_t address);

    // --------------------------------------------------------
    //  DTR helpers (public - needed by application layer too)
    // --------------------------------------------------------
    void setDTR0(uint8_t value);
    void setDTR1(uint8_t value);

private:
    // --------------------------------------------------------
    //  RMT timing (1 tick = 1 us at 80 MHz / CLK_DIV=80)
    // --------------------------------------------------------
    static const uint32_t CLK_DIV         = 80;
    static const uint32_t HALF_BIT_US     = 417;
    static const uint32_t STOP_US         = 1333;
    static const uint32_t RX_IDLE_THRESH  = 2000;
    static const uint32_t RX_TIMEOUT_MS   = 20;
    static const size_t   MAX_RMT_SYMBOLS = 64;
    static const int      NUM_BITS        = 17;

    // --------------------------------------------------------
    //  RMT state
    // --------------------------------------------------------
    rmt_channel_t   _tx_ch;
    rmt_channel_t   _rx_ch;
    RingbufHandle_t _rx_ringbuf;
    bool            _ready;

    // Manchester decode scratch-pad
    int           _receivedBits[9];
    unsigned long _transitionTimes[20];
    int           _transitionValues[20];
    int           _transitionCount;

    // --------------------------------------------------------
    //  Core TX / RX primitives
    // --------------------------------------------------------
    void sendRawCommand(const uint8_t command[17]);
    int  receiveResponse();   // returns decoded byte 0-255, or -1

    // --------------------------------------------------------
    //  Internal helpers
    // --------------------------------------------------------
    bool manchesterDecode();
    void readMemoryLocation(uint8_t device_address);
    bool validateDTR0Range(uint8_t bank, uint8_t address) const;

    /**
     * Build a standard 17-bit DALI frame into frame[17].
     *
     *  frame[0]     = 1  (start bit, always)
     *  frame[1]     = 0 address mode / 1 group+broadcast mode
     *  frame[2..7]  = 6-bit address or group field (MSB first)
     *  frame[8]     = 0 DAPC / 1 command selector
     *  frame[9..16] = 8-bit command byte or arc-power level (MSB first)
     */
    static void buildFrame(uint8_t frame[17],
                           uint8_t addrField,
                           bool    isGroup,
                           uint8_t cmdByte,
                           bool    isCommand);
};

#endif // DALI_CONTROLLER_H
