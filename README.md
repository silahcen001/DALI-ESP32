# DaliController

[![Platform: ESP32](https://img.shields.io/badge/Platform-ESP32-brightgreen.svg)](https://www.espressif.com/en/products/socs/esp32)
[![PlatformIO Compatible](https://img.shields.io/badge/PlatformIO-Compatible-orange.svg)](https://platformio.org/)

A library for controlling DALI (Digital Addressable Lighting Interface, IEC 62386) devices from an ESP32, built entirely on the ESP32's hardware **RMT (Remote Control) peripheral**.

---

## Why Hardware RMT?

Most DALI libraries for Arduino rely on software timers or CPU-busy loops to generate the precise Manchester-encoded bit timing that the DALI protocol requires. This approach is fragile: any interrupt or task preemption during transmission can corrupt the signal.

**DaliController uses the ESP32's dedicated RMT peripheral instead.** The RMT hardware generates and receives Manchester-encoded pulses with microsecond accuracy, completely independent of the CPU. Once a frame is queued, the processor is free to do other work — no busy waiting, no timing jitter, no corrupted frames under load.

---

## Compatibility

>  **ESP32 family only !!**
>
> This library uses the ESP32 RMT peripheral via the ESP-IDF `driver/rmt.h` API and is **not** compatible with AVR (Uno, Mega), SAMD, STM32, or other architectures.

Tested on:

- ESP32 (original, 38-pin and 30-pin modules)
- ESP32-S2 *(RMT channel numbers may differ — verify pin assignments)*
- ESP32-WROOM-32

---

## Installation

### PlatformIO

Add the library to your `platformio.ini`:

```ini
[env:esp32dev]
platform  = espressif32
board     = esp32dev
framework = arduino
lib_deps  = https://github.com/silahcen001/DaliController.git
```

### Arduino IDE

1. Download this repository as a ZIP file (**Code → Download ZIP**).
2. In the Arduino IDE, go to **Sketch → Include Library → Add .ZIP Library…**
3. Select the downloaded ZIP file.
4. The library will appear under **Sketch → Include Library → DaliController**.

---

## Wiring

A DALI bus operates at 16 V and uses a current-limited two-wire interface — it cannot be connected directly to ESP32 GPIO pins.A DALI transceiver module or shield is required.

**Below is the verified and tested schematic used alongside this library:**

<img width="706" height="349" alt="image" src="https://github.com/user-attachments/assets/9b96dce9-1539-49be-bd48-8163c5342da5" />



```
ESP32 GPIO (TX_PIN)  →  DALI Transceiver TX input
ESP32 GPIO (RX_PIN)  ←  DALI Transceiver RX output
ESP32 GND            →  DALI Transceiver GND
3.3 V or 5 V         →  DALI Transceiver VCC (check your module datasheet)
```

The TX and RX GPIO pins are fully configurable in `dali.begin()`. The defaults used in the examples are GPIO 2 (TX) and GPIO 15 (RX), but any suitable GPIO pair can be used.

---

## Quick Start

```cpp
#include "DaliController.h"

#define DALI_TX_PIN  2
#define DALI_RX_PIN  15

DaliController dali;

void setup() {
    Serial.begin(115200);

    // Initialise the RMT TX and RX channels
    dali.begin(DALI_TX_PIN, DALI_RX_PIN);

    // Turn on the fixture at short address 5
    dali.turnOnAddress(5);

    // Dim it to 60 %
    dali.setBrightness(5, 60);
}

void loop() { }
```

---

## API Reference

### Initialisation

| Method | Description |
|--------|-------------|
| `begin(tx_pin, rx_pin)` | Initialise RMT TX + RX. Call once in `setup()`. Returns `true` on success. |
| `begin(tx_pin, rx_pin, tx_ch, rx_ch)` | Same, with explicit RMT channel selection. |

---

### On / Off

| Method | Description |
|--------|-------------|
| `turnOnAddress(address)` | Recall max level at a short address (0–63). |
| `turnOnGroup(group)` | Recall max level for a group (0–15). |
| `turnOnAll()` | Recall max level — broadcast to all devices. |
| `turnOffAddress(address)` | Turn off a short address. |
| `turnOffGroup(group)` | Turn off a group. |
| `turnOffAll()` | Turn off all devices (broadcast). |

---

### Brightness (DAPC)

Brightness values are expressed as a **percentage (0–100)** and mapped to the DALI arc-power range internally.

| Method | Description |
|--------|-------------|
| `setBrightness(address, percent)` | Set brightness % at a short address. |
| `setGroupBrightness(group, percent)` | Set brightness % for a group. |
| `setAllBrightness(percent)` | Set brightness % for all devices (broadcast). |

---

### Step / Recall

| Method | Description |
|--------|-------------|
| `stepUpAddress(address)` | Step up one level at address. |
| `stepDownAddress(address)` | Step down one level at address. |
| `stepUpGroup(group)` | Step up one level for a group. |
| `stepDownGroup(group)` | Step down one level for a group. |
| `recallMaxLevelAddress(address)` | Recall max level at address. |
| `recallMinLevelAddress(address)` | Recall min level at address. |
| `recallMaxLevelGroup(group)` | Recall max level for a group. |
| `recallMinLevelGroup(group)` | Recall min level for a group. |

---

### Scenes

| Method | Description |
|--------|-------------|
| `setScene(address, scene, level)` | Store a level into scene `scene` (0–15) at address. |
| `recallSceneAddress(address, scene)` | Go to scene at address. |
| `recallSceneGroup(group, scene)` | Go to scene for a group. |

---

### Group Membership

| Method | Description |
|--------|-------------|
| `addDeviceToGroup(address, group)` | Add address to group. |
| `removeDeviceFromGroup(address, group)` | Remove address from group. |

---

### Fade

| Method | Description |
|--------|-------------|
| `setFadeTime(address, fadeTime)` | Set fade time at address (DALI fade time code, 0–15). |
| `setFadeRate(address, fadeRate)` | Set fade rate at address (DALI fade rate code, 1–15). |

---

### Address Management

| Method | Description |
|--------|-------------|
| `changeShortAddress(oldAddress, newAddress)` | Reassigns a DALI device from `oldAddress` to `newAddress`. Returns `true` if successful. |

>  **!! CRITICAL WARNING: Changing Addresses !!**
> 
> The `changeShortAddress` function relies on DTR broadcast commands. To prevent accidentally overwriting the addresses of other devices on the network, **you must disconnect all other ballasts from the DALI bus** and leave ONLY the single ballast you wish to reprogram connected.

---

### Reset / Store

| Method | Description |
|--------|-------------|
| `resetAddress(address)` | Reset device at address to factory defaults. |
| `storeActualLevelInDTR(address)` | Store the current arc-power level into DTR0. |

---

### Query Commands

All query methods return the raw 8-bit response byte (0–255), or **`-1`** on timeout / no response.

| Method | Returns |
|--------|---------|
| `queryStatus(address)` | Status byte (IEC 62386 Table 15). |
| `queryActualLevel(address)` | Current arc-power level (0–254). |
| `queryMaxLevel(address)` | Configured maximum level. |
| `queryMinLevel(address)` | Configured minimum level. |
| `queryDeviceType(address)` | Device type (e.g. 0 = fluorescent, 8 = LED). |
| `queryGroups0_7(address)` | Group membership bitmask for groups 0–7. |
| `queryGroups8_15(address)` | Group membership bitmask for groups 8–15. |
| `isDevicePresent(address)` | Returns `true` if the device responded to Query Status. |

**Reading group membership:**

```cpp
int mask = dali.queryGroups0_7(5);
if (mask >= 0) {
    for (int g = 0; g < 8; g++) {
        if (mask & (1 << g)) {
            Serial.print("Member of group "); Serial.println(g);
        }
    }
}
```

---

### Memory Bank Access

| Method | Returns |
|--------|---------|
| `readMemoryBank(device_address, bank, address)` | Raw byte from memory bank, or `0xFF` on error. |

Supported banks: 0, 1, 202, 204, 205, 206.

---

### DTR Helpers

| Method | Description |
|--------|-------------|
| `setDTR0(value)` | Broadcast Set DTR0 to value. |
| `setDTR1(value)` | Broadcast Set DTR1 to value. |

---

## Examples

| Sketch | Description |
|--------|-------------|
| `blink_led` | Repeatedly turns a single address on and off. |
| `ScanAddresses` | Scans all 64 short addresses and reports which devices are present. |
| `SerialBrightnessControl` | Controls brightness and on/off state from the Serial Monitor. |

---

## DALI Protocol Notes

- **Configuration commands** (reset, set scene, add/remove group, set fade) must be sent **twice within 100 ms** per IEC 62386. This library handles the double-send automatically.
- **Query commands** require a 22-bit backward frame response from the device. The RMT RX channel decodes this automatically with a 20 ms timeout.
- The DALI bus supports a maximum of **64 short addresses** (0–63) and **16 groups** (0–15).
- **Broadcast** commands address all devices simultaneously.

---

## License

This project is licensed under the **GNU General Public License v3.0**. See [LICENSE](LICENSE) for details.
