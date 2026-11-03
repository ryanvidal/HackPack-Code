# Space Oddity - CH32V003 Dongleless SWIO Flasher

Standalone recovery firmware for the Space Oddity ESP32-C3 that programs the onboard WCH CH32V003 coprocessor over its single-wire debug interface (`SWDIO`) using a single jumper wire and no external WCH-LinkE dongle.

## Symptoms of an Unflashed CH32V003

- The red power LED on the carrier PCB stays on continuously whenever power is connected, and holding the power button for 5+ seconds never turns it off.
- `StockCode` prints `begin` on Serial and hangs inside `carrierPCB.begin()` waiting for the `0xAE -> 0xAB` (`MSG_INIT -> MSG_READY`) UART handshake from the CH32V003.
- Probing the CH32V003 flash at `0x08000000` returns all `0xFF` (blank factory flash).

## Wiring (Single Jumper Wire)

1. Turn off the battery switch and unplug the USB-C cable for 2 seconds so the 3.3V rail fully discharges.
2. Connect a single jumper wire from **`IO7`** (or **`IO9`**) on the ESP32-C3 header to the **`SWDIO`** plated through-hole at the top-left corner of the carrier PCB (`J5`).
3. You do not need to jumper `GND` or `+3.3V` because the ESP32-C3 and carrier PCB already share ground and the 3.3V rail.

## Usage (OLED + Joystick Button)

1. Flash this project (`CH32V003_Flasher`) onto the ESP32-C3 via USB-C using PlatformIO (`pio run -t upload`).
2. On boot, the OLED display checks whether the CH32V003 responds on UART (`GPIO2`/`GPIO8`).
3. If there is no UART response, the OLED prompts you to connect `IO7 -> SWDIO` (or `IO9 -> SWDIO`) and **click the Joystick button**.
4. Clicking the Joystick button automatically scans `IO7` and `IO9`, unlocks and erases the CH32V003 flash, writes and verifies all `7612` bytes of the stock CH32V003 firmware with a live progress bar on the OLED, and reboots the CH32V003.
5. When the OLED displays `FLASH SUCCESS!`, remove the `SWDIO` jumper wire and press the **Power** button on the carrier PCB.
6. The OLED transitions to `CH32V003 VERIFIED OK!` and shows live status boxes (`PWR`, `B0`, `B1`, `B2`) so you can test all four carrier PCB buttons before flashing `StockCode` back onto the ESP32-C3.

## Optional Serial Monitor Commands (`115200` baud)

- `p` / `P`: Probe CH32V003 over SWIO on `IO7` (`p`) or `IO9` (`P`) and print Option Bytes, Chip UID, Flash size, and the first 64 bytes of Flash at `0x08000000`.
- `f` / `F`: Flash and verify the stock CH32V003 firmware over SWIO on `IO7` (`f`) or `IO9` (`F`).

