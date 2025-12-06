# DMX to ARTNET converter 
DMX to Artnet converter, to allow older non-networked Lighting consoles to be visalised for pre-programming using Artnet.

# Backstory
I bought a Avolites Pearl 2000 for very cheap, but commercial Artnet nodes with server mode were too expensive.
Most of them lacked support to convert existing DMX to artnet, only the other way around because newer consoles can be networked and output pure Artnet.

# Libraries used in ARDUINO IDE
- ESP32 Arduino core 2.0.17
- esp_dmx 4.1.0
- RStephan's "ArtnetWifi"
    GitHub:
    https://github.com/rstephan/ArtnetWifi
- ESPAsyncWebServer
    GitHub:
    https://github.com/me-no-dev/ESPAsyncWebServer
- AsyncTCP
    GitHub:
    https://github.com/me-no-dev/AsyncTCP
- WiFi.h
    Builtin library
    Part of the ESP32 Arduino core.
No installation required.

- Preferences.h
    Builtin library
    Used to save config (SSID, universes, direction, etc.) in NVS.
- WifiUdp.h
    Builtin library
    Used for manually sending Art-Net packets as UDP datagrams.

# Default Wi-Fi APs:
Bridge A: SSID DMX-Bridge-A, password dmx12345
Bridge B: SSID DMX-Bridge-B, password dmx12345

# GPIO Wiring and functiona between boards
GPIO - Role - Connected To - Notes

     17  - DMX1 TX - MAX485 #1 DI
     16  - DMX1 RX - MAX485 #1 RO
     21  - DMX1 EN - MAX485 #1 DE+RE DMX1 driver enable
     32  - DMX1 Termination (optional) Termination circuit Controls 120Ω between A/B

     25  - DMX2 TX - MAX485 #2 DI
     26  - DMX2 RX - MAX485 #2 RO
     22  - DMX2 EN - MAX485 #2 DE+RE DMX2 driver enable

     33  - DMX2 Termination (optional) Termination circuit Controls 120Ω between A/B

# ESP32 Pinout for DMX Bridge
TX and RX lines connect to DI and RO on each MAX485. EN is tied to both DE and RE; setting EN high
enables the transceiver, while low disables it.

# DMX512 XLR Pinout for a Avolites Pearl 2000
The DMX512 standard typically uses 3-pin or 5-pin XLR connectors. This bridge assumes standard
DMX polarity: Pin 3 is DMX+, Pin 2 is DMX−, Pin 1 is shield/ground.
Pin 1 – GND 
Pin 2 – DMX1− (B)
Pin 3 – DMX1+ (A)
Pin 4 - DMX2- (B)
Pin 5 - DMX2+ (A)
On the MAX485 side, A and B differential outputs connect directly to the DMX line. A is DMX+ (Pin 3),
B is DMX− (Pin 2), and the transceiver ground must be tied to the XLR shield (Pin 1).


# Difference between sketch Bridge A vs Bridge B
Bridge A
• Default Art-Net universes: DMX1 = 0, DMX2 = 1
• Access Point SSID: DMX-Bridge-A
• Default password: dmx12345
• DMX → Art-Net by default (both ports). Direction can be changed in the web UI.

Bridge B
• Default Art-Net universes: DMX1 = 2, DMX2 = 3
• Access Point SSID: DMX-Bridge-B
• Default password: dmx12345
• DMX → Art-Net by default (both ports). Direction can be changed in the web UI.

Both bridges expose a configuration web page where you can adjust: 
• Wi-Fi SSID and password (for STA mode) 
• Broadcast IP for Art-Net 
• Per-port universe number 
• Direction: DMX→Art-Net or Art-Net→DMX 
• Refresh rate for DMX→Art-Net 
• Line termination (on/off) 
• Polarity inversion (normal/inverted)

# DMX & Art-Net Data Integrity
To avoid errors in Art-Net monitoring tools (such as Artnetominator), the firmware enforces the following
rules when sending ArtDMX packets:
• DMX frame length is clamped to the range 2–512 bytes.
• Length is forced to be even, as required by the Art-Net specification.
• Any empty or malformed DMX frames are discarded before sending.
These safeguards prevent malformed Art-Net packets that could cause errors such as "shift and length are outside
the limit of the matrix" in analysis software.

# Quick Wiring Checklist
    - ESP32 and MAX485 share a common ground.
    - MAX485 VCC is powered from a stable 5V supply.
    - DE and RE pins on each MAX485 are tied together and connect to the appropriate EN GPIO.
    - TX/RX pins match DI/RO on each MAX485 (TX → DI, RX ← RO).
    - DMX A/B outputs from MAX485 go to the correct XLR pins (A → Pin 3, B → Pin 2).
    - At least one device on each DMX line has a 120Ω termination resistor; the bridge’s internal termination can be toggled in software.

# Common faults after wiring:
If the ESP32 continuously resets, double-check that no boot-strapping pins (e.g. GPIO0, GPIO2, GPIO4, GPIO12,
GPIO15) are used as enable lines and that external hardware is not forcing them high/low during boot