This is a project to make a zigbee controlled battery powered led candle.
The hardware is a nice!nano v2 (nRF52840) with Adafruit UF2 bootloader.
Flash by dropping .uf2 file onto the USB mass storage drive (double-tap RST pad to enter bootloader).

GPIO assignments (confirmed working):
- P1.11: candle LED (PWM, active high)
- P0.15: onboard blue status LED (active low, built-in)
- P0.31: pairing button (active low, pull-up)

Bootloader info:
- UF2 Bootloader 0.6.0, SoftDevice S140 v6.1.1
- App starts at 0x26000
- Family ID: 0xADA52840
