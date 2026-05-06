This is a project to make a zigbee controlled battery powered led candle.
The hardware is a nice!nano v2 (nRF52840) with Adafruit UF2 bootloader.

## GPIO assignments (confirmed working)
- P1.11: candle LED (PWM0 ch0, active low — LED connected VCC→GPIO, polarity inverted)
- P0.15: onboard blue status LED (active low, built-in)
- P0.31: pairing button (active low, pull-up)
- P0.29: battery voltage ADC input (AIN5, 2MΩ+1MΩ resistor divider, scales 4.2V→1.4V)

## Bootloader info
- UF2 Bootloader 0.6.0, SoftDevice S140 v6.1.1
- App starts at 0x26000
- Family ID: 0xADA52840

## Build
```bash
cd /home/vinter/claude1/nrf_siqbee_led_candle
rm -rf build
export ZEPHYR_BASE=~/ncs/zephyr
export ZEPHYR_SDK_INSTALL_DIR=/tmp/zephyr-sdk-0.16.8
~/.local/bin/west build -b "pro_micro/nrf52840" . -- -DBOARD_ROOT=/home/vinter/claude1
```

## Flash via ST-Link V2 + OpenOCD (direct, no bootloader)
```bash
openocd -f interface/stlink.cfg -f target/nrf52.cfg \
  -c "init; reset halt; \
      program /home/vinter/claude1/nrf_siqbee_led_candle/build/zephyr/zephyr.hex verify; \
      reset; exit"
```
Note: firmware flashed this way lands at 0x0 (CONFIG_FLASH_LOAD_OFFSET=0x0).
ST-Link wiring: VCC, GND, SWDIO (DO), SWDCLK (CLK).

## Flash via UF2 bootloader (double-tap RST to enter, drag .uf2 onto USB drive)
```bash
# Build produces build/zephyr/zephyr.hex — convert to UF2 if needed:
# uf2conv.py build/zephyr/zephyr.hex --family 0xADA52840 -o zephyr.uf2
```

## Recovery (if bricked / bootloader wiped)
```bash
# Erase chip first, then power cycle, then flash:
openocd -f interface/stlink.cfg -f target/nrf52.cfg \
  -c "init; reset halt; nrf5 mass_erase; exit"
# Power cycle the board, then run the flash command above.
```

## Zigbee pairing
- Button P0.31 short press (not joined): clears NVRAM credentials and reboots into fresh pairing
- Button P0.31 short press (joined): enters identify mode (ZHA binding)
- Button P0.31 hold 5s: factory reset
- LED blinks 2Hz until joined; switches to candle flicker after join
- Zigbee channel: 25, ZHA dimmable light profile (device type 0x0101)
- Reports as "DIY LED_Candle_v1" in Home Assistant ZHA

## Battery measurement
- ADC reads P0.29 every 5 minutes
- 2MΩ+1MΩ divider: V_bat = V_adc × 3
- LiPo range: 3.0V (empty) – 4.2V (full)
- Reported to ZHA via Power Configuration cluster (voltage in 100mV units, percentage 0–200)

## Sleepy End Device (SED) power saving
- zigbee_configure_sleepy_behavior(true) enabled
- 2s startup delay before Zigbee init to preserve UF2 bootloader double-tap detection
- Deep sleep (CONFIG_PM) left at default; do NOT enable CONFIG_PM=n in production
