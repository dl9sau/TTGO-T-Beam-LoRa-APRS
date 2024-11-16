# LoRa tracker with KISS TNC capability

Tracker can be used on its own. 
You can also connect it via blueetooth with APRSdroid.
After connection with APRX based DIGI it can be used as KISS-TNC

![diagram](https://github.com/SQ9MDD/TTGO-T-Beam-LoRa-APRS/blob/master/img/digi-schemat.png)

## Contributors
* Initial work: OE1ACM, OE3CJB
* Redesigned: SQ9MDD, DL9SAU, DL3EL
* KISS TNC Over Serial or Bluetooth: SQ5RWU
* Lora32 board support: DJ1AN
* This firmware also supports T-Beam V1.2 and and Heltec Wireless tracker: by DL9SAU
* Self-telemetry improvement: KB1GIM

## Supported boards
* TTGO T-beam v.0.7
* TTGO T-beam v.1.0 
* TTGO T-beam v.1.2
* Lora32 board
* Heltec Wireless Tracker

## User key functions:
### short press (OLED Enabled):
* if OLED is OFF a short press will wake it up and show current data (as "Display Timeout" seconds)
* if OLED is ON with GPS-FIX immediate sending of a frame with the position from the GPS
* if OLED is ON without GPS-FIX, immediate sending of the frame with the position saved in the configuration

### short press (OLED Disabled):
* with a valid GPS-FIX immediate sending of a frame with the position from the GPS
* without valid GPS-FIX, immediate sending of the frame with the position saved in the configuration

### long press while booting 
* reset to factory default

### long press: toggle on or off GPS power supply 
* if the "FIXED_BEACON_EN" option is enabled in the configuration, a beacon will be sent every set time interval

## Oled screens information
* ((TX)) - position frame sent automatically
* ((RX)) - informs about the received frame 
* ((GPSOFF)) - GPS power-off information
* ((GPS ON)) - GPS power-on information
* ((MAN TX)) - information about sending a manually initialized GPS position frame
* ((FIX TX)) - information about the forced manual sending of a frame with the position saved in the configuration when GPS is off or no fix
* ((AUT TX)) - information about sending automatic positioning frame when GPS is turned off
* ((KISSTX)) - information about sending the frame sent by KISS
* ((WEB TX)) - sending frame as requested via HTTP
* ((TEL TX)) - information about sending telemetry

## How to binary first flash readme... (thanx SP6VWX)
* Download the appropriate binary file for your board from github does not work.
  Self-compile is recommended. But you could download a latest stable / recommended release from
     http://db0fhn.hamnet.network/~dl9sau/lora-aprs-sq9mdd-dl9sau-dl3el-latest.zip
* Download current version of the esphome-flasher tool: https://github.com/esphome/esphome-flasher/releases
* Connect your board to your USB port and open ESPHome Flasher.
* If your board is not showing under Serial Port then you likely need to install the drivers for the CP210X serial chip. In Windows you can check by searching “Device Manager” and ensuring the device is shown under “Ports”.
* In ESPHome Flasher, refresh the serial ports and select your board's serial port.
* Browse to the downloaded firmware and select the correct firmware based on the board type.
* Select/Press Flash ESP.
* Once complete, “Done! Flashing is complete!” will be shown.
* any subsequent updates can be done via WWW

## How to flash with esptool the first time
If you won't compile with platformio and like to use esptool(.py),
a user succeeded this way with his new heltec wireless tracker:
* python -m esptool --chip esp32s3 --port /dev/ttyACM0 --baud 460800 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size 8MB 0x0000 bootloader.bin 0x8000 partitions.bin 0xe000 boot_app0.bin 0x10000 heltec-wireless-tracker-firmware.bin
  Got this by adding -v to pio run: ~/.platformio/penv/bin/pio run -v -e heltec-wireless-tracker -t upload --upload-port /dev/null
   I provided him the appropriete bootloader.bin and partitions.bin, .platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin
* On other platforms, speed, flash-size and hex-addresses may differ.

## Installing from source
* good idea, not only from advanced users
* Clone Git repo to any folder of your PC
* If you prefer the platormio-cli (-> compile in your shell; no VCS is needed!),
  please read through this easy documentation:
    https://docs.platformio.org/en/latest/core/index.html
* Else:
* Download and install "Visual Studio Code"
* Open "Visual Studio Code" and add "PlatformIO" plugin
* Open "PlatformIO" and click on "Import Arduino Project"
* In the top part of the window choose youy board then browse to cloned repo and click "import"
* In the left column click on the ANT-shaped icon, choose your board and click on "Upload". COM port should be detected automatically Wait for procedure to finish and keep reading

## Configuring parameters
Wait for the board to reboot, connect to "N0CALL AP" WiFi network, password is: xxxxxxxxxx (10 times "x") and point your browser to "http://192.168.4.1" (http, not http*s*). Hover your mouse to textboxes to get useful hints.

### WiFi Settings
you can scan for local SSID or manually type in name and password
* Scan WiFi: scan for local WiFi networks
* SSID: name of the AP to connect to
* Password: password of WiFi AP
* AUTO AP Password: if configured network is not reachable the AP mode will be enabled, SSID will be your callsign and this will be the password

###  APRS Settings
These are main APRS settings such as callsign, SSID and symbol (refer to: http://www.aprs.org/symbols.html). Please remember to turn ON GPS in order to use it as a tracker.
* Frequency: frequency of receive and transmit
* Speed: speed of the LoRa APRS packet (default: 300)
* Callsign and SSID: Callsign of the tracker (e.g: N0CALL-10)
* Relay PATH: destination path of APRS packet
* Symbol table & Symbol: custom APRS icon
* Show altitude: allow altitude to be reported
* Show comment: includes comment in APRS packet
* Comment: custom message to send
* Show Battery: includes current battery voltage
* Fixed beacon: enables static beacon transmission
* Fixed beacon interval: beacon interval in seconds (please more than 30 minutes to avoid overflow on the network)
* Latitude & Longitude: fixed beacon position to transmit (in APRS format)
* Min interval: packet are transmitted not less than x second apart
* Max interval: maximum interval between packages
* Speed and course: variables to calculate smart beaconing
* GPS enabled: enables power to GPS module

### Device Settings
These are main device settings, hover the mouse on the checkboxes and explainations will appear.
* OLED Display enabled: Enables OLED functionalities
* Bluetooth enabled: Enables bluetooth functions (such as TNC via APRSDROID)
* LED Signaling: not yet implemented
* Auto Power OFF: activate auto power off when USB is unplugged (only for TBEAM 1.0 or later)
* Auto Power OFF delay: timer to turn off board after USB is unplugged (only if enabled)
* Display show RX Time: when a packet is received display the packet is shown for X seconds
* Display Timeout: display will turn OFF after X seconds for better power save (0 to disable and keep OLED ON)

### Received
Here is the list of recently received stations with some details

### Actions
Some shortcuts to useful functions such as manually send beacon

### Update
This box allows firmware to be uploaded via OTA

## Default WiFi Password:
* Default WiFi Password is: xxxxxxxxxx
* that is, 10 times x
