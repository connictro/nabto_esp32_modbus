## Nabto ESP32 Modbus gateway

## Brief description:
 Uses the dual-role FreeModbus port by Connictro GmbH (based on FreeModbus, and armink's / erfengwe's Master ports).
 The demo allows to control Modbus slave devices from an Espressif ESP32 running in master mode via mobile phone app.
 The example also includes Wifi Setup via SoftAP - the user can connect to ESP32 access point, select Wifi SSID and enter password, which will be stored in NVS.
 AP mode has been a bit simplified, only one wifi network can be stored. 
 Reset of the stored credentials is possible by starting the ESP32 out of range of the selected
 Wifi 5 times (or change the password that it won't connect anymore). Resetting therefore can take up to 5 minutes.

### Install the project:
1. Make sure the preconditions on your system for ESP32 esp-idf are met, see also https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html
2. Clone the repository nabto_esp32_modbus by git clone https://github.com/connictro/nabto_esp32_modbus
3. In components/unabto set the branch to version 4.2 (newer branches not yet supported by unabto-esp-idf and modbus application yet)
   cd components/unabto; git checkout 4.2; cd ../..
7. (You can do this while compiling the device below) Download the Nabto Modbus demo app from: <t.b.d.> and install on your mobile phone.
   It is available for Android and iOS.


## Preconditions
* 1. Additionally to the ESP32, you'll need some Modbus devices and an 3.3V-to-RS485 level shifter (preferably automatic mode who don't need the
     RTS signal, just TxD and RxD). 
     Example: https://www.ebay.de/itm/TTL-RS485-Adapter-485-UART-Seriell-3-3V-5-Volt-Level-Konverter-Modul-Arduino/252852473229?hash=item3adf2e8d8d:g:DQwAAOSw5UFb4H25:rk:2:pf:1&frcectupt=true 
     Other level shifters should be fine as well and the RTS signal is supported if needed.
* 2. You need access to a HTTPS server on the internet (or local resource), configure a path to device-specific config files for the next step.
* 3. Obtain a Nabto ID and key from Nabto's www.appmyproduct.com website.
     Store them in a file corresponding to the path you'll use on the device, format must be in one line like:
     aaaaaaaa.bbbbb.appmyproduct.com;01234567890123456789012345678901
     Name of the configuration file must be: modbus-gw-<MAC Address of device>.
* 4. Create a .JSON file matching your modbus slave configuration, store it as "modbus-gw-<MAC Address of device>_modbus_config.json"
     in the same path as the Nabto credentials file.
     Example: This configuration comprises of 2 devices, a temperature/humidity sensor on slave ID 2 and a 4-port relay card on slave ID 3.
{"r":[{"slaveID":"2","n":"Temperature","t":"n","r":"0000", "f":"r/10"},{"slaveID":"2","n":"Humidity","t":"n","r":"0001","f":"r/10"},
{"slaveID":"3","n":"AlfredoSwitch","fc":"05","r":"0000","o":"0100","c":"0000"},
{"slaveID":"3","n":"MexicanEggSwitch","fc":"05","r":"0001","o":"0100","c":"0000"},
{"slaveID":"3","n":"ScentedLambSwitch","fc":"05","r":"0002","o":"0100","c":"0000"},
{"slaveID":"3","n":"WalnutSwitch","fc":"05","r":"0003","o":"0100","c":"0000"}
]}

We used the following devices:
- Temperature/Humidity sensor: https://www.ebay.de/itm/SHT20-Temperature-Humidity-Sensor-Modbus-RS485-High-Precision-Transmitter/173582921504?hash=item286a592f20:g:vGwAAOSwElxbvveq:rk:3:pf:1&frcectupt=true
- Relay card:                  https://www.ebay.de/itm/Modbus-RTU-4-Way-Relaismodul-DIY-STM8S103-System-4-Road-Input-485-Communication/132951586179?hash=item1ef4883583:g:VuYAAOSw4UtWRvlM:rk:2:pf:1&frcectupt=true

If you use different devices, please adapt the .JSON file above accordingly.


## Configure the project

``
make menuconfig
```

* Set Modbus options under Components-->Master/Slave Modbus configuration
  (NOTE: Modbus configuration is the original Freemodbus configuration which supports slave mode only and does not interface with Nabto).
  Typically serial port options and baudrate can be changed here (in the future it will be made configurable by configuration file instead).
* Set "Wifi and Nabto Configurations"
  Retry and Timeout are related to Wifi configuration - after the configured amount of unsuccessful retries the Wifi credentials
  (SSID/Password) will be erased and the ESP32 enters Access point mode again.
  URL prefix is the path on a secure server where the Nabto credentials are stored.
  NOTE: If the server is changed or the server certificate is updated, also the root certificate (in main/keyserver_root_cert.pem) needs to be
  changed and the device software rebuilt. Please look into nabto_cred_config.c for instructions on how to update this file.
  
## Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
make -j4 flash monitor
```

(To exit the serial monitor, type ``Ctrl-]``.)


## How to use the example:
* 1. To connect to the gateway, start it and connect to the access point named "nabto-modbus-gw-<MAC address>". 
*    The password is "modbusdemo".
* 2. Select your preferred wifi network and enter its credentials. If this is successful, the ESP32 will
*    store the credentials and reboot, it will now connect to the wifi and is ready to accept commands
*    from the Nabto app in a couple of seconds.
*    Behind the scenes, it will download the two config files (Nabto credentials, Modbus JSON), and store them in NVS,
*    so they don't need to be downloaded again on subsequent starts.
* 3. The application is supposed to send Modbus requests in JSON format down to the device.
*    There are 2 options - raw and list-formatted. See also unabto_queries.xml for details.
*    Raw Format (query 20000) is:
     <bus>, <slave address>, <raw modbus frame w/o CRC>
*    List Format (query 20002) uses the schema:
     <bus>, <slave address> <function code>, <read start address>, <read length>, <write start address>, <write length>, [data ...]
     - Bus, Slave address, and function code are mandatory. Bus is currently ignored (only one bus on UART2 supported).
     - Read start address and read length have to be supplied for all read function codes 
       (0x01 - read coils, 0x02 - read discrete inputs, 0x03 - read holding registers, 0x04 - read input registers and 0x17 - read+write holding registers).
     - Write start address have to be supplied for all write function codes:
       (0x05: write single coil, 0x0f: write multiple coils, 0x06: write single holding register, 0x10: write multiple holding registers and 0x17 - read+write holding registers).
     - Data is to be supplied for all write commands, up to 125 maximum (which will be the limit of a modbus RTU frame).
       Example for a single coil write ON to slave 24, address 3 send:  1, 24, 5, 0, 0, 3, 1, 1
       Example for a single holding register write (value 0xDE) to address 0xAD on slave 24 send: 1, 24, 6, 0, 0, 0xAD, 1, 0xDE
       Example to read 7 holding registers on address 0 from slave 23: 1, 23, 3, 0, 7, 0, 0
*    Response for raw requests is:
     <bus>, <slave address>, <result code>, <raw data>, <query status>
*    Response for list requests is:
     <bus>, <slave address>, <result code>, <data list of type uint16>, <query status>
     where data is only returned for read commands, or the combined read/write function code 0x17. So data can be empty, and if data is returned it will be 
     the amount o elements requested in the according read command.

## - Thanks to:
- armink for the STM32 port               - see https://github.com/armink/FreeModbus_Slave-Master-RTT-STM32
- erfengwe for the ESP32 port             - see https://github.com/erfengwelink/modbus_port_esp32
- Tony Pottier for the ESP32 Wifi Manager - see https://github.com/tonyp7/esp32-wifi-manager
- Nabto for the IoT platform              - see https://github.com/nabto

## - Bugs
Please report via github or to info@connictro.de .

## Example Output (credentials intentionally removed or obfuscated):

lchain path: /home/connictro/ESP32/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc
Toolchain version: crosstool-ng-1.22.0-80-g6c4433a
Compiler version: 5.2.0
Python requirements from /home/connictro/ESP32/esp-idf/requirements.txt are satisfied.
--- forcing DTR inactive
--- forcing RTS inactive
--- Miniterm on /dev/ttyUSB0  115200,8,N,1 ---
--- Quit: Ctrl+] | Menu: Ctrl+T | Help: Ctrl+T followed by Ctrl+H ---
ets Jun  8 2016 00:22:57

rst:0x1 (POWERON_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)
configsip: 0, SPIWP:0xee
clk_drv:0x00,q_drv:0x00,d_drv:0x00,cs0_drv:0x00,hd_drv:0x00,wp_drv:0x00
mode:DIO, clock div:2
load:0x3fff0018,len:4
load:0x3fff001c,len:6264
load:0x40078000,len:11164
load:0x40080400,len:6672
entry 0x40080764
I (29) boot: ESP-IDF v3.3-beta1-328-gabea9e4-dirty 2nd stage bootloader
I (29) boot: compile time 13:22:26
I (30) boot: Enabling RNG early entropy source...
I (35) boot: SPI Speed      : 40MHz
I (40) boot: SPI Mode       : DIO
I (44) boot: SPI Flash Size : 4MB
I (48) boot: Partition Table:
I (51) boot: ## Label            Usage          Type ST Offset   Length
I (58) boot:  0 nvs              WiFi data        01 02 00009000 00006000
I (66) boot:  1 phy_init         RF data          01 01 0000f000 00001000
I (73) boot:  2 factory          factory app      00 00 00010000 00100000
I (81) boot: End of partition table
I (85) esp_image: segment 0: paddr=0x00010020 vaddr=0x3f400020 size=0x32b78 (207736) map
I (167) esp_image: segment 1: paddr=0x00042ba0 vaddr=0x3ffb0000 size=0x02eec ( 12012) load
I (172) esp_image: segment 2: paddr=0x00045a94 vaddr=0x40080000 size=0x00400 (  1024) load
I (174) esp_image: segment 3: paddr=0x00045e9c vaddr=0x40080400 size=0x0a174 ( 41332) load
I (200) esp_image: segment 4: paddr=0x00050018 vaddr=0x400d0018 size=0x9c584 (640388) map
I (424) esp_image: segment 5: paddr=0x000ec5a4 vaddr=0x4008a574 size=0x07858 ( 30808) load
I (448) boot: Loaded app from partition at offset 0x10000
I (448) boot: Disabling RNG early entropy source...
I (448) cpu_start: Pro cpu up.
I (452) cpu_start: Application information:
I (457) cpu_start: Project name:     nabto_esp32_modbus_gateway
I (463) cpu_start: App version:      1
I (468) cpu_start: Compile time:     13:22:29
I (473) cpu_start: Compile date:     Mar  1 2019
I (478) cpu_start: ESP-IDF:          v3.3-beta1-328-gabea9e4-dirty
I (485) cpu_start: Starting app cpu, entry point is 0x400810b4
I (0) cpu_start: App cpu up.
I (495) heap_init: Initializing. RAM available for dynamic allocation:
I (502) heap_init: At 3FFAE6E0 len 00001920 (6 KiB): DRAM
I (508) heap_init: At 3FFBCAB0 len 00023550 (141 KiB): DRAM
I (514) heap_init: At 3FFE0440 len 00003AE0 (14 KiB): D/IRAM
I (521) heap_init: At 3FFE4350 len 0001BCB0 (111 KiB): D/IRAM
I (527) heap_init: At 40091DCC len 0000E234 (56 KiB): IRAM
I (533) cpu_start: Pro cpu start user code
I (216) cpu_start: Starting scheduler on PRO CPU.
I (0) cpu_start: Starting scheduler on APP CPU.
I (217) MB_MAIN: [APP] Startup..
I (217) MB_MAIN: [APP] Free memory: 258520 bytes
I (217) MB_MAIN: [APP] IDF version: v3.3-beta1-328-gabea9e4-dirty
I (267) MB_MAIN: ESP SSID prefix: nabto-modbus-gw-30aea4372168
I (267) APP_WIFI: open NVS storage
I (267) APP_WIFI: trying to read credentials from NVS
I (277) APP_WIFI: SSID Connictro_GmbH read from NVS
I (277) APP_WIFI: Password ****** read from NVS
I (297) wifi: wifi driver task: 3ffc48cc, prio:23, stack:3584, core=0
I (297) wifi: wifi firmware version: 939cd60
I (297) wifi: config NVS flash: enabled
I (297) wifi: config nano formating: disabled
I (307) system_api: Base MAC address is not set, read default base MAC address from BLK0 of EFUSE
I (317) system_api: Base MAC address is not set, read default base MAC address from BLK0 of EFUSE
I (347) wifi: Init dynamic tx buffer num: 32
I (347) wifi: Init data frame dynamic rx buffer num: 32
I (347) wifi: Init management frame dynamic rx buffer num: 32
I (347) wifi: Init static rx buffer size: 1600
I (357) wifi: Init static rx buffer num: 10
I (357) wifi: Init dynamic rx buffer num: 32
I (357) APP_WIFI: start the WIFI SSID:[Connictro_GmbH] password:[******]
I (467) phy: phy_version: 4100, 6fa5e27, Jan 25 2019, 17:02:06, 0, 0
I (467) wifi: mode : sta (30:ae:a4:37:21:68)
I (467) APP_WIFI: Waiting for wifi
I (587) wifi: new:<1,0>, old:<1,0>, ap:<255,255>, sta:<1,0>, prof:1
I (1577) wifi: state: init -> auth (b0)
I (1577) wifi: state: auth -> init (8a0)
I (1577) wifi: new:<1,0>, old:<1,0>, ap:<255,255>, sta:<1,0>, prof:1
I (4117) wifi: new:<1,0>, old:<1,0>, ap:<255,255>, sta:<1,0>, prof:1
I (4117) wifi: state: init -> auth (b0)
I (4127) wifi: state: auth -> assoc (0)
I (4137) wifi: state: assoc -> run (10)
I (4167) wifi: connected with Connictro_GmbH, channel 1, bssid = 44:4e:6d:43:68:e3
I (4167) wifi: pm start, type: 1

I (4767) event: sta ip: 192.168.178.33, mask: 255.255.255.0, gw: 192.168.178.1
I (4767) APP_WIFI: IP Address:  192.168.178.33
I (4767) APP_WIFI: Subnet mask: 255.255.255.0
I (4777) APP_WIFI: Gateway:     192.168.178.1
I (4777) MB_C: Entered eMBInit with role 1, mode 0, port 2, baudrate 9600, parity 0
I (4787) MB_RTU: entered eMBGenericRTUInit
I (4787) uart: queue free spaces: 20
I (4787) MB_TIMERS: entered xMBPortTimersInit, timeout 80, is_master = 1
I (4807) MB_RTU: entered eMBRTUStart
I (4807) MB_APP: Modbus init finished with status: 00000000
I (4817) NA_CRED: namespace exists, trying to read Nabto ID from NVS
I (4817) NA_CRED: Nabto ID: aaaaaaaa.bbbbb.appmyproduct.com read from NVS
I (4827) NA_CRED: namespace exists, trying to read Nabto Key from NVS
I (4837) NA_CRED: Nabto Key: xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx read from NVS
I (4847) MB_APP: Nabto app main task started.
00:00:04:642 unabto_common_main.c(121) Device id: 'aaaaaaaa.bbbbb.appmyproduct.com'
00:00:04:649 unabto_common_main.c(122) Program Release 4.2.4
00:00:04:654 unabto_connection.c(103) Init connections
00:00:04:659 network_adapter.c(19) Open socket: ip=33.178.168.192, port=5570
00:00:04:667 network_adapter.c(57) Socket opened: ip=33.178.168.192, port=5570
00:00:04:673 network_adapter.c(19) Open socket: ip=33.178.168.192, port=0
00:00:04:680 network_adapter.c(57) Socket opened: ip=33.178.168.192, port=49153
00:00:04:686 unabto_context.c(55) SECURE ATTACH: 1, DATA: 1
00:00:04:692 unabto_context.c(63) NONCE_SIZE: 32, CLEAR_TEXT: 0
00:00:04:698 unabto_common_main.c(204) Nabto was successfully initialized
W (4907) NA_UTIL: WARNING: Remote access to the device is turned on by default. Please read TEN36 "Security in Nabto Solutions" to understand the security implications.
I (4927) NA_UTIL: Before fp_mem_init, default_settings.systemPermissions = E0000000, .defaultUserPermissions=C0000000, .firstUserPermissions=E0000000
I (4937) NA_CRED: namespace exists, trying to read Nabto persistence store from NVS
I (4977) NA_CRED: Nabto persistence store:  read from NVS
I (4977) NA_UTIL: Before acl_ae_init
I (4977) NA_CRED: namespace exists, trying to read Modbus config from NVS
I (4987) NA_CRED: Modbus config: {"r":[{"slaveID":"2","n":"Temperature","t":"n","r":"0000", "f":"r/10"},{"slaveID":"2","n":"Humidity","t":"n","r":"0001","f":"r/10"},
{"slaveID":"3","n":"AlfredoSwitch","fc":"05","r":"0000","o":"0100","c":"0000"},
{"slaveID":"3","n":"MexicanEggSwitch","fc":"05","r":"0001","o":"0100","c":"0000"},
{"slaveID":"3","n":"ScentedLambSwitch","fc":"05","r":"0002","o":"0100","c":"0000"},
{"slaveID":"3","n":"WalnutSwitch","fc":"05","r":"0003","o":"0100","c":"0000"}
]}
 read from NVS
I (5027) MB_APP: Nabto init completed.
00:00:04:831 unabto_context.c(55) SECURE ATTACH: 1, DATA: 1
00:00:04:836 unabto_context.c(63) NONCE_SIZE: 32, CLEAR_TEXT: 0
00:00:04:842 network_adapter.c(19) Open socket: ip=33.178.168.192, port=0
00:00:04:848 network_adapter.c(57) Socket opened: ip=33.178.168.192, port=49154
00:00:04:855 unabto_attach.c(805) State change from IDLE to WAIT_DNS
00:00:04:861 unabto_attach.c(806) Resolving DNS for aaaaaaaa.bbbbb.appmyproduct.com
00:00:04:877 unabto_attach.c(819) DNS error (returned by application)
00:00:04:877 unabto_attach.c(820) State change from WAIT_DNS to IDLE
00:00:04:880 unabto_attach.c(1045) There is no valid address to send statistics packets to
00:00:06:887 unabto_context.c(55) SECURE ATTACH: 1, DATA: 1
00:00:06:887 unabto_context.c(63) NONCE_SIZE: 32, CLEAR_TEXT: 0
00:00:06:889 network_adapter.c(19) Open socket: ip=33.178.168.192, port=0
00:00:06:895 network_adapter.c(57) Socket opened: ip=33.178.168.192, port=49155
00:00:06:901 unabto_attach.c(805) State change from IDLE to WAIT_DNS
00:00:06:907 unabto_attach.c(806) Resolving DNS for aaaaaaaa.bbbbb.appmyproduct.com
00:00:06:917 unabto_attach.c(825) Resolved DNS for aaaaaaaa.bbbbb.appmyproduct.com to:
00:00:06:922 unabto_attach.c(831)   Controller ip: 54.72.234.97
00:00:06:927 unabto_attach.c(837) State change from WAIT_DNS to WAIT_BS
00:00:06:937 unabto_attach.c(301) Sending INVITE to Base Station: 1
00:00:06:940 unabto_attach.c(238) len=31
00:00:06:943 unabto_attach.c(239) Send INVITE from 'aaaaaaaa.bbbbb.appmyproduct.com' to BS
00:00:07:007 unabto_common_main.c(340) Received remote packet length 33
00:00:07:008 unabto_message.c(256) Received from Base Station: 33 bytes
00:00:07:009 unabto_attach.c(471) Using GSP at 54.72.234.97:5565
00:00:07:015 unabto_attach.c(487) State change from WAIT_BS to WAIT_GSP
00:00:07:021 unabto_attach.c(488) GSP address: 54.72.234.97:5565
00:00:07:037 unabto_attach.c(312) Sending INVITE to GSP: 1
00:00:07:037 unabto_attach.c(238) len=31
00:00:07:037 unabto_attach.c(239) Send INVITE from 'aaaaaaaa.bbbbb.appmyproduct.com' to GSP
00:00:07:044 unabto_attach.c(275) ########    U_INVITE with LARGE nonce sent, version: - URL: -
00:00:07:097 unabto_common_main.c(340) Received remote packet length 180
00:00:07:098 unabto_message.c(258) Received from GSP: 180 bytes
00:00:07:098 unabto_attach.c(616) received ATTACH event
00:00:07:103 unabto_attach.c(641) nmc.ctx.privat     : 0.0.0.0:49155
00:00:07:109 unabto_attach.c(642) nmc.ctx.global     : 93.230.210.60:49155
00:00:07:116 unabto_attach.c(549) ########    U_ATTACH WITH cryptographic payload received
00:00:07:124 unabto_attach.c(557) before verify
00:00:07:129 unabto_attach.c(563) verifSize: 16 end-ptr 112
00:00:07:134 unabto_attach.c(589) GSP-ID(nsi): 1085623587
00:00:07:140 unabto_crypto.c(493) CRYPT_W_AES_CBC_HMAC_SHA256
00:00:07:146 unabto_packet.c(350) Integrity inserted
00:00:07:153 unabto_attach.c(591) State change from WAIT_GSP to ATTACHED
00:00:17:307 unabto_common_main.c(340) Received remote packet length 25
00:00:17:308 unabto_message.c(258) Received from GSP: 25 bytes
00:00:17:308 unabto_attach.c(701) (0.1085623587.0) Alive received
00:00:17:314 unabto_attach.c(703) Sending ALIVE response to GSP s=1
00:00:30:327 unabto_attach.c(425) Sending ALIVE POLL to GSP: 1
00:00:30:328 network_adapter.c(103) ERROR: 118 in nabto_write()
I (36267) event: sta ip: 192.168.178.33, mask: 255.255.255.0, gw: 192.168.178.1
00:00:37:687 unabto_common_main.c(340) Received remote packet length 25
00:00:37:688 unabto_message.c(258) Received from GSP: 25 bytes
00:00:37:688 unabto_attach.c(701) (0.1085623587.0) Alive received
00:00:37:694 unabto_attach.c(703) Sending ALIVE response to GSP s=2

