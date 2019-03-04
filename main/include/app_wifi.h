/*
@file app_wifi.h
@brief handles the Wifi manager created by Tony Pottier, storing of credentials
       into NVM and credentials cleanup.

Copyright (c) 2019 Connictro GmbH / Michael Paar

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __APP_WIFI_H__
#define __APP_WIFI_H__

/* Macros */

/* Prototypes */

/*
 * Initialize wifi and wait for hard-coded connection; TODO to be changed as that is not desirable
 * (need to deliver godzillion of images... -> worth the effort) 
 *
 * TODO 
 *  - Read configuration from NVS. There could be a valid configuration or not.
 *  a) configuration is valid:
 *     - Try to connect to Wifi.
 *     - If it fails, retry 5 times more (pause 1 min.), if it still fails, delete configuration.
 *     - If it succeeds, returns (no return value).
 *  b) configuration is invalid:
 *    - Start AP mode with a very simple unsecure web server
 *    - User enters Wifi credentials (possibly also MQTT Broker URL/certificate, with default)
 *    - Store configuration in NVS and reboot (never returns).
 *
 */
void app_wifi_init(char *ssid_str);

#endif /* __APP_WIFI_H__ */
