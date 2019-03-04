/*
@file unabto_config.h
@brief Nabto configuration for Nabto - FreeModbus gateway.

Copyright (c) 2019 Connictro GmbH / Michael Paar and Nabto ApS

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

#ifndef _UNABTO_CONFIG_H_
#define _UNABTO_CONFIG_H_

#ifdef NABTO_ENABLE_LOCAL_ACCESS_LEGACY_PROTOCOL
#undef NABTO_ENABLE_LOCAL_ACCESS_LEGACY_PROTOCOL
#undef NABTO_REQUEST_MAX_SIZE
#undef NABTO_RESPONSE_MAX_SIZE
#undef NABTO_CONNECTIONS_SIZE
#undef NABTO_ENABLE_CLIENT_ID
#undef NABTO_ENABLE_CONNECTION_ESTABLISHMENT_ACL_CHECK
#undef NABTO_LOG_ALL
#undef NABTO_LOG_MODULE_FILTER
#undef NABTO_LOG_SEVERITY_FILTER
#undef NABTO_APPLICATION_EVENT_MODEL_ASYNC
#undef NABTO_APPREQ_QUEUE_SIZE
#endif

#define NABTO_ENABLE_STATUS_CALLBACKS 0
#define NABTO_ENABLE_LOCAL_CONNECTION                               1

// Default for gateway application
#define DEFAULT_BAUDRATE                                            9600
#define DEFAULT_STOPBITS                                            UART_STOPBITS_ONE
#define DEFAULT_PARITY                                              UART_PARITY_NONE
#define DEFAULT_MODBUS_NUMBER_OF_ADDRESSES                          240
#define MODBUS_NUMBER_OF_BUSSES                                     1


#define MAXIMUM_NUMBER_OF_SIMULTANEOUS_MODBUS_QUERIES               10

#define NABTO_ENABLE_REMOTE_CONNECTION                              1

#define NABTO_ENABLE_LOCAL_ACCESS_LEGACY_PROTOCOL                   0

#define NABTO_REQUEST_MAX_SIZE                                      1400
#define NABTO_RESPONSE_MAX_SIZE                                     1400
#define NABTO_CONNECTIONS_SIZE                                      20
#define NABTO_SET_TIME_FROM_ALIVE                                   0

#define NABTO_ENABLE_STREAM                                         0

#define NABTO_ENABLE_CLIENT_ID                                      1
#define NABTO_ENABLE_CONNECTION_ESTABLISHMENT_ACL_CHECK             1

#define NABTO_ENABLE_LOGGING                                        1
#define NABTO_LOG_ALL                                               1
//#define NABTO_LOG_MODULE_FILTER                                     NABTO_LOG_MODULE_ALL & ~(NABTO_LOG_MODULE_PERIPHERAL | NABTO_LOG_MODULE_ENCRYPTION | NABTO_LOG_MODULE_NETWORK)
#define NABTO_LOG_MODULE_FILTER                                     NABTO_LOG_MODULE_MODBUS
#define NABTO_LOG_SEVERITY_FILTER                                   NABTO_LOG_SEVERITY_WARN

#define NABTO_APPLICATION_EVENT_MODEL_ASYNC                         1
#define NABTO_APPREQ_QUEUE_SIZE                                     10

#endif /* _UNABTO_CONFIG_H_ */

