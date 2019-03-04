/*
@file mbtypes.h
@brief Generic modbus types definitions both used in modbus stack and application.

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

#ifndef __MBTYPES_H__
#define __MBTYPES_H__

#include "queue_berkeley.h"

#define MAX_SIMULTANEOUS_MODBUS_REQUESTS_PER_INSTANCE 10
#define MB_MAX_APPLICATION_DATA 125
#define MAX_COIL_BUF_SIZE        16    // this accomodates 128 coil values

#ifndef MB_SER_PDU_SIZE_MAX
#define MB_SER_PDU_SIZE_MAX     256
#endif

#ifndef MB_BASIC_TYPES
#define MB_BASIC_TYPES
typedef char    BOOL;

typedef unsigned char UCHAR;
typedef char    CHAR;

typedef unsigned short USHORT;
typedef short   SHORT;

typedef unsigned long ULONG;
typedef long    LONG;

#ifndef TRUE
#define TRUE            1
#endif

#ifndef FALSE
#define FALSE           0
#endif

#endif /* MB_BASIC_TYPES */

/*! \ingroup modbus
 * \brief Modbus role (Slave/Master).
 *
 * Modbus serial supports two roles: Either Slave or Master.
 * In Slave the device just responds to requests from a master 
 * (Example use case: device controls sensors and is attached to a Modbus network).
 * In Master role the device controls nodes on a Modbus network 
 * (Example use case: Modbus to Wifi or IoT gateway).
 */
    typedef enum
{
    MB_ROLE_SLAVE,              /*!< Slave role. */
    MB_ROLE_MASTER,             /*!< Master role. */
} eMBRole;

/*! \ingroup modbus
 * \brief Modbus serial transmission modes (RTU/ASCII).
 *
 * Modbus serial supports two transmission modes. Either ASCII or RTU. RTU
 * is faster but has more hardware requirements and requires a network with
 * a low jitter. ASCII is slower and more reliable on slower links (E.g. modems)
 */
    typedef enum
{
    MB_RTU,                     /*!< RTU transmission mode. */
    MB_ASCII,                   /*!< ASCII transmission mode. */
    MB_TCP                      /*!< TCP mode. */
} eMBMode;

/*! \ingroup modbus
 * \brief Parity used for characters in serial mode.
 *
 * The parity which should be applied to the characters sent over the serial
 * link. Please note that this values are actually passed to the porting
 * layer and therefore not all parity modes might be available.
 */
typedef enum
{
    MB_PAR_NONE,                /*!< No parity. */
    MB_PAR_ODD,                 /*!< Odd parity. */
    MB_PAR_EVEN                 /*!< Even parity. */
} eMBParity;


/*! \ingroup modbus
 * \brief Errorcodes used by all function in the Master request.
 */
typedef enum {
    MB_MRE_NO_ERR,      /*!< no error. */
    MB_MRE_NO_REG,      /*!< illegal register address. */
    MB_MRE_ILL_ARG,     /*!< illegal argument. */
    MB_MRE_REV_DATA,    /*!< receive data error. */
    MB_MRE_TIMEDOUT,    /*!< timeout error occurred. */
    MB_MRE_MASTER_BUSY, /*!< master is busy now. */
    MB_MRE_EXE_FUN,     /*!< execute function error. */
    /* The following codes are used in Modbus supervisor only. */
    MB_MRE_ILL_FCT,     /*!< illegal function code. */
    MB_MRE_IO_ERR,      /*!< I/O error occured. */
    MB_MRE_PORTING,     /*!< Illegal state in specific porting layer occured. */
    MB_MRE_ILL_STATE,   /*!< Illegal state in protocol stack occured. */
    MB_MRE_OUTOFMEM,    /*!< out of memory error occured. */
    MB_MRE_PENDING,     /*!< request with this request ID still pending. */
    MB_MRE_NOTFOUND     /*!< descriptor request ID not found. */
} eMBMasterReqErrCode;


/*! \ingroup modbus
 *  \brief Message for formatted list transfer.
 */
typedef struct mb_formatted_list_cmd {
    uint8_t                        slave_address;  /*!< Slave address this request is for. */
    uint8_t                        function_code;  /*!< Allowed modbus function code for request (1-6, 0x0f, 0x10, 0x17). Particular slaves might not respond to all of it. */
    uint16_t                       read_start;     /*!< First register to read (meaningful for modbus function code 1-4 and 0x17 only). */
    uint16_t                       read_len;       /*!< Number of registers to read (meaningful for modbus function code 1-4 and 0x17 only). */
    uint16_t                       write_start;    /*!< First register to write (meaningful for modbus function code 5, 6, 0x0f, 0x10 and 0x17 only). */
    uint16_t                       write_len;      /*!< First register to write (meaningful for modbus function code 5, 6, 0x0f, 0x10 and 0x17 only). */
    uint16_t                       unpacked_data[MB_MAX_APPLICATION_DATA];  /*!< Data for write or read goes here. */
} __attribute__((packed)) sMBFormattedList;

/*! \ingroup modbus
 *  \brief Generic message passing descriptor for reading/writing data from/to modbus slaves.
 *         This is used for interfacing with the application.
 *         Suitable for use with tail queue.
 *
 *  Data buffer is reused if data is returned from a response, undefined otherwise in responses.
 *  For Modbus coils, any value > 0 is interpreted as 1 (returns for data readas are either 0 or 1), 
 * packing/unpacking is handled by the modbus controller.
 *  No need to do the bit twiddling on application side if using formatted list format.
 */
struct mb_request_entry {
    uint32_t                       request_id;      /*!< Master unique internal request ID. */
    QueueHandle_t                  req_origin;      /*!< Queue handle of originator of this request. */
    eMBMasterReqErrCode            result;          /*!< Master response status */
    union {
        sMBFormattedList           f;               /*!< Request/response as formatted list */
        unsigned char              raw_frame[MB_SER_PDU_SIZE_MAX]; /*!< Request/response as raw modbus frame */
    } u;
    TAILQ_ENTRY(mb_request_entry)  entries;
    uint16_t                       retransmit_count;/*!< Number of retransmits to try if an attempt failed. NOTE: unused/ignored at the moment. */
    uint16_t                       raw_frame_length;/*!< Length of raw frame, used only in raw modbus frame mode. */
    BOOL                           is_raw_request;  /*!< If the request is based on raw modbus frames, not formatted. */
} __attribute__((packed));

#endif /* __MBTYPES_H__ */

