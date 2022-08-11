/*
  This file is part of the STM32duinoBLE library.
  Copyright (c) 2022 STMicroelectronics Srl. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _HCI_VENDOR_CLASS_H_
#define _HCI_VENDOR_CLASS_H_

#include "HCI.h"
#include "BLEChip.h"

#define HCI_EVENT_EXT_PKT   0x82
#define OGF_VENDOR_SPECIFIC 0x3F
#define EVT_VENDOR_SPECIFIC 0xFF

#define VS_EVT_BLUE_INITIALIZE 0x0001

#define VS_OPCODE_GATT_INIT  0xFD01
#define VS_OPCODE_GAP_INIT   0xFC8A
#define VS_OPCODE_WRITE_CONF 0xFC0C
#define VS_OPCODE_READ_CONF  0xFC0D

class HCIVendorClass: public HCIClass {
  enum {
    STARTED,
    INITIALIZED
  } status;

public:
  virtual int reset()
  {
    int ret = -1;

    // hci reset
    if (!_first_run) {
      ret = HCIClass::reset();
      if (ret < 0) return -1;
    }
    _first_run = false;

    // wait for blue initialize
    poll();

    // enable link-layer only
    uint8_t ll_only_params[] = {0x2C, 0x01, 0x01};
    ret = sendCommand(VS_OPCODE_WRITE_CONF, sizeof(ll_only_params), &ll_only_params);
    if (ret < 0) return -1;

    // GATT init
    ret = sendCommand(VS_OPCODE_GATT_INIT);
    if (ret < 0) return -1;

    // GAP init
    uint8_t gap_init_params[] = {0x0F, 0x00, 0x00, 0x00};
    ret = sendCommand(VS_OPCODE_GAP_INIT, sizeof(gap_init_params), &gap_init_params);
    if (ret < 0) return -1;

    // READ random address
    uint8_t read_raddr_params[] = {0x80};
    ret = sendCommand(VS_OPCODE_READ_CONF, sizeof(read_raddr_params), &read_raddr_params);
    if (ret < 0) return -1;

    // store random address
    uint8_t random_address[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    memcpy(random_address, &_cmdResponse[1], 6);

    // HCI reset
    ret = HCIClass::reset();
    if (ret < 0) return -1;

    // wait for blue initialize
    poll();

    // enable link-layer only
    // uint8_t ll_only_params[] = {0x2C, 0x01, 0x01};
    ret = sendCommand(VS_OPCODE_WRITE_CONF, sizeof(ll_only_params), &ll_only_params);
    if (ret < 0) return -1;

    // set random address
    ret = sendCommand(OGF_LE_CTL << 10 | OCF_LE_SET_RANDOM_ADDRESS, sizeof(random_address), random_address);
    if (ret < 0) return -1;

    return ret;
  }

  virtual void poll(unsigned long timeout = 0)
  {
    if (timeout) {
      _HCITransport->wait(timeout);
    }

    while (_HCITransport->available()) {
      byte b = _HCITransport->read();

      _recvBuffer[_recvIndex++] = b;

      if (_recvBuffer[0] == HCI_ACLDATA_PKT) {
        if (_recvIndex > 5 && _recvIndex >= (5 + (_recvBuffer[3] + (_recvBuffer[4] << 8)))) {
          if (_debug) {
            dumpPkt("HCI ACLDATA RX <- ", _recvIndex, _recvBuffer);
          }

          int pktLen = _recvIndex - 1;
          _recvIndex = 0;

          handleAclDataPkt(pktLen, &_recvBuffer[1]);
        }
      } else if (_recvBuffer[0] == HCI_EVENT_PKT) {
        if (_recvIndex > 3 && _recvIndex >= (3 + _recvBuffer[2])) {
          if (_debug) {
            dumpPkt("HCI EVENT RX <- ", _recvIndex, _recvBuffer);
          }

          // received full event
          int pktLen = _recvIndex - 1;
          _recvIndex = 0;

          handleEventPkt(pktLen, &_recvBuffer[1]);
        }
      } else if (_recvBuffer[0] == HCI_EVENT_EXT_PKT) {
        if (_recvIndex > 4 && _recvIndex >= (4 + (_recvBuffer[2] + (_recvBuffer[3] << 8)))) {
          if (_debug) {
            dumpPkt("HCI VS EVENT RX <- ", _recvIndex, _recvBuffer);
          }

          int pktLen = _recvIndex - 1;
          _recvIndex = 0;

          handleExtEventPkt(pktLen, &_recvBuffer[1]);
        }
      } else {
        _recvIndex = 0;

        if (_debug) {
          _debug->println(b, HEX);
        }
      }
    }
  }

protected:
  virtual void handleExtEventPkt(uint8_t plen, uint8_t pdata[])
  {
    struct __attribute__ ((packed)) HCIExtEventHdr {
      uint8_t evt;
      uint16_t plen;
    } *eventHdr = (HCIExtEventHdr*)pdata;

    if (eventHdr->evt == EVT_VENDOR_SPECIFIC) {
      uint16_t ecode = pdata[sizeof(HCIExtEventHdr)];
      if (ecode == VS_EVT_BLUE_INITIALIZE) {
        struct __attribute__ ((packed)) BlueInitialize {
          uint8_t reason;
        } *blueInitialize = (BlueInitialize*)&pdata[sizeof(HCIExtEventHdr) + 2];
        if (blueInitialize->reason == 0x00) {
          status = INITIALIZED;
        }
      }
    } else {
      HCIClass::handleEventPkt(plen, pdata);
    }
  }

  bool _first_run = true;
};

#endif /* _HCI_VENDOR_CLASS_H_ */
