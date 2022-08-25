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

#include "ATT.h"
#include "GAP.h"
#include "L2CAPSignaling.h"
#include "HCI.h"
#include "BLEChip.h"

#define HCI_EVENT_EXT_PKT   0x82
// #define OGF_VENDOR_SPECIFIC 0x3F
#define EVT_VENDOR_SPECIFIC 0xFF

#define VS_EVT_BLUE_INITIALIZE 0x0001

#define VS_OPCODE_GATT_INIT  0xFD01
#define VS_OPCODE_GAP_INIT   0xFC8A
#define VS_OPCODE_WRITE_CONF 0xFC0C
#define VS_OPCODE_READ_CONF  0xFC0D

class HCIVendorClass: public HCIClass {
public:
  virtual int begin()
  {
    status = STARTING;
    return HCIClass::begin();
  }

  virtual int reset()
  {
    int ret = 0;

    // if it's already running, we should hw-reset it
    if (status != STARTING) {
      // _HCITransport->end();
      // (void)_HCITransport->begin();
      end();
      ret = begin();
      if (ret < 0) return -1;
    }

    if (ble_type() == 1 || ble_chip() == BLUENRG_LP) {
      // wait for blue initialize (poll will handle this)
      poll();
      if (status != STARTED) return -2;
    } else if (ble_type() == 2) {
      // give the module some time to bootstrap
      delay(300);
      status = STARTED;
    } else {
      return -3;
    }

    // hci reset
    ret = HCIClass::reset(); // sendCommand(OGF_HOST_CTL << 10 | OCF_RESET)
    if (ret < 0) return -4;
    status = HCIRESET;

    if (ble_chip() == BLUENRG_LP) {
      // wait for blue initialize (poll will handle this)
      poll();
      if (status != STARTED) return -5;
      status = HCIRESET;
    }

    if (ble_type() == 1 || ble_chip() == BLUENRG_LP) {
      // enable link-layer only
      const uint8_t ll_only_params[] = {0x2C, 0x01, 0x01};
      ret = sendCommand(VS_OPCODE_WRITE_CONF, sizeof(ll_only_params), (void*)ll_only_params);
      if (ret < 0) return -6;
    } else {
      // BLE chip was reset: we need to wait for a while
      delay(300);
    }

    // BlueNRG-LP Workaround B part 2: if we already have the random address
    // just set it then return to avoid looping
    // memcmp(random_address, (const uint8_t[]){0x00,0x00,0x00,0x00,0x00,0x00}, 6) != 0
    if (ble_chip() == BLUENRG_LP && memcmp(random_address, (const uint8_t[]){0x00,0x00,0x00,0x00,0x00,0x00}, 6) != 0) {
      // set random address
      ret = sendCommand(OGF_LE_CTL << 10 | OCF_LE_SET_RANDOM_ADDRESS, sizeof(random_address), random_address);
      if (ret < 0) return -100;
      status = RUNNING;
    } else {
      // GATT init
      ret = sendCommand(VS_OPCODE_GATT_INIT);
      if (ret < 0) return -7;

      // GAP init
      if (ble_chip() == BLUENRG_LP) {
        uint8_t gap_init_params[] = {0x0F, 0x00, 0x00, 0x00};
        ret = sendCommand(VS_OPCODE_GAP_INIT, sizeof(gap_init_params), &gap_init_params);
      } else {
        uint8_t gap_init_params[] = {0x0F, 0x00, 0x00};
        ret = sendCommand(VS_OPCODE_GAP_INIT, sizeof(gap_init_params), &gap_init_params);
      }
      if (ret < 0) return -8;

      // read random address
      uint8_t read_raddr_params[] = {0x80};
      ret = sendCommand(VS_OPCODE_READ_CONF, sizeof(read_raddr_params), &read_raddr_params);
      if (ret < 0) return -9;

      // store random address
      memcpy(random_address, &_cmdResponse[1], 6);

      // BlueNRG-LP Workaround B part 1: repeat the reset procedure
      if (ble_chip() == BLUENRG_LP) {
        if (_debug) {
          _debug->println("WORKAROUND B: repeat reset");
        }
        ret = reset();
        if (ret < 0) return ret - 100;
      }

      status = RUNNING;
    }

    return ret;
  }

  virtual void poll(unsigned long timeout = 0UL)
  {
    // BlueNRG-LP Workaround A: give it some time be ready (should be done after each reset)
    if (status == STARTED && ble_chip() == BLUENRG_LP) {
      if (_debug) {
        _debug->println("WORKAROUND A: delay");
      }
      delay(100);
    }

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
          return;
        }
      } else if (_recvBuffer[0] == HCI_EVENT_PKT) {
        if (_recvIndex > 3 && _recvIndex >= (3 + _recvBuffer[2])) {
          if (_debug) {
            dumpPkt("HCI EVENT RX <- ", _recvIndex, _recvBuffer);
          }

          // received full event
          _recvIndex = 0;

          handleGenericEventPkt<uint8_t>(&_recvBuffer[1]);
          return;
        }
      } else if (_recvBuffer[0] == HCI_EVENT_EXT_PKT) {
        if (_recvIndex > 4 && _recvIndex >= (4 + (_recvBuffer[2] + (_recvBuffer[3] << 8)))) {
          if (_debug) {
            dumpPkt("HCI EXT EVENT RX <- ", _recvIndex, _recvBuffer);
          }

          // received full extended event
          _recvIndex = 0;

          handleGenericEventPkt<uint16_t>(&_recvBuffer[1]);
          return;
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
  enum {
    STARTING, // just after begin
    STARTED, // chip started
    HCIRESET, // hci reset done
    RUNNING // up and running
  } status;

  uint8_t random_address[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  template<typename LenTp>
  void handleGenericEventPkt(uint8_t pdata[])
  {
    struct __attribute__ ((packed)) HCIEventHdr {
      uint8_t evt;
      LenTp plen;
    } *eventHdr = (HCIEventHdr*)pdata;

    if (eventHdr->evt == EVT_DISCONN_COMPLETE) {
      struct __attribute__ ((packed)) DisconnComplete {
        uint8_t status;
        uint16_t handle;
        uint8_t reason;
      } *disconnComplete = (DisconnComplete*)&pdata[sizeof(HCIEventHdr)];

      ATT.removeConnection(disconnComplete->handle, disconnComplete->reason);
      L2CAPSignaling.removeConnection(disconnComplete->handle, disconnComplete->reason);

      if(GAP.advertising()) {
        HCI.leSetAdvertiseEnable(0x01);
      }
    } else if (eventHdr->evt == EVT_CMD_COMPLETE) {
      struct __attribute__ ((packed)) CmdComplete {
        uint8_t ncmd;
        uint16_t opcode;
        uint8_t status;
      } *cmdCompleteHeader = (CmdComplete*)&pdata[sizeof(HCIEventHdr)];

      _cmdCompleteOpcode = cmdCompleteHeader->opcode;
      _cmdCompleteStatus = cmdCompleteHeader->status;
      _cmdResponseLen = eventHdr->plen - sizeof(CmdComplete);
      _cmdResponse = &pdata[sizeof(HCIEventHdr) + sizeof(CmdComplete)];

    } else if (eventHdr->evt == EVT_CMD_STATUS) {
      struct __attribute__ ((packed)) CmdStatus {
        uint8_t status;
        uint8_t ncmd;
        uint16_t opcode;
      } *cmdStatusHeader = (CmdStatus*)&pdata[sizeof(HCIEventHdr)];

      _cmdCompleteOpcode = cmdStatusHeader->opcode;
      _cmdCompleteStatus = cmdStatusHeader->status;
      _cmdResponseLen = 0;
    } else if (eventHdr->evt == EVT_NUM_COMP_PKTS) {
      uint8_t numHandles = pdata[sizeof(HCIEventHdr)];
      uint16_t* data = (uint16_t*)&pdata[sizeof(HCIEventHdr) + sizeof(numHandles)];

      for (uint8_t i = 0; i < numHandles; i++) {
        handleNumCompPkts(data[0], data[1]);

        data += 2;
      }
    } else if (eventHdr->evt == EVT_LE_META_EVENT) {
      struct __attribute__ ((packed)) LeMetaEventHeader {
        uint8_t subevent;
      } *leMetaHeader = (LeMetaEventHeader*)&pdata[sizeof(HCIEventHdr)];

      if (leMetaHeader->subevent == EVT_LE_CONN_COMPLETE) {
        struct __attribute__ ((packed)) EvtLeConnectionComplete {
          uint8_t status;
          uint16_t handle;
          uint8_t role;
          uint8_t peerBdaddrType;
          uint8_t peerBdaddr[6];
          uint16_t interval;
          uint16_t latency;
          uint16_t supervisionTimeout;
          uint8_t masterClockAccuracy;
        } *leConnectionComplete = (EvtLeConnectionComplete*)&pdata[sizeof(HCIEventHdr) + sizeof(LeMetaEventHeader)];
      
        if (leConnectionComplete->status == 0x00) {
          ATT.addConnection(leConnectionComplete->handle,
                            leConnectionComplete->role,
                            leConnectionComplete->peerBdaddrType,
                            leConnectionComplete->peerBdaddr,
                            leConnectionComplete->interval,
                            leConnectionComplete->latency,
                            leConnectionComplete->supervisionTimeout,
                            leConnectionComplete->masterClockAccuracy);

          L2CAPSignaling.addConnection(leConnectionComplete->handle,
                                leConnectionComplete->role,
                                leConnectionComplete->peerBdaddrType,
                                leConnectionComplete->peerBdaddr,
                                leConnectionComplete->interval,
                                leConnectionComplete->latency,
                                leConnectionComplete->supervisionTimeout,
                                leConnectionComplete->masterClockAccuracy);
        }
      } else if (leMetaHeader->subevent == EVT_LE_ADVERTISING_REPORT) {
        struct __attribute__ ((packed)) EvtLeAdvertisingReport {
          uint8_t status;
          uint8_t type;
          uint8_t peerBdaddrType;
          uint8_t peerBdaddr[6];
          uint8_t eirLength;
          uint8_t eirData[31];
        } *leAdvertisingReport = (EvtLeAdvertisingReport*)&pdata[sizeof(HCIEventHdr) + sizeof(LeMetaEventHeader)];

        if (leAdvertisingReport->status == 0x01) {
          // last byte is RSSI
          int8_t rssi = leAdvertisingReport->eirData[leAdvertisingReport->eirLength];

          GAP.handleLeAdvertisingReport(leAdvertisingReport->type,
                                        leAdvertisingReport->peerBdaddrType,
                                        leAdvertisingReport->peerBdaddr,
                                        leAdvertisingReport->eirLength,
                                        leAdvertisingReport->eirData,
                                        rssi);

        }
      }
    } else if (eventHdr->evt == EVT_VENDOR_SPECIFIC) {
      uint16_t ecode = pdata[sizeof(HCIEventHdr)];
      if (ecode == VS_EVT_BLUE_INITIALIZE) {
        uint8_t reason = pdata[sizeof(HCIEventHdr) + 2];
        if (reason == 0x01) {
          status = STARTED;
        }
      }
    }
  }

  inline BLEChip_t ble_chip()
  {
    return static_cast<HCISpiTransportClass*>(_HCITransport)->ble_chip();
  }

  inline int ble_type()
  {
    return static_cast<HCISpiTransportClass*>(_HCITransport)->ble_type();
  }
};

#endif /* _HCI_VENDOR_CLASS_H_ */
