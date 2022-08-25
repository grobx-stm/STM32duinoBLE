/*
  This file is part of the ArduinoBLE library.
  Copyright (c) 2018 Arduino SA. All rights reserved.

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

#ifndef _HCI_H_
#define _HCI_H_

#include <Arduino.h>
#include "HCITransport.h"

#define HCI_COMMAND_PKT 0x01
#define HCI_ACLDATA_PKT 0x02
#define HCI_EVENT_PKT   0x04

#define EVT_DISCONN_COMPLETE 0x05
#define EVT_CMD_COMPLETE     0xe
#define EVT_CMD_STATUS       0x0f
#define EVT_NUM_COMP_PKTS    0x13
#define EVT_LE_META_EVENT    0x3e

#define EVT_LE_CONN_COMPLETE      0x01
#define EVT_LE_ADVERTISING_REPORT 0x02

#define OGF_LINK_CTL           0x01
#define OGF_HOST_CTL           0x03
#define OGF_INFO_PARAM         0x04
#define OGF_STATUS_PARAM       0x05
#define OGF_LE_CTL             0x08

// OGF_LINK_CTL
#define OCF_DISCONNECT         0x0006

// OGF_HOST_CTL
#define OCF_SET_EVENT_MASK     0x0001
#define OCF_RESET              0x0003

// OGF_INFO_PARAM
#define OCF_READ_LOCAL_VERSION 0x0001
#define OCF_READ_BD_ADDR       0x0009

// OGF_STATUS_PARAM
#define OCF_READ_RSSI          0x0005

// OGF_LE_CTL
#define OCF_LE_READ_BUFFER_SIZE           0x0002
#define OCF_LE_SET_RANDOM_ADDRESS         0x0005
#define OCF_LE_SET_ADVERTISING_PARAMETERS 0x0006
#define OCF_LE_SET_ADVERTISING_DATA       0x0008
#define OCF_LE_SET_SCAN_RESPONSE_DATA     0x0009
#define OCF_LE_SET_ADVERTISE_ENABLE       0x000a
#define OCF_LE_SET_SCAN_PARAMETERS        0x000b
#define OCF_LE_SET_SCAN_ENABLE            0x000c
#define OCF_LE_CREATE_CONN                0x000d
#define OCF_LE_CANCEL_CONN                0x000e
#define OCF_LE_CONN_UPDATE                0x0013

#define HCI_OE_USER_ENDED_CONNECTION 0x13

class HCIClass {
public:
  HCIClass();
  virtual ~HCIClass();

  virtual int begin();
  virtual void end();

  virtual void poll();
  virtual void poll(unsigned long timeout);

  virtual int reset();
  virtual int readLocalVersion(uint8_t& hciVer, uint16_t& hciRev, uint8_t& lmpVer,
                       uint16_t& manufacturer, uint16_t& lmpSubVer);
  virtual int readBdAddr(uint8_t addr[6]);

  virtual int readRssi(uint16_t handle);

  virtual int setEventMask(uint64_t eventMask);

  virtual int readLeBufferSize(uint16_t& pktLen, uint8_t& maxPkt);
  virtual int leSetRandomAddress(uint8_t addr[6]);
  virtual int leSetAdvertisingParameters(uint16_t minInterval, uint16_t maxInterval,
                                 uint8_t advType, uint8_t ownBdaddrType,
                                 uint8_t directBdaddrType, uint8_t directBdaddr[6],
                                 uint8_t chanMap,
                                 uint8_t filter);
  virtual int leSetAdvertisingData(uint8_t length, uint8_t data[]);
  virtual int leSetScanResponseData(uint8_t length, uint8_t data[]);
  virtual int leSetAdvertiseEnable(uint8_t enable);
  virtual int leSetScanParameters(uint8_t type, uint16_t interval, uint16_t window, 
                          uint8_t ownBdaddrType, uint8_t filter);
  virtual int leSetScanEnable(uint8_t enabled, uint8_t duplicates);
  virtual int leCreateConn(uint16_t interval, uint16_t window, uint8_t initiatorFilter,
                  uint8_t peerBdaddrType, uint8_t peerBdaddr[6], uint8_t ownBdaddrType,
                  uint16_t minInterval, uint16_t maxInterval, uint16_t latency,
                  uint16_t supervisionTimeout, uint16_t minCeLength, uint16_t maxCeLength);
  virtual int leConnUpdate(uint16_t handle, uint16_t minInterval, uint16_t maxInterval, 
                  uint16_t latency, uint16_t supervisionTimeout);
  virtual int leCancelConn();


  virtual int sendAclPkt(uint16_t handle, uint8_t cid, uint8_t plen, void* data);

  virtual int disconnect(uint16_t handle);

  virtual void debug(Stream& stream);
  virtual void noDebug();

  void setTransport(HCITransportInterface *HCITransport);

protected:
  virtual int sendCommand(uint16_t opcode, uint8_t plen = 0, void* parameters = NULL);

  virtual void handleAclDataPkt(uint8_t plen, uint8_t pdata[]);
  virtual void handleNumCompPkts(uint16_t handle, uint16_t numPkts);
  virtual void handleEventPkt(uint8_t plen, uint8_t pdata[]);

  virtual void dumpPkt(const char* prefix, uint8_t plen, uint8_t pdata[]);

  Stream* _debug;

  int _recvIndex;
  uint8_t _recvBuffer[3 + 255];

  uint16_t _cmdCompleteOpcode;
  int _cmdCompleteStatus;
  uint16_t _cmdResponseLen;
  uint8_t* _cmdResponse;

  uint8_t _maxPkt;
  uint8_t _pendingPkt;

  uint8_t _aclPktBuffer[255];

  HCITransportInterface *_HCITransport;
};

extern HCIClass& HCI;

#endif
