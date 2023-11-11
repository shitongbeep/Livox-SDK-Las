#pragma once

#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "las_file_handler.h"
#include "livox_def.h"
#include "livox_sdk.h"

typedef enum {
  kDeviceStateDisconnect = 0,
  kDeviceStateConnect = 1,
  kDeviceStateSampling = 2,
} DeviceState;

typedef struct {
  uint8_t handle;
  DeviceState device_state;
  DeviceInfo info;
} DeviceItem;

extern std::vector<std::string> broadcast_code_rev;
extern DeviceItem devices[kMaxLidarCount];
extern std::list<LvxBasePackDetail> point_packet_list;
extern std::shared_ptr<LasFileHandler> las_file_handler;
extern uint8_t connected_lidar_count;
extern std::mutex mtx;

void OnDeviceInfoChange(const DeviceInfo* info, DeviceEvent type);
void OnDeviceBroadcast(const BroadcastDeviceInfo* info);
void WaitForDevicesReady();
void AddDevicesToConnect(const std::vector<std::string>& broadcast_code_list);
