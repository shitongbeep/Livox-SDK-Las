#include "device_handler.h"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <list>

DeviceItem devices[kMaxLidarCount];
std::list<LvxBasePackDetail> point_packet_list;
std::vector<std::string> broadcast_code_rev;
std::shared_ptr<LasFileHandler> las_file_handler = nullptr;
uint8_t connected_lidar_count = 0;
std::mutex mtx;

std::condition_variable lidar_arrive_condition;
std::condition_variable extrinsic_condition;
std::condition_variable point_pack_condition;

bool is_finish_extrinsic_parameter = false;
bool is_read_extrinsic_from_xml = false;

using namespace std::chrono;

/** Receiving error message from Livox Lidar. */
void OnLidarErrorStatusCallback(livox_status status, uint8_t handle, ErrorMessage* message) {
  static uint32_t error_message_count = 0;
  if (message != NULL) {
    ++error_message_count;
    if (0 == (error_message_count % 100)) {
      printf("handle: %u\n", handle);
      printf("temp_status : %u\n", message->lidar_error_code.temp_status);
      printf("volt_status : %u\n", message->lidar_error_code.volt_status);
      printf("motor_status : %u\n", message->lidar_error_code.motor_status);
      printf("dirty_warn : %u\n", message->lidar_error_code.dirty_warn);
      printf("firmware_err : %u\n", message->lidar_error_code.firmware_err);
      printf("pps_status : %u\n", message->lidar_error_code.device_status);
      printf("fan_status : %u\n", message->lidar_error_code.fan_status);
      printf("self_heating : %u\n", message->lidar_error_code.self_heating);
      printf("ptp_status : %u\n", message->lidar_error_code.ptp_status);
      printf("time_sync_status : %u\n", message->lidar_error_code.time_sync_status);
      printf("system_status : %u\n", message->lidar_error_code.system_status);
    }
  }
}

/** Receiving point cloud data from Livox LiDAR. */
void GetLidarData(uint8_t handle, LivoxEthPacket* data, uint32_t data_num, void* client_data) {
  if (data) {
    if (handle < connected_lidar_count && las_file_handler != nullptr) {
      std::unique_lock<std::mutex> lock(mtx);
      LvxBasePackDetail packet;
      packet.device_index = handle;
      las_file_handler->BasePointsHandle(data, packet);
      point_packet_list.push_back(packet);
    }
  }
}

/** Callback function of starting sampling. */
void OnSampleCallback(livox_status status, uint8_t handle, uint8_t response, void* data) {
  printf("OnSampleCallback statues %d handle %d response %d \n", status, handle, response);
  if (status == kStatusSuccess) {
    if (response != 0) {
      devices[handle].device_state = kDeviceStateConnect;
    }
  } else if (status == kStatusTimeout) {
    devices[handle].device_state = kDeviceStateConnect;
  }
}

/** Callback function of stopping sampling. */
void OnStopSampleCallback(livox_status status, uint8_t handle, uint8_t response, void* data) {
}

/** Callback function of get LiDARs' extrinsic parameter. */
// void OnGetLidarExtrinsicParameter(livox_status status, uint8_t handle, LidarGetExtrinsicParameterResponse* response, void* data) {
//   if (status == kStatusSuccess) {
//     if (response != 0) {
//       printf("OnGetLidarExtrinsicParameter statue %d handle %d response %d \n", status, handle, response->ret_code);
//       std::unique_lock<std::mutex> lock(mtx);
//       LvxDeviceInfo lidar_info;
//       strncpy((char*)lidar_info.lidar_broadcast_code, devices[handle].info.broadcast_code, kBroadcastCodeSize);
//       memset(lidar_info.hub_broadcast_code, 0, kBroadcastCodeSize);
//       lidar_info.device_index = handle;
//       lidar_info.device_type = devices[handle].info.type;
//       lidar_info.extrinsic_enable = true;
//       lidar_info.pitch = response->pitch;
//       lidar_info.roll = response->roll;
//       lidar_info.yaw = response->yaw;
//       lidar_info.x = static_cast<float>(response->x / 1000.0);
//       lidar_info.y = static_cast<float>(response->y / 1000.0);
//       lidar_info.z = static_cast<float>(response->z / 1000.0);
//       lvx_file_handler.AddDeviceInfo(lidar_info);
//       if (lvx_file_handler.GetDeviceInfoListSize() == connected_lidar_count) {
//         is_finish_extrinsic_parameter = true;
//         extrinsic_condition.notify_one();
//       }
//     }
//   } else if (status == kStatusTimeout) {
//     printf("GetLidarExtrinsicParameter timeout! \n");
//   }
// }

/** Get LiDARs' extrinsic parameter from file named "extrinsic.xml". */
// void LidarGetExtrinsicFromXml(uint8_t handle) {
//   LvxDeviceInfo lidar_info;
//   ParseExtrinsicXml(devices[handle], lidar_info);
//   lvx_file_handler.AddDeviceInfo(lidar_info);
//   lidar_info.extrinsic_enable = true;
//   if (lvx_file_handler.GetDeviceInfoListSize() == broadcast_code_list.size()) {
//     is_finish_extrinsic_parameter = true;
//     extrinsic_condition.notify_one();
//   }
// }

/** Query the firmware version of Livox LiDAR. */
void OnDeviceInformation(livox_status status, uint8_t handle, DeviceInformationResponse* ack, void* data) {
  if (status != kStatusSuccess) {
    printf("Device Query Informations Failed %d\n", status);
  }
  if (ack) {
    printf("firm ver: %d.%d.%d.%d\n",
           ack->firmware_version[0],
           ack->firmware_version[1],
           ack->firmware_version[2],
           ack->firmware_version[3]);
  }
}

void LidarConnect(const DeviceInfo* info) {
  uint8_t handle = info->handle;
  QueryDeviceInformation(handle, OnDeviceInformation, NULL);
  if (devices[handle].device_state == kDeviceStateDisconnect) {
    devices[handle].device_state = kDeviceStateConnect;
    devices[handle].info = *info;
  }
}

void LidarDisConnect(const DeviceInfo* info) {
  uint8_t handle = info->handle;
  devices[handle].device_state = kDeviceStateDisconnect;
}

void LidarStateChange(const DeviceInfo* info) {
  uint8_t handle = info->handle;
  devices[handle].info = *info;
}

/** Callback function of changing of device state. */
void OnDeviceInfoChange(const DeviceInfo* info, DeviceEvent type) {
  if (info == nullptr) {
    return;
  }
  printf("OnDeviceChange broadcast code %s update type %d\n", info->broadcast_code, type);
  uint8_t handle = info->handle;
  if (handle >= kMaxLidarCount) {
    return;
  }

  if (type == kEventConnect) {
    LidarConnect(info);
    printf("[WARNING] Lidar sn: [%s] Connect!!!\n", info->broadcast_code);
  } else if (type == kEventDisconnect) {
    LidarDisConnect(info);
    printf("[WARNING] Lidar sn: [%s] Disconnect!!!\n", info->broadcast_code);
  } else if (type == kEventStateChange) {
    LidarStateChange(info);
    printf("[WARNING] Lidar sn: [%s] StateChange!!!\n", info->broadcast_code);
  }

  if (devices[handle].device_state == kDeviceStateConnect) {
    printf("Device Working State %d\n", devices[handle].info.state);
    if (devices[handle].info.state == kLidarStateInit) {
      printf("Device State Change Progress %u\n", devices[handle].info.status.progress);
    } else {
      printf("Device State Error Code 0X%08x\n", devices[handle].info.status.status_code.error_code);
    }
    printf("Device feature %d\n", devices[handle].info.feature);
    SetErrorMessageCallback(handle, OnLidarErrorStatusCallback);
    if (devices[handle].info.state == kLidarStateNormal) {
      // if (!is_read_extrinsic_from_xml) {
      //   LidarGetExtrinsicParameter(handle, OnGetLidarExtrinsicParameter, nullptr);
      // } else {
      //   LidarGetExtrinsicFromXml(handle);
      // }
      LidarStartSampling(handle, OnSampleCallback, nullptr);
      devices[handle].device_state = kDeviceStateSampling;
    }
  }
}

/** Callback function when broadcast message received.
 * You need to add listening device broadcast code and set the point cloud data callback in this function.
 */
void OnDeviceBroadcast(const BroadcastDeviceInfo* info) {
  if (info == nullptr || info->dev_type == kDeviceTypeHub) {
    return;
  }

  printf("Receive Broadcast Code %s\n", info->broadcast_code);
  if ((broadcast_code_rev.size() == 0) ||
      (std::find(broadcast_code_rev.begin(), broadcast_code_rev.end(), info->broadcast_code) == broadcast_code_rev.end())) {
    broadcast_code_rev.push_back(info->broadcast_code);
    lidar_arrive_condition.notify_one();
  }
}

/** Wait until no new device arriving in 2 second. */
void WaitForDevicesReady() {
  bool device_ready = false;
  seconds wait_time = seconds(2);
  steady_clock::time_point last_time = steady_clock::now();
  while (!device_ready) {
    std::unique_lock<std::mutex> lock(mtx);
    lidar_arrive_condition.wait_for(lock, wait_time);
    if ((steady_clock::now() - last_time + milliseconds(50)) >= wait_time) {
      device_ready = true;
    } else {
      last_time = steady_clock::now();
    }
  }
}

// void WaitForExtrinsicParameter() {
//   std::unique_lock<std::mutex> lock(mtx);
//   extrinsic_condition.wait(lock);
// }

void AddDevicesToConnect(const std::vector<std::string>& broadcast_code_list) {
  if (broadcast_code_rev.size() == 0)
    return;
  for (int i = 0; i < broadcast_code_rev.size(); ++i) {
    if ((broadcast_code_list.size() != 0) &&
        (std::find(broadcast_code_list.begin(), broadcast_code_list.end(), broadcast_code_rev[i]) == broadcast_code_list.end())) {
      continue;
    }
    std::cout << "add device " << broadcast_code_rev[i] << std::endl;
    uint8_t handle = 0;
    livox_status result = AddLidarToConnect(broadcast_code_rev[i].c_str(), &handle);
    if (result == kStatusSuccess) {
      /** Set the point cloud data for a specific Livox LiDAR. */
      SetDataCallback(handle, GetLidarData, nullptr);
      devices[handle].handle = handle;
      devices[handle].device_state = kDeviceStateDisconnect;
      connected_lidar_count++;
    }
  }
}