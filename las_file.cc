#include "las_file.h"

#include <cassert>
#include <chrono>
#include <condition_variable>
#include <iostream>

#include "device_handler.h"
#include "las_file_handler.h"
#include "livox_def.h"
#include "livox_sdk.h"

std::condition_variable point_pack_duration;

namespace {
using std::chrono::milliseconds;
using std::chrono::steady_clock;
const int32_t kFrameDurationTime = 50;
}  // namespace

int32_t GenerateLasFile(const std::vector<std::string>& broadcast_code_list,
                        const std::string& las_path,
                        const std::string& las_name,
                        int32_t duration) {
  assert(!las_path.empty() && duration > 0);
  if (broadcast_code_list.empty()) {
    std::cerr << "no broadcast_code input";
    return -2;
  }
  std::cout << "Livox SDK initializing.\n";
  /** Initialize Livox-SDK. */
  if (!Init()) {
    std::cerr << "Livox SDK initiale fail.\n";
    return -2;
  }
  LivoxSdkVersion _sdkversion;
  GetLivoxSdkVersion(&_sdkversion);
  std::cout << "Livox SDK version " << _sdkversion.major << "."
            << _sdkversion.minor << "." << _sdkversion.patch << std::endl;
  memset(devices, 0, sizeof(devices));
  SetBroadcastCallback(OnDeviceBroadcast);
  SetDeviceStateUpdateCallback(OnDeviceInfoChange);
  /** Start the device discovering routine. */
  if (!Start()) {
    Uninit();
    std::cerr << "Livox SDK start fail.\n";
    return -2;
  }

  std::cout << "Start discovering device." << std::endl;
  WaitForDevicesReady();
  AddDevicesToConnect(broadcast_code_list);
  if (connected_lidar_count == 0) {
    std::cerr << "No device will be connected." << std::endl;
    Uninit();
    return -2;
  } else {
    std::cout << (int32_t)connected_lidar_count << " device connected." << std::endl;
  }

  std::cout << "Start initialize las file." << std::endl;
  las_file_handler.reset(new LasFileHandler(las_path, duration));
  if (!las_file_handler->InitLasFile(las_name)) {
    std::cerr << "las file init fail." << std::endl;
    Uninit();
    return -1;
  } else {
    std::cout << "las file init succ." << std::endl;
  }
  las_file_handler->InitLasFileHeader();
  std::cout << "las file header init succ." << std::endl;

  steady_clock::time_point start_time = steady_clock::now();
  steady_clock::time_point last_time = steady_clock::now();
  const int32_t frame_rate = 1000 / kFrameDurationTime;
  for (int32_t i = 0; i < frame_rate * duration; ++i) {
    std::cout << "Saving frame " << i << "to lvx file." << std::endl;
    std::list<LvxBasePackDetail> point_packet_list_temp;
    {
      std::unique_lock<std::mutex> lock(mtx);
      point_pack_duration.wait_for(lock, milliseconds(kFrameDurationTime) -
                                             (steady_clock::now() - last_time));
      last_time = steady_clock::now();
      point_packet_list_temp.swap(point_packet_list);
    }
    las_file_handler->SaveFrameToLasFile(point_packet_list_temp);
    std::cout << "Finish save frame " << i << " to lvx file at: " << std::fixed
              << (steady_clock::now() - start_time).count() * 1.e-9 << std::endl;
  }
  std::cout << "save " << las_file_handler->point_num() << " points"
            << std::endl;

  return 1;
}
