#pragma once

#include <fstream>
#include <memory>
#include <string>

#include "livox_def.h"

#include <liblas/liblas.hpp>

constexpr int32_t kMaxPointSize = 1500;

typedef struct {
  uint8_t device_index;
  uint8_t version;
  uint8_t port_id;
  uint8_t lidar_index;
  uint8_t rsvd;
  uint32_t error_code;
  uint8_t timestamp_type;
  uint8_t data_type;
  uint8_t timestamp[8];
  uint8_t raw_point[kMaxPointSize];
  uint32_t pack_size;
} LvxBasePackDetail;

struct LvxPacket {
  uint8_t lidar_index = 0u;
  uint8_t data_type = 0u;
  uint32_t pack_size = 0u;
  std::vector<std::vector<double>> xyz;
  std::vector<uint64_t> timestamp;
  std::vector<double> intensity;
};  // unused

class LasFileHandler final {
 public:
  LasFileHandler(const std::string& las_path, int32_t duration);
  ~LasFileHandler() = default;

  bool InitLasFile(const std::string& file_name = "");
  void InitLasFileHeader();
  void SaveFrameToLasFile(const std::list<LvxBasePackDetail>& point_packet);
  void CloseLasFile();
  void BasePointsHandle(LivoxEthPacket* data, LvxBasePackDetail& packet) const;
  uint32_t point_num() const { return point_num_; }

 private:
  liblas::Header header_;
  std::shared_ptr<liblas::Writer> writer_ = nullptr;
  std::ofstream las_file_;
  std::string las_path_;
  int32_t frame_duration_ = 5;
  uint32_t cur_frame_index_ = 0u;
  uint32_t point_num_ = 0u;
  double max_[3] = {0., 0., 0.};
  double min_[3] = {0., 0., 0.};
};
