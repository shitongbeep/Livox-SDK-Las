#include "las_file_handler.h"

#include <liblas/liblas.hpp>

namespace {
uint32_t RAW_POINT_NUM = 100;
uint32_t SINGLE_POINT_NUM = 96;
uint32_t DUAL_POINT_NUM = 48;
uint32_t TRIPLE_POINT_NUM = 30;
uint32_t IMU_POINT_NUM = 1;
}  // namespace

LasFileHandler::LasFileHandler(const std::string& las_path, int32_t duration)
    : las_path_(las_path),
      frame_duration_(duration) {
}

bool LasFileHandler::InitLasFile(const std::string& file_name) {
  std::string file_path = las_path_;
  if (file_path[file_path.size() - 1] != '/') {
    file_path += '/';
  }
  if (file_name.empty()) {
    file_path += "pointcloud.las";
  } else {
    file_path += file_name;
  }
  if (file_path.substr(file_path.size() - 4, 4) != ".las") {
    file_path += "./las";
  }
  if (access(file_path.c_str(), F_OK) != -1) {
    std::cerr << "File " << file_path << " existed" << std::endl;
    return false;
  } else {
    std::clog << "File " << file_path << " will be created" << std::endl;
  }
  las_file_.open(file_path, std::ios::out | std::ios::binary);
  if (!las_file_.is_open()) {
    std::cerr << "File " << file_path << " creat fail" << std::endl;
    return false;
  } else {
    std::clog << "File " << file_path << " creat success" << std::endl;
  }
  return true;
}

void LasFileHandler::InitLasFileHeader() {
  header_.SetDataFormatId(liblas::ePointFormat3);
  header_.SetVersionMinor(2);
  header_.SetVersionMajor(1);
  header_.SetScale(0.001, 0.001, 0.001);
  writer_.reset(new liblas::Writer(las_file_, header_));
}

void LasFileHandler::SaveFrameToLasFile(
    const std::list<LvxBasePackDetail>& point_packet) {
  if (writer_ == nullptr) {
    std::cerr << "uninitialized liblas writer";
  }
  for (const auto& packet : point_packet) {
    uint64_t timestamp;
    memcpy(&timestamp, packet.timestamp, sizeof(uint64_t));
    // std::cout << "save packet at :" << std::fixed << timestamp * 1.e-9
    //           << " data type: " << (int32_t)packet.data_type << std::endl;
    if (packet.data_type == PointDataType::kCartesian) {
      for (uint32_t i = 0u; i < RAW_POINT_NUM; ++i) {
        LivoxRawPoint lvx_point;
        LivoxRawPoint* src_ptr = (LivoxRawPoint*)(packet.raw_point +
                                                  i * sizeof(LivoxRawPoint));
        memcpy(&lvx_point, src_ptr, sizeof(LivoxRawPoint));
        liblas::Point point(&header_);
        if (lvx_point.x == 0 && lvx_point.y == 0 && lvx_point.z == 0) {
          continue;
        }
        point.SetCoordinates(lvx_point.x * 1.e-3, lvx_point.y * 1.e-3,
                             lvx_point.z * 1.e-3);
        point.SetIntensity(lvx_point.reflectivity);
        // point.SetTime();
        writer_->WritePoint(point);
        ++point_num_;
      }
    } else if (packet.data_type == PointDataType::kExtendCartesian) {
      for (uint32_t i = 0u; i < SINGLE_POINT_NUM; ++i) {
        LivoxExtendRawPoint lvx_point;
        LivoxExtendRawPoint* src_ptr =
            (LivoxExtendRawPoint*)(packet.raw_point +
                                   i * sizeof(LivoxExtendRawPoint));
        memcpy(&lvx_point, src_ptr, sizeof(LivoxExtendRawPoint));
        liblas::Point point(&header_);
        if (lvx_point.x == 0 && lvx_point.y == 0 && lvx_point.z == 0) {
          continue;
        }
        point.SetCoordinates((double)lvx_point.x * 1.e-3,
                             (double)lvx_point.y * 1.e-3,
                             (double)lvx_point.z * 1.e-3);
        point.SetIntensity(lvx_point.reflectivity);
        point.SetColor(liblas::Color(255, 255, 255));
        max_[0] = std::fmax(max_[0], (double)lvx_point.x * 1.e-3);
        max_[1] = std::fmax(max_[1], (double)lvx_point.y * 1.e-3);
        max_[2] = std::fmax(max_[2], (double)lvx_point.z * 1.e-3);
        min_[0] = std::fmin(min_[0], (double)lvx_point.x * 1.e-3);
        min_[1] = std::fmin(min_[1], (double)lvx_point.y * 1.e-3);
        min_[2] = std::fmin(min_[2], (double)lvx_point.z * 1.e-3);
        // point.SetTime();
        writer_->WritePoint(point);
        ++point_num_;
      }
    } else if (packet.data_type == PointDataType::kDualExtendCartesian) {
      for (uint32_t i = 0u; i < DUAL_POINT_NUM; ++i) {
        LivoxDualExtendRawPoint lvx_point;
        LivoxDualExtendRawPoint* src_ptr =
            (LivoxDualExtendRawPoint*)(packet.raw_point +
                                       i * sizeof(LivoxDualExtendRawPoint));
        memcpy(&lvx_point, src_ptr, sizeof(LivoxDualExtendRawPoint));
        liblas::Point point(&header_);
        if (lvx_point.x1 != 0 || lvx_point.y1 != 0 || lvx_point.z1 != 0) {
          point.SetCoordinates((double)lvx_point.x1 * 1.e-3,
                               (double)lvx_point.y1 * 1.e-3,
                               (double)lvx_point.z1 * 1.e-3);
          point.SetIntensity(lvx_point.reflectivity1);
          writer_->WritePoint(point);
          ++point_num_;
        }
        if (lvx_point.x2 != 0 || lvx_point.y2 != 0 || lvx_point.z2 != 0) {
          point.SetCoordinates((double)lvx_point.x2 * 1.e-3,
                               (double)lvx_point.y2 * 1.e-3,
                               (double)lvx_point.z2 * 1.e-3);
          point.SetIntensity(lvx_point.reflectivity2);
          writer_->WritePoint(point);
          ++point_num_;
        }
      }
    } else if (packet.data_type == PointDataType::kTripleExtendCartesian) {
      for (uint32_t i = 0u; i < TRIPLE_POINT_NUM; ++i) {
        LivoxTripleExtendRawPoint lvx_point;
        LivoxTripleExtendRawPoint* src_ptr =
            (LivoxTripleExtendRawPoint*)(packet.raw_point +
                                         i * sizeof(LivoxTripleExtendRawPoint));
        memcpy(&lvx_point, src_ptr, sizeof(LivoxTripleExtendRawPoint));
        liblas::Point point(&header_);
        if (lvx_point.x1 != 0 || lvx_point.y1 != 0 || lvx_point.z1 != 0) {
          point.SetCoordinates((double)lvx_point.x1 * 1.e-3,
                               (double)lvx_point.y1 * 1.e-3,
                               (double)lvx_point.z1 * 1.e-3);
          point.SetIntensity(lvx_point.reflectivity1);
          writer_->WritePoint(point);
          ++point_num_;
        }
        if (lvx_point.x2 != 0 || lvx_point.y2 != 0 || lvx_point.z2 != 0) {
          point.SetCoordinates((double)lvx_point.x2 * 1.e-3,
                               (double)lvx_point.y2 * 1.e-3,
                               (double)lvx_point.z2 * 1.e-3);
          point.SetIntensity(lvx_point.reflectivity2);
          writer_->WritePoint(point);
          ++point_num_;
        }
        if (lvx_point.x3 != 0 || lvx_point.y3 != 0 || lvx_point.z3 != 0) {
          point.SetCoordinates((double)lvx_point.x3 * 1.e-3,
                               (double)lvx_point.y3 * 1.e-3,
                               (double)lvx_point.z3 * 1.e-3);
          point.SetIntensity(lvx_point.reflectivity3);
          writer_->WritePoint(point);
          ++point_num_;
        }
      }
    } else {
      continue;
    }
  }
  // std::clog << "headersize" << header_.GetPointRecordsCount() << std::endl;
  // std::clog << "max:" << header_.GetMaxX() << std::endl;
  // std::clog << "min:" << header_.GetMinX() << std::endl;
  header_.SetPointRecordsCount(point_num_);
  header_.SetPointRecordsByReturnCount(0, point_num_);
  header_.SetMax(max_[0], max_[1], max_[2]);
  header_.SetMin(min_[0], min_[1], min_[2]);
  writer_->SetHeader(header_);
  writer_->WriteHeader();
}

void LasFileHandler::CloseLasFile() {
  if (las_file_.is_open()) {
    las_file_.flush();
    las_file_.close();
  }
}

void LasFileHandler::BasePointsHandle(LivoxEthPacket* data,
                                      LvxBasePackDetail& packet) const {
  packet.version = data->version;
  packet.port_id = data->slot;
  packet.lidar_index = data->id;
  packet.rsvd = data->rsvd;
  packet.error_code = data->err_code;
  packet.timestamp_type = data->timestamp_type;
  packet.data_type = data->data_type;
  memcpy(packet.timestamp, data->timestamp, 8 * sizeof(uint8_t));
  switch (packet.data_type) {
    case PointDataType::kCartesian:
      packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point) -
                         sizeof(packet.pack_size) +
                         RAW_POINT_NUM * sizeof(LivoxRawPoint);
      memcpy(packet.raw_point, (void*)data->data,
             RAW_POINT_NUM * sizeof(LivoxRawPoint));
      break;
    case PointDataType::kSpherical:
      packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point) -
                         sizeof(packet.pack_size) +
                         RAW_POINT_NUM * sizeof(LivoxSpherPoint);
      memcpy(packet.raw_point, (void*)data->data,
             RAW_POINT_NUM * sizeof(LivoxSpherPoint));
      break;
    case PointDataType::kExtendCartesian:
      packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point) -
                         sizeof(packet.pack_size) +
                         SINGLE_POINT_NUM * sizeof(LivoxExtendRawPoint);
      memcpy(packet.raw_point, (void*)data->data,
             SINGLE_POINT_NUM * sizeof(LivoxExtendRawPoint));
      break;
    case PointDataType::kExtendSpherical:
      packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point) -
                         sizeof(packet.pack_size) +
                         SINGLE_POINT_NUM * sizeof(LivoxExtendSpherPoint);
      memcpy(packet.raw_point, (void*)data->data,
             SINGLE_POINT_NUM * sizeof(LivoxExtendSpherPoint));
      break;
    case PointDataType::kDualExtendCartesian:
      packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point) -
                         sizeof(packet.pack_size) +
                         DUAL_POINT_NUM * sizeof(LivoxDualExtendRawPoint);
      memcpy(packet.raw_point, (void*)data->data,
             DUAL_POINT_NUM * sizeof(LivoxDualExtendRawPoint));
      break;
    case PointDataType::kDualExtendSpherical:
      packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point) -
                         sizeof(packet.pack_size) +
                         DUAL_POINT_NUM * sizeof(LivoxDualExtendSpherPoint);
      memcpy(packet.raw_point, (void*)data->data,
             DUAL_POINT_NUM * sizeof(LivoxDualExtendSpherPoint));
      break;
    case PointDataType::kImu:
      packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point) -
                         sizeof(packet.pack_size) +
                         IMU_POINT_NUM * sizeof(LivoxImuPoint);
      memcpy(packet.raw_point, (void*)data->data,
             IMU_POINT_NUM * sizeof(LivoxImuPoint));
      break;
    case PointDataType::kTripleExtendCartesian:
      packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point) -
                         sizeof(packet.pack_size) +
                         TRIPLE_POINT_NUM * sizeof(LivoxTripleExtendRawPoint);
      memcpy(packet.raw_point, (void*)data->data,
             TRIPLE_POINT_NUM * sizeof(LivoxTripleExtendRawPoint));
      break;
    case PointDataType::kTripleExtendSpherical:
      packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point) -
                         sizeof(packet.pack_size) +
                         TRIPLE_POINT_NUM * sizeof(LivoxTripleExtendSpherPoint);
      memcpy(packet.raw_point, (void*)data->data,
             TRIPLE_POINT_NUM * sizeof(LivoxTripleExtendSpherPoint));
      break;
    default:
      break;
  }
}