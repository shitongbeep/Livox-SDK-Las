#pragma once

#include <string>
#include <vector>

/**
 * @brief 保存duration时间段的livox数据包到一个las点云
 *
 * @param broadcast_code_list livox对应的broadcast_code，仅支持一个
 * @param las_path las点云的路径
 * @param las_name las点云的文件名
 * @param duration 累计点云时间段
 *
 * @return -2: 未发现设备  -1: 初始化las文件失败  0: 保存las失败  1: 保存las成功
 */

int32_t GenerateLasFile(const std::vector<std::string>& broadcast_code_list,
                        const std::string& las_path,
                        const std::string& las_name = "pointcloud.las",
                        int32_t duration = 5);
