#pragma once

#include <string>
#include <vector>

/**
 * @brief 保存duration时间段的点云到一个las文件
 *
 * @param las_path las文件的路径
 * @param duration 累计点云时间段
 *
 * @return -2: 未发现设备  -1: 初始化las文件失败  0: 保存las失败  1: 保存las成功
 */

int32_t GenerateLasFile(const std::vector<std::string>& broadcast_code_list,
                        const std::string& las_path, int32_t duration = 5);