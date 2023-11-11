#include "las_file.h"

#include <iostream>
#include <string>

int main(int32_t argc, char** argv) {
  std::string las_path = "/home/shitong/livox/Livox-MSDK/app/las/";
  std::vector<std::string> broadcast_code_list = {"3JEDL5T00133251"};
  std::cout << GenerateLasFile(broadcast_code_list, las_path) << std::endl;
  return 0;
}