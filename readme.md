# liblas
编译安装 https://github.com/libLAS/libLAS.git

# 测试
修改main.cc中的broadcast_code_list

# 编译
cmake -B build -S .
cd build && make

# 运行
./build/las_file
