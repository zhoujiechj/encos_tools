cmake_minimum_required(VERSION 3.8)   # 版本号≤你本地 cmake 版本就行
project(thirdparty)                    # 包名随意，和文件夹一致最直观
find_package(ament_cmake REQUIRED)  

add_subdirectory(soem)
