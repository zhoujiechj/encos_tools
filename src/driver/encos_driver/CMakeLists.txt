cmake_minimum_required(VERSION 3.8)
project(encos_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# ++++++++++++++++++++++++++++++++++++++++++++++++++++
# 生成 ROS 2 消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorClearBuff.msg"
  "msg/MotorGetId.msg"
  "msg/MotorGetParam.msg"
  "msg/MotorResetId.msg"
  "msg/MotorSetId.msg"
  "msg/MotorSetParam.msg"
  "msg/MotorSetPos.msg"
  "msg/MotorSetSpeed.msg"
  "msg/MotorSetTorCur.msg"
  "msg/MotorSetTorPos.msg"
  "msg/MotorSetZero.msg"
  "msg/MotorStop.msg"
  DEPENDENCIES std_msgs
)

# 导出 ROS 2 消息
ament_export_dependencies(rosidl_default_runtime std_msgs)

# 获取 C++ 类型支持目标
rosidl_get_typesupport_target(cpp_typesupport_target 
  ${PROJECT_NAME} 
  rosidl_typesupport_cpp
)

rosidl_get_typesupport_target(cpp_fastrtps_typesupport_target 
  ${PROJECT_NAME} 
  rosidl_typesupport_fastrtps_cpp
)

rosidl_get_typesupport_target(cpp_introspection_typesupport_target 
  ${PROJECT_NAME} 
  rosidl_typesupport_introspection_cpp
)

# ++++++++++++++++++++++++++++++++++++++++++++++++++++
# 添加第三方库
set(THIRD_PARTY_DIR "${CMAKE_SOURCE_DIR}/../../thirdparty")
add_subdirectory(${THIRD_PARTY_DIR}/soem ${CMAKE_BINARY_DIR}/soem)

# 添加库，共享库(.so)用 SHARED；静态库(.a)用 STATIC
add_library(encos_driver_lib STATIC
  src/transmit.cpp
  src/motor_control.cpp
  src/math_ops.cpp
)

# 包含头文件
target_include_directories(encos_driver_lib 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    $<INSTALL_INTERFACE:include/soem>
  PRIVATE
    ${THIRD_PARTY_DIR}/soem/soem
    ${THIRD_PARTY_DIR}/soem/osal
    ${THIRD_PARTY_DIR}/soem/osal/linux
    ${THIRD_PARTY_DIR}/soem/oshw/linux
)

# 链接 ROS 2 依赖
ament_target_dependencies(encos_driver_lib rclcpp std_msgs)

# 让外界 find_package(encos_driver) 时能拿到头文件+库+依赖
ament_export_targets(export_encos_driver_lib HAS_LIBRARY_TARGET)

# 
ament_export_dependencies(rclcpp std_msgs rosidl_default_runtime)

# ++++++++++++++++++++++++++++++++++++++++++++++++++++
# 添加可执行文件
add_executable(ec_server 
  example/ec_server.cpp
)

set_target_properties(ec_server PROPERTIES
    INSTALL_RPATH "${CMAKE_SOURCE_DIR}/../../../install/encos_driver/lib;"
    BUILD_WITH_INSTALL_RPATH TRUE
)

# 包含头文件
target_include_directories(ec_server 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/example>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  PRIVATE
    ${THIRD_PARTY_DIR}/soem/soem
    ${THIRD_PARTY_DIR}/soem/osal
    ${THIRD_PARTY_DIR}/soem/osal/linux
    ${THIRD_PARTY_DIR}/soem/oshw/linux
)

# 链接库
target_link_libraries(ec_server 
  encos_driver_lib
  soem
  ${cpp_typesupport_target}
  ${cpp_fastrtps_typesupport_target}
  ${cpp_introspection_typesupport_target}
)
target_link_options(ec_server PRIVATE "-Wl,--no-as-needed")

# 链接 ROS 2 依赖
ament_target_dependencies(ec_server rclcpp std_msgs rosidl_default_runtime)

# ++++++++++++++++++++++++++++++++++++++++++++++++++++
# 添加可执行文件
add_executable(ec_client 
  example/ec_client.cpp
)

# 包含头文件
target_include_directories(ec_client 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/example>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)

# 链接库
target_link_libraries(ec_client 
  ${cpp_typesupport_target}
)
# target_link_options(ec_client PRIVATE "-Wl,--no-as-needed")

# 链接 ROS 2 依赖
ament_target_dependencies(ec_client rclcpp std_msgs)

# ++++++++++++++++++++++++++++++++++++++++++++++++++++
# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 安装第三方库头文件
install(DIRECTORY ${THIRD_PARTY_DIR}/soem/soem/
  DESTINATION include/soem
  FILES_MATCHING PATTERN "*.h"
)

# 安装库
install(TARGETS encos_driver_lib
  EXPORT export_encos_driver_lib          # 关键：生成导出文件
  ARCHIVE DESTINATION lib       # 静态库
  LIBRARY DESTINATION lib       # 共享库
  RUNTIME DESTINATION bin       # Windows dll
)

# 安装可执行文件
install(TARGETS ec_server
  DESTINATION lib/${PROJECT_NAME}
)

# 安装可执行文件
install(TARGETS ec_client
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()