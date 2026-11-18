cmake_minimum_required(VERSION 3.16)
project(forcerobot_customer_sdk VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 把库路径写进 ELF RPATH，sudo 时不必再设 LD_LIBRARY_PATH
set(CMAKE_BUILD_RPATH_USE_ORIGIN ON)
set(CMAKE_SKIP_BUILD_RPATH OFF)
set(CMAKE_BUILD_WITH_INSTALL_RPATH OFF)

# 仅支持在 Linux aarch64 目标机本地编译链接
if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(WARNING "本 SDK 运行库为 Linux aarch64 ELF，请在目标 Linux 上编译链接。")
endif()

set(FRS_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(FRS_VENDOR_INC ${FRS_ROOT}/vendor/include)
set(FRS_VENDOR_LIB ${FRS_ROOT}/vendor/lib)

# 构建目录旁放一份底层 .so，便于 $ORIGIN 加载（sudo 友好）
function(frs_stage_vendor_libs dest_dir)
  file(MAKE_DIRECTORY "${dest_dir}")
  foreach(_lib libforce_robot.so libforce_robot_kinematics.so)
    add_custom_command(OUTPUT "${dest_dir}/${_lib}"
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
              "${FRS_VENDOR_LIB}/${_lib}" "${dest_dir}/${_lib}"
      DEPENDS "${FRS_VENDOR_LIB}/${_lib}"
      COMMENT "Stage ${_lib} -> ${dest_dir}"
    )
    list(APPEND _staged "${dest_dir}/${_lib}")
  endforeach()
  add_custom_target(frs_stage_libs ALL DEPENDS ${_staged})
endfunction()

frs_stage_vendor_libs("${CMAKE_BINARY_DIR}")

add_library(forcerobot_sdk SHARED
  src/forcerobot_sdk.cpp
)
add_dependencies(forcerobot_sdk frs_stage_libs)
target_include_directories(forcerobot_sdk
  PUBLIC
    $<BUILD_INTERFACE:${FRS_ROOT}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${FRS_VENDOR_INC}
)
target_compile_definitions(forcerobot_sdk PRIVATE FORCEROBOT_SDK_BUILD)
target_link_directories(forcerobot_sdk PUBLIC ${CMAKE_BINARY_DIR} ${FRS_VENDOR_LIB})
target_link_libraries(forcerobot_sdk
  PUBLIC
    force_robot
    force_robot_kinematics
    pthread
    rt
)
set_target_properties(forcerobot_sdk PROPERTIES
  OUTPUT_NAME forcerobot_sdk
  VERSION ${PROJECT_VERSION}
  SOVERSION 1
  BUILD_RPATH "\$ORIGIN"
  INSTALL_RPATH "\$ORIGIN"
)

# ---------- examples ----------
option(FRS_BUILD_EXAMPLES "Build example programs" ON)
if(FRS_BUILD_EXAMPLES)
  function(frs_add_example name)
    add_executable(${name} examples/${name}.cpp)
    add_dependencies(${name} frs_stage_libs)
    target_include_directories(${name} PRIVATE ${FRS_ROOT}/examples/common)
    target_link_libraries(${name} PRIVATE forcerobot_sdk)
    # 可执行文件与 libforcerobot_sdk.so、libforce_robot*.so 同在 build/
    set_target_properties(${name} PROPERTIES
      BUILD_RPATH "\$ORIGIN"
      INSTALL_RPATH "\$ORIGIN"
    )
  endfunction()

  # 无硬件 / FakeBus
  frs_add_example(ex_lifecycle)
  frs_add_example(ex_kinematics)
  frs_add_example(ex_kin_full)
  frs_add_example(ex_move_joint_fake)

  # 真机（ifname=fake 时可 FakeBus）
  frs_add_example(ex_hw_read)
  frs_add_example(ex_hw_joint)
  frs_add_example(ex_hw_move_joint)
  frs_add_example(ex_hw_move_tcp)
  frs_add_example(ex_hw_path)
  frs_add_example(ex_hw_path_blend)
  frs_add_example(ex_hw_tcp_path)
  frs_add_example(ex_hw_tcp_path_blend)
  frs_add_example(ex_hw_stream)
  frs_add_example(ex_hw_tcp_tool)
  frs_add_example(ex_hw_force)
  frs_add_example(ex_hw_subscribe_csv)
  frs_add_example(ex_hw_errors)
  frs_add_example(ex_hw_both)
endif()

# ---------- install layout ----------
include(GNUInstallDirs)
install(TARGETS forcerobot_sdk
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(FILES include/forcerobot_sdk.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY vendor/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/forcerobot_vendor)
install(DIRECTORY vendor/lib/ DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY config/ DESTINATION share/forcerobot/config)
install(FILES
  README.md
  MANIFEST.txt
  docs/CAPABILITY.txt
  docs/QUICKSTART.txt
  docs/EXAMPLES_MAP.txt
  docs/API_EXAMPLES.txt
  docs/RELEASE_NOTES_v1.0.txt
  DESTINATION share/doc/forcerobot)
