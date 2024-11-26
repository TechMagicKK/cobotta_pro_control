cmake_minimum_required(VERSION 3.8)
project(cobotta_pro_control)

# コンパイルオプションを追加
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 必要なパッケージを見つける
find_package(ament_cmake REQUIRED)
find_package(bcap_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(bcap_service_interfaces REQUIRED)
find_package(cobotta_pro_control_interfaces REQUIRED)

# ライブラリを作成
add_library(cobotta_pro_control_lib
  src/rc9_controller.cpp
)
ament_target_dependencies(cobotta_pro_control_lib
  rclcpp rclcpp_lifecycle std_msgs bcap_core bcap_service_interfaces cobotta_pro_control_interfaces
)
target_include_directories(cobotta_pro_control_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# 実行可能ファイルを作成
add_executable(cobotta_pro_control
  src/cobotta_pro_control.cpp
)

# ライブラリをリンク
target_link_libraries(cobotta_pro_control
  cobotta_pro_control_lib
)

# インストール先を指定
install(TARGETS
  cobotta_pro_control
  cobotta_pro_control_lib
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# テストの設定
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 著作権のリントをスキップ（必要に応じてコメントアウト）
  set(ament_cmake_copyright_FOUND TRUE)
  # cpplintをスキップ（必要に応じてコメントアウト）
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# amentパッケージを宣言
ament_package()
