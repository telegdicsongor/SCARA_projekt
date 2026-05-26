cmake_minimum_required(VERSION 3.8)
project(projekt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PixelDetection.msg"
  "msg/PixelDetectionArray.msg"
  DEPENDENCIES std_msgs
)

install(PROGRAMS
  scripts/attach_detach_controller.py
  scripts/controller_state_to_joint_states.py
  scripts/scara_sorter.py
  scripts/save_training_images.py
  scripts/static_pixel_detection_publisher.py
  scripts/yolo_cube_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  yolo26n.pt
  yolov8n.pt
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  datasets
  launch
  rviz
  runs
  urdf
  worlds
  meshes
  training_images
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest QUIET)
  if(ament_cmake_pytest_FOUND)
    ament_add_pytest_test(test_scara_sorter_math test/test_scara_sorter_math.py)
  endif()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
