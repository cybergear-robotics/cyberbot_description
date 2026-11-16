cmake_minimum_required(VERSION 3.5)
project(cyberbot_description)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

install(
  DIRECTORY launch sdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/body_pose.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME body_pose
)

install(PROGRAMS scripts/odom_tf.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME odom_tf
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(description test/test_description.py)
  ament_add_pytest_test(body_pose test/test_body_pose.py)
  ament_add_pytest_test(odom_tf test/test_odom_tf.py)
endif()

ament_package()
