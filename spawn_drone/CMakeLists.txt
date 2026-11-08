cmake_minimum_required(VERSION 3.8)
project(spawn_drone)

if(CMAKE_COMPILE_WARNING_AS_ERROR)
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
