find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(assimp REQUIRED)

add_executable(rmagine_sensors main.cpp)
target_link_libraries(rmagine_sensors
    rmagine::core
    rmagine::optix
    ${ASSIMP_LIBRARIES}
)
install(TARGETS rmagine_sensors
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    COMPONENT optix
)
ament_target_dependencies(rmagine_sensors rclcpp sensor_msgs visualization_msgs interactive_markers)
ament_package()
