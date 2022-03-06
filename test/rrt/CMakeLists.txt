# RRT test
set(CMAKE_BINARY_DIR ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/sai2-planning)
add_executable(rrt_test rrt_test.cpp)
target_link_libraries(rrt_test sai2-planning::sai2-planning ${SAI_COMMON_LIBRARIES})

# Copy files
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY joint_robot.urdf DESTINATION ${APP_RESOURCE_DIR})
