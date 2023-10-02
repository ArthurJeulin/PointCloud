cmake_minimum_required(VERSION 3.21.8)


project(PointCloudLearning)
find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(passthrough src/passthrough.cpp)
target_link_libraries(passthrough ${PCL_LIBRARIES})

#add_executable(cloudVisualization src/cloudVisualization.cpp)
#target_link_libraries(cloudVisualization ${PCL_LIBRARIES})

add_executable(loadPointCloud src/loadPointCloud.cpp)
target_link_libraries(loadPointCloud ${PCL_LIBRARIES})

add_executable(writePCD src/writePCD.cpp)
target_link_libraries(writePCD ${PCL_LIBRARIES})

add_executable(filter src/writePCD.cpp)
target_link_libraries(filter ${PCL_LIBRARIES})

add_executable(filterPointCloud src/filterPointCloud.cpp)
target_link_libraries(filterPointCloud ${PCL_LIBRARIES})