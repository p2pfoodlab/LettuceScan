cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(plant)
find_package(PCL 1.8 REQUIRED)
find_package(OpenCV 3.1 REQUIRED)
find_package(Qt5Widgets REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} include)
include_directories(${OpenCV_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(cloudgen2 src/cloudgen_review.cpp src/cnpy.cpp)
target_link_libraries(cloudgen2 ${PCL_LIBRARIES} ${OpenCV_LIBS})

#add_executable(cloudgen src/cloudgen.cpp src/cnpy.cpp)
#target_link_libraries(cloudgen ${PCL_LIBRARIES} ${OpenCV_LIBS})
#add_executable(cloudreg src/cloudreg.cpp)
#target_link_libraries(cloudreg ${PCL_LIBRARIES} ${OpenCV_LIBS})
#add_executable(cloudmerge src/cloudmerge.cpp)
#target_link_libraries(cloudmerge ${PCL_LIBRARIES} ${OpenCV_LIBS})
