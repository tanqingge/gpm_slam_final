find_package(OpenCV 3.4.5 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
list(APPEND ALL_TARGET_LIBRARIES ${OpenCV_LIBRARIES})