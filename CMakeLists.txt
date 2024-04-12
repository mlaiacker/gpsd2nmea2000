
PROJECT(nmea0183_to_nmea2000)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

include_directories(gpsd/include)
include_directories(NMEA2000/src)
include_directories(NMEA2000_socketCAN)

#add_subdirectory(NMEA2000)

file( GLOB SRCS NMEA2000_socketCAN/*.cpp NMEA2000/src/*.cpp )
#file( GLOB SRCS NMEA2000_socketCAN/*.cpp )

ADD_EXECUTABLE(${PROJECT_NAME} 
${SRCS}
src/timeconv.cpp
src/main.cpp
)

target_link_libraries(${PROJECT_NAME}
gps
pthread
rt
m
)
