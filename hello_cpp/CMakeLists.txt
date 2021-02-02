cmake_minimum_required(VERSION 2.8.3)
project(hello_cpp)
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-std=c++11 -Werror=return-type -Wall)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        nodelet
        cv_bridge
        )
find_package(nlohmann_json REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem date_time)


catkin_package(
#  INCLUDE_DIRS include
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(hello_cpp src/hello_cpp.cpp)
add_executable(hello_consul src/hello_consul.cpp)
add_executable(hello_json src/hello_json.cpp)
add_executable(hello_redis src/hello_redis.cpp)
add_executable(more_ros src/more_ros.cpp src/globals.h src/globals.cpp)
add_executable(steady src/steady.cpp)
add_executable(pvframe src/pvframe.cpp)
#add_library(hello_nodelet src/hello_nodelet.cpp src/MyNodeletClass.cpp)

# <------------ add hiredis dependency --------------->
find_path(HIREDIS_HEADER hiredis)
target_include_directories(hello_redis PUBLIC ${HIREDIS_HEADER})

find_library(HIREDIS_LIB hiredis)

# <------------ add redis-plus-plus dependency -------------->
# NOTE: this should be *sw* NOT *redis++*
find_path(REDIS_PLUS_PLUS_HEADER sw)
target_include_directories(hello_redis PUBLIC ${REDIS_PLUS_PLUS_HEADER})

find_library(REDIS_PLUS_PLUS_LIB redis++)
target_link_libraries(hello_redis ${HIREDIS_LIB} ${REDIS_PLUS_PLUS_LIB}   ${catkin_LIBRARIES} )
target_link_libraries(more_ros  ${catkin_LIBRARIES} )
target_link_libraries(steady  ${catkin_LIBRARIES} )

## Specify libraries to link a library or executable target against
target_link_libraries(hello_cpp
  ${catkin_LIBRARIES}
)
#target_link_libraries(hello_nodelet
#  ${catkin_LIBRARIES}
#)
target_link_libraries(hello_consul
  ${catkin_LIBRARIES}
)

target_include_directories(hello_json PUBLIC ${Boost_INCLUDE_DIR})
target_link_libraries(hello_json
        ${Boost_LIBRARIES}
        ${nlohmann_json_LIBRARIES}
        )

find_package(OpenSSL REQUIRED )
if (OPENSSL_FOUND)
    # Add the include directories for compiling
    target_include_directories(pvframe PUBLIC ${OPENSSL_INCLUDE_DIR})
    # Add the static lib for linking
    target_link_libraries(pvframe OpenSSL::SSL OpenSSL::Crypto)
    message(STATUS "Found OpenSSL ${OPENSSL_VERSION}")
else()
    message(STATUS "OpenSSL Not Found")
endif()

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
# install(TARGETS hello_cpp hello_cpp_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
