cmake_minimum_required(VERSION 2.8.3)
project(hello_cpp)
add_compile_options(-std=c++11 -Werror=return-type -Wall)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        nodelet
        )
find_package(nlohmann_json REQUIRED)

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
add_library(hello_nodelet src/hello_nodelet.cpp src/MyNodeletClass.cpp)
add_executable(hello_consul src/hello_consul.cpp)
add_executable(hello_json src/hello_json.cpp)
add_executable(hello_redis src/hello_redis.cpp)

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

## Specify libraries to link a library or executable target against
target_link_libraries(hello_cpp
  ${catkin_LIBRARIES}
)
target_link_libraries(hello_nodelet
  ${catkin_LIBRARIES}
)
target_link_libraries(hello_consul
  ${catkin_LIBRARIES}
)

target_link_libraries(hello_json
  ${nlohmann_json_LIBRARIES}
)

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
