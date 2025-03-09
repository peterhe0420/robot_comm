include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})
include_directories("include/")
include_directories("./")
include_directories("../common/include/")
include_directories("../third-party")
include_directories("../third-party/ParamHandler")
# include_directories("../third-party/vectornav/include")
# include_directories("../third-party/vectornav/include/vn")
# include_directories("../third-party/lord_imu/Include")

include_directories("../common/include/lcm-types/cpp")
include_directories("/usr/local/include/lcm/")   # lcm includes
include_directories("/usr/local/include")
include_directories("../motion_controller/include")
include_directories("../motion_controller")
include_directories("../motion_ros_interface/include")
set(CMAKE_CXX_STANDARD 17)

file(GLOB sources "src/*.cpp" "src/rt/*.c"  "src/rt/*.cpp" "*.cpp" )

add_library(robot SHARED ${sources})

#add end
# target_link_libraries(robot biomimetics pthread lcm inih dynacore_param_handler lord_imu soem motion_ros_interface user_command)
target_link_libraries(robot biomimetics pthread lcm dynacore_param_handler  motion_ros_interface gflags glog JetsonGPIO)

# if(CMAKE_SYSTEM_NAME MATCHES Linux)
# target_link_libraries( rt)
# # target_link_libraries(robot libvnc rt)
# endif()
