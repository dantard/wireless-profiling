cmake_minimum_required(VERSION 2.8.3)
project(wireless_profiling)

set(CMAKE_CXX_FLAGS "-std=c++11 -g -ggdb")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
   FILES
   Frame.msg
   RFExplorer.msg
   XBee.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_library(radiotap src/wifi/radiotap_iter.h src/wifi/radiotap.h src/wifi/radiotap.c)

add_executable(receiver src/wifi/receiver.cpp src/wifi/common.cpp src/wifi/bridge.cpp src/wifi/layer_1.cpp src/Argon.h)
target_link_libraries(receiver yaml-cpp radiotap pcap ${catkin_LIBRARIES})
add_dependencies(receiver wifi_generate_messages_cpp radiotap ${catkin_EXPORTED_TARGETS})
add_custom_command(TARGET receiver
        POST_BUILD
        COMMAND echo "Applying sticky bit to the receiver executable..."
        COMMAND sudo chown root:root ${CMAKE_BINARY_DIR}/../devel/lib/wireless_profiling/receiver
        COMMAND sudo chmod +s ${CMAKE_BINARY_DIR}/../devel/lib/wireless_profiling/receiver
)

add_executable(emitter src/wifi/emitter.cpp src/wifi/common.cpp src/wifi/bridge.cpp src/wifi/layer_1.cpp src/Argon.h)
target_link_libraries(emitter yaml-cpp radiotap ${catkin_LIBRARIES})
add_dependencies(emitter ${catkin_EXPORTED_TARGETS})
add_custom_command(TARGET emitter
        POST_BUILD
        COMMAND echo "Applying sticky bit to the emitter executable..."
        COMMAND sudo chown root:root ${CMAKE_BINARY_DIR}/../devel/lib/wireless_profiling/emitter
        COMMAND sudo chmod +s ${CMAKE_BINARY_DIR}/../devel/lib/wireless_profiling/emitter
)

add_executable(rfexplorer src/rfexplorer/rfexplorer.cpp)
target_link_libraries(rfexplorer ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(rfexplorer wifi_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_custom_command(TARGET rfexplorer
        POST_BUILD
        COMMAND echo "Applying sticky bit to the rfexplorer executable..."
        COMMAND sudo chown root:root ${CMAKE_BINARY_DIR}/../devel/lib/wireless_profiling/rfexplorer
        COMMAND sudo chmod +s ${CMAKE_BINARY_DIR}/../devel/lib/wireless_profiling/rfexplorer
)

add_executable(xbee src/xbee/xbee.cpp)
target_link_libraries(xbee ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(xbee wifi_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_custom_command(TARGET xbee
        POST_BUILD
        COMMAND echo "Applying sticky bit to the xbee executable..."
        COMMAND sudo chown root:root ${CMAKE_BINARY_DIR}/../devel/lib/wireless_profiling/xbee
        COMMAND sudo chmod +s ${CMAKE_BINARY_DIR}/../devel/lib/wireless_profiling/xbee
)




