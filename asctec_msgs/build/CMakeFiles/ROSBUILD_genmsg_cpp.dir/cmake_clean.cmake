FILE(REMOVE_RECURSE
  "../src/asctec_msgs/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/asctec_msgs/LLStatus.h"
  "../msg_gen/cpp/include/asctec_msgs/RCData.h"
  "../msg_gen/cpp/include/asctec_msgs/ACK.h"
  "../msg_gen/cpp/include/asctec_msgs/CtrlInput.h"
  "../msg_gen/cpp/include/asctec_msgs/IMUCalcData.h"
  "../msg_gen/cpp/include/asctec_msgs/ControllerOutput.h"
  "../msg_gen/cpp/include/asctec_msgs/IMURawData.h"
  "../msg_gen/cpp/include/asctec_msgs/GPSDataAdvanced.h"
  "../msg_gen/cpp/include/asctec_msgs/waypoint.h"
  "../msg_gen/cpp/include/asctec_msgs/CurrentWay.h"
  "../msg_gen/cpp/include/asctec_msgs/GPSData.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
