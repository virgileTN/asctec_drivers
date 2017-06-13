FILE(REMOVE_RECURSE
  "../src/asctec_msgs/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/asctec_msgs/msg/__init__.py"
  "../src/asctec_msgs/msg/_LLStatus.py"
  "../src/asctec_msgs/msg/_RCData.py"
  "../src/asctec_msgs/msg/_ACK.py"
  "../src/asctec_msgs/msg/_CtrlInput.py"
  "../src/asctec_msgs/msg/_IMUCalcData.py"
  "../src/asctec_msgs/msg/_ControllerOutput.py"
  "../src/asctec_msgs/msg/_IMURawData.py"
  "../src/asctec_msgs/msg/_GPSDataAdvanced.py"
  "../src/asctec_msgs/msg/_waypoint.py"
  "../src/asctec_msgs/msg/_CurrentWay.py"
  "../src/asctec_msgs/msg/_GPSData.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
