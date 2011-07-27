include_directories(${CMAKE_CURRENT_SOURCE_DIR})

ectomodule(ecto_corrector 
  ecto_corrector.cpp
  Corrector.cpp
  DetectionSimulator.cpp)
link_ecto(ecto_corrector
  ${sensor_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${pose_corrector_LIBRARIES}
  )