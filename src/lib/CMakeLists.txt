# Core inspection library
add_library(${PROJECT_NAME}
  CameraToMapCalibrator.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    ../../include
)
target_link_libraries(${PROJECT_NAME}
  beam::calibration 
  beam::colorize
  beam::utils
  beam::mapping
  beam::optimization
  PCL::PCL
  OpenCV::OpenCV
  nlohmann_json::nlohmann_json
)
