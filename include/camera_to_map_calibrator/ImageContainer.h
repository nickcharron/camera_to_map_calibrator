#pragma once

#include <nlohmann/json.hpp>

#include <beam_utils/filesystem.h>

class ImageContainer {
public:
  /**
   * @brief constructor requiring image path and json path
   * @param image_path full path to image file
   * @param info_path full path to json which must contain a field named
   * time_stamp which is in nsec
   */
  explicit ImageContainer(const std::string& image_path,
                          const std::string& info_path) {
    nlohmann::json J;
    BEAM_INFO("Reading image info from json: {}", info_path);
    if (!beam::ReadJson(info_path, J)) {
      throw std::runtime_error{"invalid file path"};
    }
    beam::ValidateJsonKeysOrThrow({"time_stamp"}, J);

    int64_t time_stamp_ns_int = J["time_stamp"];
    time_stamp_ns_.fromNSec(time_stamp_ns_int);

    BEAM_INFO("Reading image file: {}", image_path);
    image_ = cv::imread(image_path);
  }

  ~ImageContainer() = default;

  ros::Time GetRosTime() const { return time_stamp_ns_; }

  cv::Mat GetImage() const { return image_; }

private:
  ros::Time time_stamp_ns_;
  cv::Mat image_;
};
