#pragma once

#include <pcl/visualization/pcl_visualizer.h>
#include <nlohmann/json.hpp>

#include <beam_calibration/CameraModel.h>
#include <beam_calibration/TfTree.h>
#include <beam_colorize/Colorizer.h>
#include <beam_utils/pointclouds.h>
#include <beam_containers/ImageBridge.h>

using PCLViewer = pcl::visualization::PCLVisualizer::Ptr;

class CameraToMapCalibrator {
public:
  struct Inputs {
    std::string map;
    std::string poses;
    std::string images_list;
    std::string extrinsics;
    std::string intrinsics;
    std::string output;
  };

  struct ImageMeasurements {
    ImageMeasurements() = default;
    ImageMeasurements(const nlohmann::json& J);

    std::vector<Eigen::Vector3d> map_features;
    std::vector<Eigen::Vector2d> image_features;
    Eigen::Matrix4d T_World_Baselink;
    ros::Time stamp;

    nlohmann::json ToJson() const;
  };

  explicit CameraToMapCalibrator(const Inputs& inputs);

  ~CameraToMapCalibrator() = default;

  void GetMeasurements();

  void LoadMeasurements(const std::string& filepath);

  void Solve();

private:
  void LoadMap();

  void LoadImages();

  void FillTfTrees();

  void KeyboardEventOccurred(const pcl::visualization::KeyboardEvent& event);

  void PointPickingEventOccurred(
      const pcl::visualization::PointPickingEvent& event);

  void LoadImage();

  void DrawImageFeatures();

  void AddLastImageMeasurements();

  void DisplayInstructions();

  void OutputResults();

  nlohmann::json GetTransformJson(const Eigen::Matrix4d& T) const ;

  void ViewResults();

  Inputs inputs_;

  pcl::visualization::PCLVisualizer::Ptr viewer_;
  std::shared_ptr<beam_calibration::CameraModel> camera_model_;
  PointCloud::Ptr map_ = std::make_shared<PointCloud>();
  pcl::PointXYZ min_;
  pcl::PointXYZ max_;
  std::vector<beam_containers::ImageBridge> images_;
  beam_calibration::TfTree poses_tree_;
  beam_calibration::TfTree extrinsics_;
  std::string world_frame_id_;
  std::string baselink_frame_id_;
  Eigen::Matrix4d T_Baselink_Camera_;
  Eigen::Matrix4d T_Baselink_Camera_final_;
  int image_iter_{0};
  int num_image_features_shown_;

  // user inputs
  bool next_{false};
  bool quit_{false};
  bool solve_{false};

  std::vector<ImageMeasurements> measurements_;

  // params
  double coordinateFrameScale_{0.5};
  int point_size_{3};
  std::vector<double> backgound_rgb_{0.8, 0.8, 0.8};
  Eigen::Vector3f voxel_filter_size_{0.03, 0.03, 0.03};
  double marker_radius_{0.1};
  std::string img_window_name_{"Image"};
  Eigen::Vector3f current_rgb_;
  int image_marker_size_{30};
  int image_marker_thickness_{2};
};
