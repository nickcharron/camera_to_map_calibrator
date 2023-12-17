#include <camera_to_map_calibrator/CameraToMapCalibrator.h>

#include <filesystem>

#include <pcl/common/transforms.h>

#include <beam_filtering/VoxelDownsample.h>
#include <beam_mapping/Poses.h>
#include <beam_optimization/CamPoseReprojectionCost.h>
#include <beam_optimization/CeresParams.h>
#include <beam_utils/se3.h>

std::vector<Eigen::Vector2d> _pixels_selected;

static void ImageMouseHandler(int event, int x, int y, int flags, void* img) {
  if (event == cv::EVENT_RBUTTONDOWN) {
    _pixels_selected.emplace_back(x, y);
    BEAM_INFO("Adding pixel feature: [{}, {}]", x, y);
    return;
  }
}

CameraToMapCalibrator::CameraToMapCalibrator(const Inputs& inputs)
    : inputs_(inputs) {
  camera_model_ = beam_calibration::CameraModel::Create(inputs_.intrinsics);
  FillTfTrees();
  T_Baselink_Camera_ =
      extrinsics_
          .GetTransformEigen(baselink_frame_id_, camera_model_->GetFrameID())
          .matrix();
  BEAM_INFO("Done initializing CameraToMapCalibrator");
}

CameraToMapCalibrator::ImageMeasurements::ImageMeasurements(
    const nlohmann::json& J) {
  beam::ValidateJsonKeysOrThrow(
      {"map_features", "image_features", "T_World_Baselink", "timestamp_ns"},
      J);
  std::vector<std::vector<double>> J_map_features = J["map_features"];
  for (const auto& v : J_map_features) {
    map_features.emplace_back(v[0], v[1], v[2]);
  }
  std::vector<std::vector<double>> J_image_features = J["image_features"];
  for (const auto& v : J_image_features) {
    image_features.emplace_back(v[0], v[1]);
  }
  std::vector<double> T_vec = J["T_World_Baselink"];
  T_World_Baselink = beam::VectorToEigenTransform(T_vec);
  stamp.fromNSec(J["timestamp_ns"]);
}

nlohmann::json CameraToMapCalibrator::ImageMeasurements::ToJson() const {
  std::vector<std::vector<double>> map_features_vec;
  for (const auto& f : map_features) {
    map_features_vec.push_back({f[0], f[1], f[2]});
  }

  std::vector<std::vector<double>> img_features_vec;
  for (const auto& f : image_features) {
    img_features_vec.push_back({f[0], f[1]});
  }

  nlohmann::json J;
  J["map_features"] = map_features_vec;
  J["image_features"] = img_features_vec;
  J["T_World_Baselink"] = beam::EigenTransformToVector(T_World_Baselink);
  J["timestamp_ns"] = stamp.toNSec();
  return J;
}

void CameraToMapCalibrator::LoadMap() {
  BEAM_INFO("Loading map from {}", inputs_.map);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(inputs_.map, *map_) == -1) {
    BEAM_ERROR("Couldn't read map file");
  }
  beam::getMinMax3D(*map_, min_, max_);
  BEAM_INFO("Done loading map of size: {}", map_->size());

  beam_filtering::VoxelDownsample voxel_filter(voxel_filter_size_);
  voxel_filter.SetInputCloud(map_);
  voxel_filter.Filter();
  *map_ = voxel_filter.GetFilteredCloud();
  BEAM_INFO("Filtered map to size: {}", map_->size());
}

void CameraToMapCalibrator::LoadImages() {
  BEAM_INFO("Reading image paths from : {}", inputs_.images_list);
  nlohmann::json J;
  if (!beam::ReadJson(inputs_.images_list, J)) {
    throw std::runtime_error{"invalid json file path"};
  }
  beam::ValidateJsonKeysOrThrow({"Images"}, J);
  std::vector<std::string> image_filenames = J["Images"];

  std::filesystem::path images_root =
      std::filesystem::path(inputs_.images_list).parent_path();

  for (const auto& image_filename : image_filenames) {
    std::filesystem::path image_path =
        images_root / std::filesystem::path(image_filename);
    images_.push_back(beam_containers::ImageBridge());
    images_.back().LoadFromJSON(image_path);
  }
  BEAM_INFO("Loaded {} images", images_.size());
}

void CameraToMapCalibrator::FillTfTrees() {
  // Load previous poses file specified in labeler json
  BEAM_INFO("Loading poses form {}", inputs_.poses);
  beam_mapping::Poses poses_container;
  poses_container.LoadFromFile(inputs_.poses);
  const auto& poses = poses_container.GetPoses();
  const auto& timestamps = poses_container.GetTimeStamps();
  world_frame_id_ = poses_container.GetFixedFrame();
  baselink_frame_id_ = poses_container.GetMovingFrame();

  BEAM_INFO("Filling TF tree with {} poses", poses.size());
  for (int i = 0; i < poses.size(); i++) {
    Eigen::Affine3d T(poses[i]);
    poses_tree_.AddTransform(T, world_frame_id_, baselink_frame_id_,
                             timestamps[i]);
  }

  BEAM_INFO("Filling TF tree with extrinsic from {}", inputs_.extrinsics);
  extrinsics_.LoadJSON(inputs_.extrinsics);
}

void CameraToMapCalibrator::GetMeasurements() {
  viewer_ = std::make_shared<pcl::visualization::PCLVisualizer>();
  LoadMap();
  LoadImages();

  std::function<void(const pcl::visualization::PointPickingEvent&)> point_cb =
      [this](const pcl::visualization::PointPickingEvent& event) {
        PointPickingEventOccurred(event);
      };

  std::function<void(const pcl::visualization::KeyboardEvent&)> keyboard_cb =
      [this](const pcl::visualization::KeyboardEvent& event) {
        KeyboardEventOccurred(event);
      };
  viewer_->registerKeyboardCallback(keyboard_cb);
  viewer_->registerPointPickingCallback(point_cb);
  LoadImage();
  DisplayInstructions();

  while (!viewer_->wasStopped()) {
    viewer_->spinOnce(10);
    if (solve_) { break; }
    if (quit_) {
      BEAM_INFO("Quitting CameraToMapCalibrator");
      return;
    }
    if (next_) {
      next_ = false;
      image_iter_++;
      if (image_iter_ >= images_.size()) {
        BEAM_INFO("Reached the end of the images list");
        break;
      }
      AddLastImageMeasurements();
      LoadImage();
      DisplayInstructions();
    } else {
      DrawImageFeatures();
    }
  }
  AddLastImageMeasurements();
  BEAM_INFO("Done getting measurements");
}

void CameraToMapCalibrator::KeyboardEventOccurred(
    const pcl::visualization::KeyboardEvent& event) {
  if (event.getKeySym() == "n" && event.keyDown()) {
    next_ = true;
  } else if (event.getKeySym() == "s" && event.keyDown()) {
    solve_ = true;
  } else if (event.getKeySym() == "q" && event.keyDown()) {
    quit_ = true;
  } else if (event.getKeySym() == "m" && event.keyDown()) {
    // remove last map feature
    auto& m = measurements_.back();
    auto& f = m.map_features.back();
    BEAM_INFO("Removing last map feature: [{}, {}, {}]", f[0], f[1], f[2]);
    std::string id = std::to_string(image_iter_) + "." +
                     std::to_string(m.map_features.size());
    viewer_->removeShape(id);
    m.map_features.pop_back();
  } else if (event.getKeySym() == "i" && event.keyDown()) {
    // remove last image feature
    if (_pixels_selected.empty()) { return; }
    BEAM_INFO("Removing last image feature: [{}, {}]",
              _pixels_selected.back()[0], _pixels_selected.back()[1]);
    _pixels_selected.pop_back();
    DrawImageFeatures();
  }
}

void CameraToMapCalibrator::DisplayInstructions() {
  std::cout << "\nPress 's' to finish picking features and solve\n"
            << "Press 'q' to quit the program\n"
            << "Press 'n' to go to next image\n"
            << "Press 'i' to remove last image feature\n"
            << "Press 'm' to remove last map feature\n"
            << "SHIFT Click to select a point\n";
}

void CameraToMapCalibrator::DrawImageFeatures() {
  if (num_image_features_shown_ == _pixels_selected.size()) { return; }
  cv::Mat marked_image = images_.at(image_iter_).GetBGRImage().clone();
  for (const auto& m : _pixels_selected) {
    cv::Point p(m[0], m[1]);
    BEAM_INFO("drawing marker: [{}, {}]", m[0], m[1]);
    cv::drawMarker(marked_image, p,
                   cv::Scalar(current_rgb_[2] * 255, current_rgb_[1] * 255,
                              current_rgb_[0] * 255),
                   cv::MARKER_CROSS, image_marker_size_,
                   image_marker_thickness_);
  }
  num_image_features_shown_ = _pixels_selected.size();
  cv::imshow(img_window_name_, marked_image);
}

void CameraToMapCalibrator::PointPickingEventOccurred(
    const pcl::visualization::PointPickingEvent& event) {
  if (event.getPointIndex() == -1) { return; }
  float x, y, z;
  event.getPoint(x, y, z);
  BEAM_INFO("selected point [{}, {}, {}]", x, y, z);

  // add measurements
  auto& m = measurements_.back();
  Eigen::Vector3d f(x, y, z);
  m.map_features.push_back(f);
  pcl::PointXYZ p(x, y, z);
  std::string id =
      std::to_string(image_iter_) + "." + std::to_string(m.map_features.size());
  viewer_->addSphere(p, marker_radius_, current_rgb_[0], current_rgb_[1],
                     current_rgb_[2], id);
}

void CameraToMapCalibrator::AddLastImageMeasurements() {
  measurements_.back().image_features = _pixels_selected;
  _pixels_selected.clear();
}

void CameraToMapCalibrator::LoadImage() {
  // check that the last image had equal number of image & map features
  if (!measurements_.empty()) {
    const auto& m = measurements_.back();
    if (m.map_features.size() != m.image_features.size()) {
      BEAM_CRITICAL(
          "number of map features does not match number of image features");
      throw std::runtime_error{"invalid measurements"};
    }
  }

  current_rgb_[0] = beam::randf(1, 0);
  current_rgb_[1] = beam::randf(1, 0);
  current_rgb_[2] = beam::randf(1, 0);

  // load image
  BEAM_INFO("Loading image: {}", image_iter_);
  const beam_containers::ImageBridge& image_container = images_.at(image_iter_);
  const ros::Time image_time = image_container.GetRosTime();

  // load pose
  Eigen::Matrix4d T_World_Baselink =
      poses_tree_
          .GetTransformEigen(world_frame_id_, baselink_frame_id_, image_time)
          .matrix();
  Eigen::Matrix4d T_World_Camera = T_World_Baselink * T_Baselink_Camera_;
  Eigen::Matrix4d T_Camera_World = beam::InvertTransform(T_World_Camera);

  // remove last measurement if no features were saved
  if (!measurements_.empty() && measurements_.back().map_features.empty()) {
    measurements_.pop_back();
  }

  ImageMeasurements m;
  m.stamp = image_time;
  m.T_World_Baselink = T_World_Baselink;
  measurements_.push_back(m);

  // crop scan
  auto map_cropped = std::make_shared<PointCloudCol>();
  const uint32_t h = camera_model_->GetHeight();
  const uint32_t w = camera_model_->GetWidth();
  const double r_max = 1 * std::sqrt(h * h + w * w);
  for (const auto& p : map_->points) {
    Eigen::Vector4d p_in_world(p.x, p.y, p.x, 1);
    Eigen::Vector4d p_in_cam = T_Camera_World * p_in_world;
    if (p_in_cam[2] < 0) { continue; }
    Eigen::Vector2d pixels;
    if (!camera_model_->ProjectPoint(p_in_cam.hnormalized(), pixels)) {
      continue;
    }

    // keep points that project 'near' the image plane
    double rx = pixels[0] - w / 2;
    double ry = pixels[1] - h / 2;
    double r = std::sqrt(rx * rx + ry * ry);
    if (r > r_max) { continue; }

    pcl::PointXYZRGB p_col;
    p_col.x = p.x;
    p_col.y = p.y;
    p_col.z = p.z;
    p_col.r = static_cast<uint8_t>((p.x - min_.x) / (max_.x - min_.x) * 255);
    p_col.g = static_cast<uint8_t>((p.y - min_.y) / (max_.y - min_.y) * 255);
    p_col.b = static_cast<uint8_t>((p.z - min_.z) / (max_.z - min_.z) * 255);
    map_cropped->points.push_back(p_col);
  }

  // update pcl viewer
  viewer_->removePointCloud("Map");
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(
      map_cropped);
  viewer_->addPointCloud<pcl::PointXYZRGB>(map_cropped, rgb, "Map");

  viewer_->removePointCloud("Frustum");
  pcl::PointCloud<pcl::PointXYZRGBL> frustum_in_camera =
      camera_model_->CreateCameraFrustum(image_time);
  auto frustum_in_map = std::make_shared<pcl::PointCloud<pcl::PointXYZRGBL>>();
  pcl::transformPointCloud(frustum_in_camera, *frustum_in_map,
                           Eigen::Affine3d(T_World_Camera));
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBL> rgb2(
      frustum_in_map);
  viewer_->addPointCloud<pcl::PointXYZRGBL>(frustum_in_map, rgb2, "Frustum");

  // display image
  cv::namedWindow(img_window_name_);
  cv::imshow(img_window_name_, images_.at(image_iter_).GetBGRImage());
  cv::setMouseCallback(img_window_name_, ImageMouseHandler, 0);
  cv::startWindowThread();
}

void CameraToMapCalibrator::Solve() {
  BEAM_INFO("Solving for calibration");

  // remove last measurement if no features were saved
  if (measurements_.back().map_features.empty()) { measurements_.pop_back(); }

  // setup problem & params
  beam_optimization::CeresParams ceres_params;
  ceres_params.GetSolverOptionsMutable().minimizer_progress_to_stdout = true;
  std::shared_ptr<ceres::Problem> problem =
      std::make_shared<ceres::Problem>(ceres_params.ProblemOptions());
  std::unique_ptr<ceres::LossFunction> loss_function =
      ceres_params.LossFunction();
  std::unique_ptr<ceres::LocalParameterization> parameterization =
      ceres_params.SE3QuatTransLocalParametrization();

  // add calibration
  Eigen::Matrix4d T_Camera_Baselink = beam::InvertTransform(T_Baselink_Camera_);
  Eigen::Matrix3d R = T_Camera_Baselink.block(0, 0, 3, 3);
  Eigen::Vector3d t = T_Camera_Baselink.block(0, 3, 3, 1);
  Eigen::Quaterniond q(R);
  std::vector<double> result{q.w(), q.x(), q.y(), q.z(), t[0], t[1], t[2]};
  problem->AddParameterBlock(&(result[0]), 7, parameterization.get());

  // add residuals. First iterate through all images
  for (const ImageMeasurements& m : measurements_) {
    Eigen::Matrix4d T_Baselink_World =
        beam::InvertTransform(m.T_World_Baselink);
    if (m.map_features.size() != m.image_features.size()) {
      throw std::runtime_error{
          "number of image features must match number of image features"};
    }
    // now iterate through all features for this image
    for (int k = 0; k < m.map_features.size(); k++) {
      const Eigen::Vector3d& p_in_world = m.map_features.at(k);
      Eigen::Vector4d p_in_baselink =
          T_Baselink_World * p_in_world.homogeneous();
      Eigen::Vector2d pixels(m.image_features.at(k)[0],
                             m.image_features.at(k)[1]);
      std::unique_ptr<ceres::CostFunction> reproj_cost(
          beam_optimization::CeresReprojectionCostFunction::Create(
              pixels, p_in_baselink.hnormalized(), camera_model_, 1));
      problem->AddResidualBlock(reproj_cost.release(), loss_function.get(),
                                &(result[0]));
    }
  }

  // solve
  ceres::Solver::Summary ceres_summary;
  ceres::Solve(ceres_params.SolverOptions(), problem.get(), &ceres_summary);

  std::string report = ceres_summary.FullReport();
  std::cout << "\n\nCeres Report:\n\n" << report << "\n";
  if (!ceres_summary.IsSolutionUsable()) {
    BEAM_ERROR("Invalid calibration refinement solution");
  } else {
    BEAM_INFO("OPTIMIZATION SUCCESSFUL!");
  }

  Eigen::Quaterniond q_final;
  q_final.w() = result[0];
  q_final.x() = result[1];
  q_final.y() = result[2];
  q_final.z() = result[3];
  Eigen::Matrix3d R_final(q_final);
  Eigen::Matrix4d T_Camera_Baselink_final = Eigen::Matrix4d::Identity();
  T_Camera_Baselink_final.block(0, 0, 3, 3) = R_final;
  T_Camera_Baselink_final(0, 3) = result[4];
  T_Camera_Baselink_final(1, 3) = result[5];
  T_Camera_Baselink_final(2, 3) = result[6];
  T_Baselink_Camera_final_ = beam::InvertTransform(T_Camera_Baselink_final);
  BEAM_INFO("Done solving for calibration");

  OutputResults();
  ViewResults();
}

void CameraToMapCalibrator::OutputResults() {
  BEAM_INFO("Outputting results");
  nlohmann::json J;

  Eigen::Matrix4d T_diff =
      T_Baselink_Camera_final_ * beam::InvertTransform(T_Baselink_Camera_);

  J["from_frame"] = camera_model_->GetFrameID();
  J["to_frame"] = baselink_frame_id_;
  J["original"] = GetTransformJson(T_Baselink_Camera_);
  J["optimized"] = GetTransformJson(T_Baselink_Camera_final_);
  J["difference"] = GetTransformJson(T_diff);

  std::vector<nlohmann::json> J_measurements;
  for (const auto& m : measurements_) { J_measurements.push_back(m.ToJson()); }
  J["measurements"] = J_measurements;

  BEAM_INFO("Saving results to: {}", inputs_.output);
  std::ofstream filejson(inputs_.output);
  filejson << std::setw(4) << J << std::endl;

  BEAM_INFO("Done outputting results");
}

nlohmann::json
    CameraToMapCalibrator::GetTransformJson(const Eigen::Matrix4d& T) const {
  Eigen::Matrix3d R = T.block(0, 0, 3, 3);
  Eigen::Vector3d t = T.block(0, 3, 3, 1);
  Eigen::Vector3d rpy = R.eulerAngles(0, 1, 2);
  Eigen::Quaterniond q(R);

  nlohmann::json J;
  J["T"] = beam::EigenTransformToVector(T);
  J["txyz_m"] = std::vector<double>{t[0], t[1], t[2]};
  J["qwxyz"] = std::vector<double>{q.w(), q.x(), q.y(), q.z()};
  J["RPY_deg"] = std::vector<double>{rpy[0], rpy[1], rpy[2]};
  return J;
}

void CameraToMapCalibrator::LoadMeasurements(const std::string& filepath) {
  nlohmann::json J;
  BEAM_INFO("Reading measurements from {}", filepath);
  if (!beam::ReadJson(filepath, J)) {
    BEAM_CRITICAL("Cannot read measurements file: {}", filepath);
    throw std::runtime_error{"invalid file path"};
  }
  beam::ValidateJsonKeysOrThrow({"measurements"}, J);
  for (const nlohmann::json& J_m : J["measurements"]) {
    measurements_.emplace_back(J_m);
  }
}

void CameraToMapCalibrator::ViewResults() {
  // images will be empty if loading measurements from results file
  if (images_.empty()) { LoadImages(); }

  for (int i = 0; i < images_.size(); i++) {
    ros::Time t = images_.at(i).GetRosTime();
    int measurement_id = -1;
    for (int k = 0; k < measurements_.size(); k++) {
      if (measurements_.at(k).stamp == t) {
        measurement_id = k;
        break;
      }
    }
    if (measurement_id == -1) { continue; }

    std::cout << "Displaying image " << i << "\n";
    std::cout << "Press any key to continue\n";
    cv::Mat image = images_.at(i).GetBGRImage().clone();

    const auto& m = measurements_.at(measurement_id);
    Eigen::Matrix4d T_Camera_World_Init =
        beam::InvertTransform(m.T_World_Baselink * T_Baselink_Camera_);
    Eigen::Matrix4d T_Camera_World_Opt =
        beam::InvertTransform(m.T_World_Baselink * T_Baselink_Camera_final_);

    // draw keypoints
    for (int k = 0; k < m.map_features.size(); k++) {
      const auto& p_in_map = m.map_features.at(k).homogeneous();
      Eigen::Vector3d p_in_cam_init =
          (T_Camera_World_Init * p_in_map).hnormalized();
      Eigen::Vector3d p_in_cam_opt =
          (T_Camera_World_Opt * p_in_map).hnormalized();
      bool in_image;
      Eigen::Vector2d pixel_initial;
      camera_model_->ProjectPoint(p_in_cam_init, pixel_initial, in_image);
      if (in_image) {
        // change from [u, v] to [v, u]
        double u = pixel_initial[0];
        double v = pixel_initial[1];
        BEAM_INFO("Drawing initial feature at [{}, {}]", v, u);
        cv::drawMarker(image, cv::Point(u, v), cv::Scalar(0, 0, 255),
                       cv::MARKER_CROSS, image_marker_size_,
                       image_marker_thickness_);
      } else {
        BEAM_ERROR("initial feature {} did not project into image", k);
      }
      Eigen::Vector2d pixel_opt;
      camera_model_->ProjectPoint(p_in_cam_opt, pixel_opt, in_image);
      if (in_image) {
        // change from [u, v] to [v, u]
        double u = pixel_opt[0];
        double v = pixel_opt[1];
        BEAM_INFO("Drawing optimized feature at [{}, {}]", u, v);
        cv::drawMarker(image, cv::Point(u, v), cv::Scalar(0, 255, 0),
                       cv::MARKER_CROSS, image_marker_size_ * 1.3,
                       image_marker_thickness_);
      } else {
        BEAM_ERROR("optimized feature {} did not project into image", k);
      }

      const Eigen::Vector2d& pixel_label = m.image_features.at(k);
      cv::drawMarker(image, cv::Point(pixel_label[0], pixel_label[1]),
                     cv::Scalar(255, 0, 0), cv::MARKER_CROSS,
                     image_marker_size_, image_marker_thickness_);
    }
    cv::namedWindow(img_window_name_);
    cv::imshow(img_window_name_, image);

    cv::waitKey();
  }
}