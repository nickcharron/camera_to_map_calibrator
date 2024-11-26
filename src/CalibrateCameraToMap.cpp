#include <gflags/gflags.h>

#include <beam_utils/gflags.h>
#include <camera_to_map_calibrator/CameraToMapCalibrator.h>

DEFINE_string(map, "",
              "Full path to map (Required). Must be a PCD and all points must "
              "be in the same frame as the poses' fixed frame");
DEFINE_validator(map, &beam::gflags::ValidateFileMustExist);

DEFINE_string(
    poses, "",
    "Full path poses file (Required). See pose file types in "
    "libbeam/beam_mapping/poses.h. Or extract poses using 3d_map_builder");
DEFINE_validator(poses, &beam::gflags::ValidateJsonFileMustExist);

DEFINE_string(
    images_list, "",
    "Full path to images list json. The should have a list of names of the "
    "subfolders in which all images are saved. Note that each subfolder must "
    "contain an image file by the name defined by images_filename, as well as "
    "a json file with the name ImageInfo.json. See example data in tutorial "
    "for a example to follow. The json just needs the time_stamp (ns) key");
DEFINE_validator(images_list, &beam::gflags::ValidateDirMustExist);

DEFINE_string(images_filename, "BGRImage.jpg",
              "Name of image file in image container folder. See images_list "
              "flag description above");

DEFINE_string(extrinsics, "",
              "Full path to extrinsics json (Required). See "
              "beam_robotics/calibration for file formats");
DEFINE_validator(extrinsics, &beam::gflags::ValidateJsonFileMustExist);

DEFINE_string(map_sensor_frame, "",
              "Frame ID of the sensor frame used to generate the map");
DEFINE_validator(map_sensor_frame, &beam::gflags::ValidateCannotBeEmpty);

DEFINE_string(intrinsics, "",
              "Full path to intrinsics json (Required). See "
              "beam_robotics/calibration for file formats");
DEFINE_validator(intrinsics, &beam::gflags::ValidateJsonFileMustExist);

DEFINE_string(output, "",
              "Full path to output file (Required). Example: "
              "/home/user/new_extrinsics.json");
DEFINE_validator(output, &beam::gflags::ValidateMustBeJson);

DEFINE_double(image_downsample_factor, 2,
              "How much to downsample images before display. Default: 2");

DEFINE_string(
    measurements, "",
    "[optional] if a measurements path is provided, we will use "
    "these measurements and re-solve. This should be a json file with "
    "the same format as the output of this pipeline");

// EXAMPLE COMMAND
// ./build/camera_to_map_calibrator/camera_to_map_calibrator_main -extrinsics
// ~/beam_robotics/calibration/results/inspector_gadget2/current/extrinsics/extrinsics.json
// -images_list
// ~/d/results/image_extractor/F1_link/selected_images.json
// -map ~/d/results/map_builder/map.pcd
// -output
// ~/d/results/calibration_results.json
// -poses
// ~/d/results/map_builder/final_poses.json
// -intrinsics
// ~/beam_robotics/calibration/results/inspector_gadget2/current/intrinsics/F1.json
// -map_sensor_frame lidar_v_link

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  CameraToMapCalibrator::Inputs inputs{
      .map = FLAGS_map,
      .poses = FLAGS_poses,
      .images_list = FLAGS_images_list,
      .images_filename = FLAGS_images_filename,
      .extrinsics = FLAGS_extrinsics,
      .intrinsics = FLAGS_intrinsics,
      .map_sensor_frame = FLAGS_map_sensor_frame,
      .output = FLAGS_output,
      .image_downsample_factor = FLAGS_image_downsample_factor};
  CameraToMapCalibrator calibrator(inputs);
  if (FLAGS_measurements.empty()) {
    calibrator.GetMeasurements();
  } else {
    calibrator.LoadMeasurements(FLAGS_measurements);
  }
  calibrator.Solve();

  return 0;
}