#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <opencv2/opencv.hpp>

#include "tools/img_tools.hpp"

const std::string keys =
  "{help h usage ? |                          | 输出命令行参数说明}"
  "{config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
  "{@input-folder  | assets/img_with_q        | 输入文件夹路径   }";

std::vector<cv::Point3f> centers_3d(const cv::Size & pattern_size, const float center_distance)
{
  std::vector<cv::Point3f> centers_3d;

  for (int i = 0; i < pattern_size.height; i++)
    for (int j = 0; j < pattern_size.width; j++)
      centers_3d.push_back({j * center_distance, i * center_distance, 0});

  return centers_3d;
}

namespace fs = std::filesystem;

struct LoadResult
{
  size_t total_images = 0;
  size_t detected_images = 0;
};

bool is_unsigned_integer(const std::string & text)
{
  return !text.empty() && std::all_of(text.begin(), text.end(), [](unsigned char ch) {
    return std::isdigit(ch) != 0;
  });
}

bool is_supported_image(const fs::path & path)
{
  if (!path.has_extension()) return false;

  auto extension = path.extension().string();
  std::transform(extension.begin(), extension.end(), extension.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return extension == ".jpg" || extension == ".jpeg" || extension == ".png" || extension == ".bmp";
}

std::vector<fs::path> collect_image_paths(const std::string & input_folder)
{
  fs::path folder(input_folder);
  if (!fs::exists(folder)) {
    throw std::runtime_error(fmt::format("输入文件夹不存在: {}", input_folder));
  }
  if (!fs::is_directory(folder)) {
    throw std::runtime_error(fmt::format("输入路径不是文件夹: {}", input_folder));
  }

  std::vector<fs::path> image_paths;
  for (const auto & entry : fs::directory_iterator(folder)) {
    if (entry.is_regular_file() && is_supported_image(entry.path())) image_paths.emplace_back(entry.path());
  }

  std::sort(image_paths.begin(), image_paths.end(), [](const fs::path & lhs, const fs::path & rhs) {
    auto lhs_stem = lhs.stem().string();
    auto rhs_stem = rhs.stem().string();
    auto lhs_is_number = is_unsigned_integer(lhs_stem);
    auto rhs_is_number = is_unsigned_integer(rhs_stem);

    if (lhs_is_number && rhs_is_number) {
      auto lhs_number = std::stoll(lhs_stem);
      auto rhs_number = std::stoll(rhs_stem);
      if (lhs_number != rhs_number) return lhs_number < rhs_number;
    }

    return lhs.filename().string() < rhs.filename().string();
  });

  return image_paths;
}

LoadResult load(
  const std::string & input_folder, const std::string & config_path, cv::Size & img_size,
  std::vector<std::vector<cv::Point3f>> & obj_points,
  std::vector<std::vector<cv::Point2f>> & img_points)
{
  // 读取yaml参数
  auto yaml = YAML::LoadFile(config_path);
  auto pattern_cols = yaml["pattern_cols"].as<int>();
  auto pattern_rows = yaml["pattern_rows"].as<int>();
  auto center_distance_mm = yaml["center_distance_mm"].as<double>();
  cv::Size pattern_size(pattern_cols, pattern_rows);
  auto image_paths = collect_image_paths(input_folder);
  LoadResult result{image_paths.size(), 0};

  if (image_paths.empty()) {
    fmt::print(stderr, "在 {} 中没有找到标定图片，支持的格式: jpg/jpeg/png/bmp\n", input_folder);
    return result;
  }

  for (const auto & img_path_fs : image_paths) {
    // 读取图片
    auto img_path = img_path_fs.string();
    auto img = cv::imread(img_path);
    if (img.empty()) {
      fmt::print(stderr, "[warning] 无法读取图片: {}\n", img_path);
      continue;
    }

    // 设置图片尺寸
    img_size = img.size();

    // 识别标定板
    std::vector<cv::Point2f> centers_2d;
    auto success = cv::findCirclesGrid(img, pattern_size, centers_2d, cv::CALIB_CB_SYMMETRIC_GRID);

    // 显示识别结果
    auto drawing = img.clone();
    cv::drawChessboardCorners(drawing, pattern_size, centers_2d, success);
    cv::resize(drawing, drawing, {}, 0.5, 0.5);  // 缩小图片尺寸便于显示完全
    cv::imshow("Press any to continue", drawing);
    cv::waitKey(0);

    // 输出识别结果
    fmt::print("[{}] {}\n", success ? "success" : "failure", img_path);
    if (!success) continue;

    // 记录所需的数据
    img_points.emplace_back(centers_2d);
    obj_points.emplace_back(centers_3d(pattern_size, center_distance_mm));
    result.detected_images++;
  }

  fmt::print(
    "共找到 {} 张图片，其中 {} 张成功识别到 {}x{} 圆点标定板\n", result.total_images,
    result.detected_images, pattern_cols, pattern_rows);
  return result;
}

void print_yaml(const cv::Mat & camera_matrix, const cv::Mat & distort_coeffs, double error)
{
  YAML::Emitter result;
  std::vector<double> camera_matrix_data(
    camera_matrix.begin<double>(), camera_matrix.end<double>());
  std::vector<double> distort_coeffs_data(
    distort_coeffs.begin<double>(), distort_coeffs.end<double>());

  result << YAML::BeginMap;
  result << YAML::Comment(fmt::format("重投影误差: {:.4f}px", error));
  result << YAML::Key << "camera_matrix";
  result << YAML::Value << YAML::Flow << camera_matrix_data;
  result << YAML::Key << "distort_coeffs";
  result << YAML::Value << YAML::Flow << distort_coeffs_data;
  result << YAML::Newline;
  result << YAML::EndMap;

  fmt::print("\n{}\n", result.c_str());
}

int main(int argc, char * argv[])
{
  try {
    // 读取命令行参数
    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help")) {
      cli.printMessage();
      return 0;
    }
    auto input_folder = cli.get<std::string>(0);
    auto config_path = cli.get<std::string>("config-path");

    // 从输入文件夹中加载标定所需的数据
    cv::Size img_size;
    std::vector<std::vector<cv::Point3f>> obj_points;
    std::vector<std::vector<cv::Point2f>> img_points;
    auto load_result = load(input_folder, config_path, img_size, obj_points, img_points);

    if (obj_points.empty()) {
      fmt::print(
        stderr,
        "没有可用于标定的图片: input_folder={}, 总图片数={}, 成功识别数={}\n"
        "请先运行 ./build/capture 采集图片，或检查配置中的 pattern_cols/pattern_rows 是否与标定板一致。\n",
        input_folder, load_result.total_images, load_result.detected_images);
      return 1;
    }

    // 相机标定
    cv::Mat camera_matrix, distort_coeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    auto criteria = cv::TermCriteria(
      cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100,
      DBL_EPSILON);  // 默认迭代次数(30)有时会导致结果发散，故设为100
    cv::calibrateCamera(
      obj_points, img_points, img_size, camera_matrix, distort_coeffs, rvecs, tvecs,
      cv::CALIB_FIX_K3, criteria);  // 由于视场角较小，不需要考虑k3

    // 重投影误差
    double error_sum = 0;
    size_t total_points = 0;
    for (size_t i = 0; i < obj_points.size(); i++) {
      std::vector<cv::Point2f> reprojected_points;
      cv::projectPoints(
        obj_points[i], rvecs[i], tvecs[i], camera_matrix, distort_coeffs, reprojected_points);

      total_points += reprojected_points.size();
      for (size_t j = 0; j < reprojected_points.size(); j++)
        error_sum += cv::norm(img_points[i][j] - reprojected_points[j]);
    }
    auto error = error_sum / total_points;

    // 输出yaml
    print_yaml(camera_matrix, distort_coeffs, error);
    return 0;
  } catch (const cv::Exception & e) {
    fmt::print(stderr, "OpenCV异常: {}\n", e.what());
    return 1;
  } catch (const std::exception & e) {
    fmt::print(stderr, "标定失败: {}\n", e.what());
    return 1;
  }
}
