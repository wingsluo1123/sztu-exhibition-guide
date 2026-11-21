// Standard PCL operations: concatenate PCD chunks, VoxelGrid, constrained RANSAC.
#include <filesystem>
#include <fstream>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

int main(int argc, char** argv) {
  if (argc != 3 && argc != 4) {std::cerr << "usage: pcd_merge chunk_directory raw_output.pcd [leveled_output.pcd]\n";return 2;}
  using Point = pcl::PointXYZI;
  auto merged = pcl::make_shared<pcl::PointCloud<Point>>();
  std::size_t chunks = 0, input_points = 0;
  for (const auto& entry : std::filesystem::directory_iterator(argv[1])) {
    if (entry.path().extension() != ".pcd") continue;
    pcl::PointCloud<Point> cloud;
    if (pcl::io::loadPCDFile(entry.path().string(), cloud) < 0) return 3;
    *merged += cloud;
    input_points += cloud.size();++chunks;
    pcl::VoxelGrid<Point> filter;
    filter.setInputCloud(merged);filter.setLeafSize(0.05f,0.05f,0.05f);
    auto reduced = pcl::make_shared<pcl::PointCloud<Point>>();
    filter.filter(*reduced);merged = reduced;
  }
  if (merged->empty()) return 4;
  if (pcl::io::savePCDFileBinary(argv[2], *merged) < 0) return 5;
  auto floor = pcl::make_shared<pcl::PointCloud<Point>>();
  pcl::PassThrough<Point> pass;
  pass.setInputCloud(merged);pass.setFilterFieldName("z");pass.setFilterLimits(-1.3,-0.6);
  pass.filter(*floor);
  pcl::SACSegmentation<Point> segmentation;
  segmentation.setOptimizeCoefficients(true);
  segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setAxis(Eigen::Vector3f::UnitZ());segmentation.setEpsAngle(0.25);
  segmentation.setDistanceThreshold(0.04);segmentation.setMaxIterations(1000);
  segmentation.setInputCloud(floor);
  pcl::PointIndices inliers;pcl::ModelCoefficients coefficients;
  segmentation.segment(inliers,coefficients);
  Point minimum,maximum;pcl::getMinMax3D(*merged,minimum,maximum);
  std::ofstream report(std::string(argv[2])+".json");
  report << "{\"chunks\":" << chunks << ",\"input_points\":" << input_points
         << ",\"voxel_points\":" << merged->size() << ",\"floor_inliers\":" << inliers.indices.size()
         << ",\"min\":[" << minimum.x << ',' << minimum.y << ',' << minimum.z
         << "],\"max\":[" << maximum.x << ',' << maximum.y << ',' << maximum.z << "],\"floor_plane\":[";
  for (std::size_t i=0;i<coefficients.values.size();++i) report << (i ? "," : "") << coefficients.values[i];
  report << "]}\n";
  if (argc == 4) {
    if (coefficients.values.size()!=4 || inliers.indices.size()<1000) return 6;
    Eigen::Vector3f normal(coefficients.values[0],coefficients.values[1],coefficients.values[2]);
    float offset=coefficients.values[3];
    if (normal.z()<0) {normal=-normal;offset=-offset;}
    Eigen::Matrix4f transform=Eigen::Matrix4f::Identity();
    transform.block<3,3>(0,0)=Eigen::Quaternionf::FromTwoVectors(normal,Eigen::Vector3f::UnitZ()).toRotationMatrix();
    transform(2,3)=offset/normal.norm();
    pcl::PointCloud<Point> leveled;
    pcl::transformPointCloud(*merged,leveled,transform);
    if (pcl::io::savePCDFileBinary(argv[3],leveled)<0) return 7;
    std::ofstream tf(std::string(argv[3])+".transform.json");
    tf << "{\"source_frame\":\"mapping_odom\",\"target_frame\":\"map\",\"matrix\":[";
    for (int row=0;row<4;++row) {
      tf << (row ? "," : "") << '[';
      for (int col=0;col<4;++col) tf << (col ? "," : "") << transform(row,col);
      tf << ']';
    }
    tf << "]}\n";
  }
  std::cout << "merged " << chunks << " chunks into " << merged->size() << " points\n";
}
