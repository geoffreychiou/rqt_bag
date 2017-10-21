#include <ros/ros.h>
#include <string>
#include <fstream>

// Messages and Services
#include <saver_service/BollesTestData.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>

// Boost
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

// CV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

// YAML
#include <yaml-cpp/yaml.h>

struct MetadataData
{
  std::string file_name;

  std::string image_file_name;
  std::string image_frame_id;

  std::string cloud_file_name;
  std::string cloud_frame_id;

  std::string location;
  std::string label_orientation;
};

struct SaveData
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cv::Mat image;  
  ros::Time image_timestamp;
  ros::Time cloud_timestamp;
  std::string image_frame_id;
  std::string cloud_frame_id;
  std::string location;
  std::string label_orientation;
};

bool writeToYAML(const MetadataData &metadata, const std::string &path)
{
  YAML::Emitter out;
  out << YAML::BeginMap;
    out << YAML::Key << "File Name";
    out << YAML::Value << metadata.file_name;
    out << YAML::Key << "Image File";
    out << YAML::Value << metadata.image_file_name;
    out << YAML::Key << "Image Frame";
    out << YAML::Value << metadata.image_frame_id;
    out << YAML::Key << "Cloud File";
    out << YAML::Value << metadata.cloud_file_name;
    out << YAML::Key << "Cloud Frame";
    out << YAML::Value << metadata.cloud_frame_id;
    out << YAML::Key << "Location";
    out << YAML::Value << metadata.location;
    out << YAML::Key << "Label Orientation";
    out << YAML::Value << metadata.label_orientation;
  out << YAML::EndMap;

  if (out.good())
  {
    std::ofstream yaml_out(path.c_str());
    yaml_out << out.c_str();
    return true;  
  }
  else {return false;}
}

std::string getRandomName(void)
{
  boost::uuids::random_generator gen;
  boost::uuids::uuid u = gen();
  return boost::uuids::to_string(u);  
}

bool save(const SaveData &save_data)
{
  // Hard coded path (creates if it doesn't exist)
  std::string save_path = "/tmp/bolles_test_data/";
  boost::filesystem::path bsave_path(save_path);
  if (!boost::filesystem::exists(bsave_path))
    boost::filesystem::create_directory(bsave_path);

  // Get random uuid for filename
  std::string name = getRandomName();

  // Set file names
  std::string metadata_file_name = name + ".yaml";
  std::string image_file_name = name + ".png";
  std::string cloud_file_name = name + ".pcd";

  // Set MetaData
  MetadataData metadata;
  metadata.file_name = name;
  metadata.image_file_name = image_file_name;
  metadata.image_frame_id = save_data.image_frame_id;
  metadata.cloud_file_name = cloud_file_name;
  metadata.cloud_frame_id = save_data.cloud_frame_id;
  metadata.location = save_data.location;
  metadata.label_orientation = save_data.label_orientation;

  // Saving image
  try
  {
    // PNG Compression
    std::vector<int> params;
    params.push_back(CV_IMWRITE_PNG_COMPRESSION); params.push_back(0);
    if (!cv::imwrite(save_path + image_file_name, save_data.image, params))
    {
      ROS_ERROR_STREAM("Failed to save image: " << save_path + image_file_name);
      return false;
    }
  }
  catch (std::exception &ex) 
  {
    ROS_ERROR_STREAM("Failed to save image: " << save_path + image_file_name);
    return false;
  }

  // Saving Point Cloud
  try
  {
    pcl::PCDWriter writer;
    writer.writeBinaryCompressed(save_path + cloud_file_name, save_data.cloud);
  }
  catch (std::exception &ex)
  {
    ROS_ERROR_STREAM("Failed to save cloud: " << save_path + cloud_file_name);
    return false;
  }

  // Saving metadata
  try
  {
    if (!writeToYAML(metadata, save_path + metadata_file_name))
    {
      ROS_ERROR_STREAM("Failed to metadata: " << save_path + metadata_file_name);
      return false;
    }
  }
  catch (std::exception &ex)
  {
    ROS_ERROR_STREAM("Failed to metadata: " << save_path + metadata_file_name);
    return false;
  }

  return true;
}

bool saveSrv(saver_service::BollesTestData::Request &req,
  saver_service::BollesTestData::Response &res)
{
  SaveData save_data;

  // Get data from cloud message in request
  pcl::fromROSMsg(req.cloud, save_data.cloud);
  save_data.cloud_timestamp = req.cloud.header.stamp;
  save_data.cloud_frame_id = req.cloud.header.frame_id;

  // Get data from image message in request
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(req.image);
    save_data.image_timestamp = req.image.header.stamp;
    save_data.image_frame_id = req.image.header.frame_id;
    save_data.image = cv_ptr->image;    
  }
  catch (cv_bridge::Exception &e) 
  {
    ROS_ERROR_STREAM("Unable to retrieve image from service call!");
    return false;
  }

  // Get location and label_orientation from message in request
  save_data.location = req.location;
  save_data.label_orientation = req.label_orientation;

  return save(save_data);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "saver_service");
  ros::NodeHandle nh;

  ros::ServiceServer saver_service = nh.advertiseService("bolles_saver_service", saveSrv);
  ros::spin();  

  return 0;
}