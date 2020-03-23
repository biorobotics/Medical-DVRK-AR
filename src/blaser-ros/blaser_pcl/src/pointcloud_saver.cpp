#include <stdio.h>
#include <boost/program_options.hpp>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <std_msgs/Float64.h>

#include <pcl/common/centroid.h>

namespace po = boost::program_options;

std::string outfile;
std::string pcl_topic = "/blaser_pcl_topic";
std::string fixed_frame = "world";

void parse_args(int argc, char** argv)
{
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce this message")
    ("outfile", po::value<std::string>(), "save data to this file")
    ("pcl_topic", po::value<std::string>(), "Input pointcloud topic (default blaser_pcl_topic)")
    ("fixed_frame", po::value<std::string>(), "Fixed frame to record in");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << desc << std::endl;
    exit(1);
  }

  if (vm.count("outfile")) {
    outfile = vm["outfile"].as<std::string>();
  } else {
    std::cout << "YOU NEED AN OUTPUT FILE!\nUsage:\n" << desc;
    exit(1);
  }
  ROS_INFO("Saving into %s!", outfile.c_str());

  if (vm.count("pcl_topic")) {
    pcl_topic = vm["pcl_topic"].as<std::string>();
    ROS_INFO("Listeneing to topic [%s].", pcl_topic.c_str());
  }

  if (vm.count("fixed_frame")) {
    fixed_frame = vm["fixed_frame"].as<std::string>();
  }
}

class HeightRecNode
{
  ros::NodeHandle nh;
  ros::Subscriber pcl_sub;
  ros::Publisher mz_pub, rz;

public:
  tf::TransformListener listener;
  std::ofstream f;

  HeightRecNode()
  {
    pcl_sub = nh.subscribe(pcl_topic.c_str(), 20, &HeightRecNode::cb, this);

    f.open(outfile.c_str(), std::ios::out | std::ios::trunc);
  }

  void cb(const sensor_msgs::PointCloud2& pc)
  {
    tf::StampedTransform t;
    try {
      listener.waitForTransform(fixed_frame.c_str(), pc.header.frame_id, pc.header.stamp, ros::Duration(0.5));
      listener.lookupTransform(fixed_frame.c_str(), pc.header.frame_id, pc.header.stamp, t);
    } catch (const std::exception& e) {
      std::cerr << e.what() << std::endl;
      return;
    }

    sensor_msgs::PointCloud2 transformed;
    pcl_ros::transformPointCloud(fixed_frame.c_str(), t, pc, transformed);

    pcl::PointCloud<pcl::PointXYZ> pts;
    pcl::fromROSMsg(transformed, pts);

    double z_cam = t.getOrigin().getZ();
    char buf[100];
    sprintf(buf, "%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%d\t", pc.header.stamp.toSec(),
        t.getOrigin().getX(),t.getOrigin().getY(),t.getOrigin().getZ(),
        t.getRotation().getX(),t.getRotation().getY(),t.getRotation().getZ(),t.getRotation().getW(),
        pts.size());
    f << buf;

    char fbuf[20];
    for (int i = 0; i < pts.size(); ++i) {
      auto p = pts.points[i];
      sprintf(fbuf, "%.5f\t%.5f\t%.5f\t", p.x, p.y, p.z);
      f << fbuf;
    }
    f << "\n";
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_height_node");
  parse_args(argc, argv);
  HeightRecNode hrn;

  ros::spin();
  hrn.f.close();
}

