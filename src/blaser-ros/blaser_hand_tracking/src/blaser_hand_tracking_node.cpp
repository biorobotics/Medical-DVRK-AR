#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <pcl/PolygonMesh.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <queue>
#include <functional>

using namespace pcl;

typedef pcl::PointCloud<pcl::PointXYZ> PCLPc;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PCXYZPtr;

class GrabCentroid
{
    public:
    GrabCentroid();
    void pointcloud_cb(const sensor_msgs::PointCloud2& msg);

    private:
    ros::NodeHandle nh;
    ros::Subscriber blaser_pcl_sub;

    ros::Publisher cluster_pub;
    ros::Publisher centroid_pub;
    tf::TransformListener listener;
};

GrabCentroid::GrabCentroid()
{
    blaser_pcl_sub = nh.subscribe("/blaser_pcl_filtered", 1, &GrabCentroid::pointcloud_cb, this);
    cluster_pub = nh.advertise<PCLPc> ("blaser_hand_cluster", 1);
    centroid_pub = nh.advertise<PCLPc> ("blaser_hand_centroid", 1);
}

bool comparePointXyz(PointXYZ a, PointXYZ b)
{
    return a.z > b.z;
}

template<typename T> void print_queue(T& q) {
    while(!q.empty()) {
        std::cout << q.top() << " ";
        q.pop();
    }
    std::cout << '\n';
}

void GrabCentroid::pointcloud_cb(const sensor_msgs::PointCloud2& input_pc)
{
    // tf::StampedTransform transform;
    // try
    // {
    //     listener.lookupTransform("world", input_pc.header.frame_id, input_pc.header.stamp, transform);
    // }
    // catch (const std::exception &e)
    // {
    //     std::cerr << e.what() << '\n';
    //     return;
    // }

    // sensor_msgs::PointCloud2 transformed_pc;
    //sensor_msgs::PointCloud2 in_pc = input_pc;
    // pcl_ros::transformPointCloud("/world", input_pc, transformed_pc, listener);
    // pcl_ros::transformPointCloud("world", transform, input_pc, transformed_pc);

    // pcl::PointCloud<pcl::PointXYZ> pc;
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(input_pc, pcl_pc2);
    PCXYZPtr pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *pc);
    std::priority_queue<PointXYZ, std::vector<PointXYZ>, std::function<bool(PointXYZ, PointXYZ)>> sorted_pc(comparePointXyz);
    for (auto it = pc->begin(); it != pc->end(); ++it)
    {
        // double x=it->x, y=it->y, z=it->z;
        // double dist = sqrt(x*x + y*y + z*z); // only want z distance? 
        sorted_pc.push(*it);
    }

    long pc_size = sorted_pc.size();

    pc->clear();

    for (int i = 0; i < pc_size * 0.05; i++) {
        pc->push_back(sorted_pc.top());
        sorted_pc.pop();
    }

    // publish the clustered points
    cluster_pub.publish(pc);

    // compute centroid
    Eigen::Vector4f centroid;
    compute3DCentroid(*pc, centroid);

    PCLPc centroid_pc;
    centroid_pc.push_back(PointXYZ(centroid[0], centroid[1], centroid[2]));
    centroid_pc.header = pc->header;

    centroid_pub.publish(centroid_pc);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hand_track_node");
    ros::start();

    GrabCentroid gc;

    ros::spin();
    return 0;
}