#include <stdio.h>
#include <boost/program_options.hpp>
#include <opencv2/opencv.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <blaser_pcl/LaserExtractionConfig.h>
#include "pclGen.hpp"

namespace po = boost::program_options;

std::string calibfile;
std::string im_topic = "/blaser_camera/image_color";
std::string pcl_topic = "/blaser_pcl_topic";
std::string pcl_frame = "blaser_camera";
float delay_offset = 0.19;

int thresh = 150;
double peak_perc = 0.8;

static cv::Mat handeye_transform;

#ifdef DYN_RECONFIG_UPDATE
cv_bridge::CvImageConstPtr current_frame_ptr;
#endif

#define RED_THRESH 100
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

static int parse_handeye(std::string path)
{
    cout << "[debug]: reading hand-eye calibration parameters\n";
    cout << "[debug]: configuration path: " << path << "\n";
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        // file not opened
        cout << "[warn]: file " << path << " not found, config not loaded\n";
        return -1;
    }
    fs["handeye_transform"] >> handeye_transform;

    cout << handeye_transform.rows << ", " << handeye_transform.cols << endl;
    if (handeye_transform.rows == 1 && handeye_transform.cols == 6)
    {
        cout << "--- Calibration Data ---\n";
        cout << "Handeye Transformation:\n";
        cout << handeye_transform << "\n";
        cout << "------------------------\n";
        return 0;
    }

    ROS_WARN("No handeye transform found, static tf not published");
    return -1;
}

void parse_args(int argc, char **argv)
{
    po::options_description desc("Allowed options");
    desc.add_options()("help", "produce help message")("calib", po::value<std::string>(), "calibration yaml file path. (REQUIRED)")("im_topic", po::value<std::string>(), "input topic name (default /blaser_cam/image_color)")("pcl_topic", po::value<std::string>(), "pointcloud topic name (default /blaser_pcl_topic)")("pcl_frame", po::value<std::string>(), "pointcloud frame (default blaser_cam)")("offset", po::value<float>(), "delay offset of blaser to ros")

        ("thresh", po::value<int>(), "Red intensity threshold")("peak_perc", po::value<float>(), "Percent of peak to consider for subpixel");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        exit(1);
    }

    if (vm.count("calib"))
    {
        calibfile = vm["calib"].as<std::string>();
        ROS_INFO("Loading calibration from [%s]...", calibfile.c_str());
    }
    else
    {
        std::cout << "Need calibration file. Stopping.\n";
        exit(1);
    }

    if (vm.count("im_topic"))
    {
        im_topic = vm["im_topic"].as<std::string>();
        ROS_INFO("Listening for images on topic [%s]...", im_topic.c_str());
    }
    else
    {
        ROS_INFO("No topic specified. Listening for images on default topic [%s]", im_topic.c_str());
    }

    if (vm.count("pcl_frame"))
    {
        pcl_frame = vm["pcl_frame"].as<std::string>();
        ROS_INFO("Broadcasting pointclouds with frame [%s]", pcl_frame.c_str());
    }

    if (vm.count("pcl_topic"))
    {
        pcl_topic = vm["pcl_topic"].as<std::string>();
        ROS_INFO("Broadcasting pointclouds on topic [%s]", pcl_topic.c_str());
    }

    if (vm.count("offset"))
    {
        delay_offset = vm["offset"].as<float>();
        ROS_INFO("Assuming constant delay from blaser to ros of %fs!", delay_offset);
    }
}

class PointcloudPub
{
    // ros::NodeHandle nh;
    cv::Mat IntrinsicMat, DistortionMat, LaserPlane;

public:
    image_transport::Subscriber im_sub;
    ros::Publisher pcl_pub;
    BEPcl pcl_gen;
    image_transport::Publisher im_pub;
#ifdef DEBUG_LASER_EXT
    image_transport::Publisher collage_im_pub;
#endif
#ifdef DYN_RECONFIG_LASER_PLANE_VIS
    ros::Publisher marker_pub;
    visualization_msgs::Marker lplane_boundary;
#endif

    PointcloudPub(ros::NodeHandle nh) : pcl_gen(calibfile) /*: ptcld(new PointCloud)*/
    {
        ROS_INFO("Opening camera calibration parameters");

        image_transport::ImageTransport itt(nh);
        im_sub = itt.subscribe(im_topic.c_str(), 1, boost::bind(&PointcloudPub::im_cb, this, _1));
        pcl_pub = nh.advertise<PointCloud>(pcl_topic.c_str(), 1);

        ROS_INFO("Opening calibration parameters successful!");

        im_pub = itt.advertise("pcl_debug_im", 1);

#ifdef DEBUG_LASER_EXT
        collage_im_pub = itt.advertise("pcl_collage_im", 1);
#endif

#ifdef DYN_RECONFIG_LASER_PLANE_VIS
        initVisualizationMarkers();
        marker_pub = nh.advertise<visualization_msgs::Marker>(
            "blaser_visual_marker", 1);
#endif

        pcl_gen.intensity_thresh = thresh;
        pcl_gen.peak_perc = peak_perc;
    }

    void im_cb(const sensor_msgs::ImageConstPtr &im)
    {
#ifdef DYN_RECONFIG_UPDATE
        // Enabling this has performance penalty
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(im, "bgr8");
        // Note: this pointer address does not seem to change, magic in
        //       boost library probably. No assumptions without understanding.
        //       By experiment, toCvShare is sufficient without toCvCopy.
        //       Perf impact is negligible in this case.
        current_frame_ptr = cv_ptr;
#else
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(im, "bgr8");
#endif

        pcl_gen.getPclFromImg(cv_ptr->image);
        pcl_conversions::toPCL(im->header.stamp - ros::Duration(delay_offset), pcl_gen.ptcld->header.stamp);
        pcl_gen.ptcld->header.frame_id = pcl_frame;
        pcl_gen.ptcld->height = 1;
        pcl_gen.ptcld->width = pcl_gen.ptcld->points.size();
        pcl_pub.publish(pcl_gen.ptcld);

        debug_pubs();
    }

    void debug_pubs()
    {
        std_msgs::Header h;
        h.stamp = ros::Time::now();
        cv_bridge::CvImage cvim(h, "bgr8", pcl_gen.undistortedFrame);

        for (auto pt : pcl_gen.undistortedPoints)
        {
            double fx = pcl_gen.IntrinsicMat.at<double>(0, 0);
            double fy = pcl_gen.IntrinsicMat.at<double>(1, 1);
            double cx = pcl_gen.IntrinsicMat.at<double>(0, 2);
            double cy = pcl_gen.IntrinsicMat.at<double>(1, 2);

            double px = fx * pt.x + cx;
            double py = fy * pt.y + cy;
            cv::circle(cvim.image, cv::Point2d(px, py), 1, cv::Scalar(0, 255, 0), -1);
        }
        im_pub.publish(cvim.toImageMsg());

#ifdef DEBUG_LASER_EXT
        cv_bridge::CvImage collage_image(h, "bgr8", pcl_gen.debugCollageFrame);
        collage_im_pub.publish(collage_image.toImageMsg());
#endif

#ifdef DYN_RECONFIG_LASER_PLANE_VIS
    lplane_boundary.header.stamp = ros::Time::now();
    marker_pub.publish(lplane_boundary);
#endif
    }

    void dycfg_callback(blaser_pcl::LaserExtractionConfig &config, uint32_t level)
    {
        ROS_INFO("Reconfigure request received");

        // Bugfix: dynamic reconfigure default values not loaded
        // https://answers.ros.org/question/213305/dynamic_reconfigure-group-defaults/

        pcl_gen.simpleCBPercent = config.simplecb_percent;
        pcl_gen.simpleCBEnabled = config.simplecb_enabled;
        
        pcl_gen.redMask1Lo = {
            double(config.RMask_1_Lo_H),
            double(config.RMask_1_Lo_S),
            double(config.RMask_1_Lo_V)
        };
        pcl_gen.redMask1Hi = {
            double(config.RMask_1_Hi_H),
            double(config.RMask_1_Hi_S),
            double(config.RMask_1_Hi_V)
        };
        pcl_gen.redMask2Lo = {
            double(config.RMask_2_Lo_H),
            double(config.RMask_2_Lo_S),
            double(config.RMask_2_Lo_V)
        };
        pcl_gen.redMask2Hi = {
            double(config.RMask_2_Hi_H),
            double(config.RMask_2_Hi_S),
            double(config.RMask_2_Hi_V)
        };

        pcl_gen.outlierRemovalEnabled =
            config.groups.noise_filters.outlier_removel;
        pcl_gen.roiRowStart =
            config.groups.noise_filters.roi_r_percent;
        pcl_gen.roiColSideCut =
            config.groups.noise_filters.roi_c_percent;

        std::cout << config.simplecb_percent << "..." << pcl_gen.redMask2Hi << std::endl;

#ifdef DYN_RECONFIG_LASER_PLANE_VIS
        initVisualizationMarkers();
#endif

#ifdef DYN_RECONFIG_UPDATE
        // Re-activate pipeline for current frame, helpful for bag stepping.
        if (current_frame_ptr)
        {
            // Note: this is for visual in param-tuning only, PCL not updated.
            pcl_gen.getPclFromImg(current_frame_ptr->image);
            debug_pubs();
        }
#endif
    }

private:
#ifdef DYN_RECONFIG_LASER_PLANE_VIS
    void initVisualizationMarkers(void)
    {
        // Visualization for laser plane under current calibration profile
        double cOffset = pcl_gen.imageWidth * pcl_gen.roiColSideCut;
        double rOffset = pcl_gen.imageHeight * pcl_gen.roiRowStart;
        std::vector<cv::Point2d> testPoints;
        std::vector<pcl::PointXYZ> testPoints3D;
        testPoints.push_back(
            cv::Point2d(0 + cOffset, rOffset));
        testPoints.push_back(
            cv::Point2d(pcl_gen.imageWidth - cOffset, rOffset));
        testPoints.push_back(
            cv::Point2d(pcl_gen.imageWidth - cOffset, pcl_gen.imageHeight));
        testPoints.push_back(
            cv::Point2d(0 + cOffset, pcl_gen.imageHeight));

        cv::undistortPoints(testPoints, testPoints,
            pcl_gen.IntrinsicMat, pcl_gen.DistortionMat);

        for (cv::Point2d pt : testPoints) {
            testPoints3D.push_back(pcl_gen.reconstruct3DPoint(pt));
        }
        testPoints3D.push_back(testPoints3D.at(0));

        lplane_boundary.type = visualization_msgs::Marker::LINE_STRIP;
        lplane_boundary.header.frame_id = pcl_frame;
        lplane_boundary.ns = "blaser_lplane";
        lplane_boundary.action = visualization_msgs::Marker::ADD;
        lplane_boundary.pose.orientation.w = 1;
        lplane_boundary.id = 0;
        lplane_boundary.scale.x = 0.001;
        lplane_boundary.color.r = 1.0;
        lplane_boundary.color.a = 1.0;
        lplane_boundary.frame_locked = false;

        lplane_boundary.points.clear();
        for (auto brdpt : testPoints3D) {
            geometry_msgs::Point p;
            p.x = brdpt.x;
            p.y = brdpt.y;
            p.z = brdpt.z;
            lplane_boundary.points.push_back(p);
        }
    }
#endif
};

int main(int argc, char **argv)
{
    parse_args(argc, argv);
    ros::init(argc, argv, "blaser_pointcloud_pub");
    ros::NodeHandle nh;

    // Pass pointer to bound callback, o/w value assignment does not work
    PointcloudPub *pcp = new PointcloudPub(nh);

    dynamic_reconfigure::Server<blaser_pcl::LaserExtractionConfig> server;
    dynamic_reconfigure::Server<blaser_pcl::LaserExtractionConfig>::CallbackType f;

    f = boost::bind(&PointcloudPub::dycfg_callback, pcp, _1, _2);
    server.setCallback(f);

    ROS_INFO("Dynamic reconfigure server is up");

    // Publish static transform for blaser camera
    if (!parse_handeye(calibfile))
    {
        static tf2_ros::StaticTransformBroadcaster static_broadcaster;
        geometry_msgs::TransformStamped handeye_tf;

        handeye_tf.header.stamp = ros::Time::now();
        handeye_tf.header.frame_id = "ee_link";
        handeye_tf.child_frame_id = pcl_frame;
        handeye_tf.transform.translation.x = handeye_transform.at<double>(0);
        handeye_tf.transform.translation.y = handeye_transform.at<double>(1);
        handeye_tf.transform.translation.z = handeye_transform.at<double>(2);
        tf2::Quaternion quat;
        quat.setRPY(handeye_transform.at<double>(3),
                    handeye_transform.at<double>(4),
                    handeye_transform.at<double>(5));
        handeye_tf.transform.rotation.x = quat.x();
        handeye_tf.transform.rotation.y = quat.y();
        handeye_tf.transform.rotation.z = quat.z();
        handeye_tf.transform.rotation.w = quat.w();

        static_broadcaster.sendTransform(handeye_tf);
        ROS_INFO("Published handeye static transform");
    }

    ros::spin();
}
