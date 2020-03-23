#define PTCLD_DELAY_OFFSET (0.05)

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

std::string cam;
std::string im_topic = "/blaser_camera/image_color";

void parse_args(int argc, char **argv)
{
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help",
            "produce help message")
        ("cam", po::value<std::string>(),
            "camera stream name(REQUIRED)")
        ("im_topic", po::value<std::string>(),
            "output topic name (default /blaser_cam/image_color)");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        exit(1);
    }

    if (vm.count("cam"))
    {
        cam = vm["cam"].as<std::string>();
        ROS_INFO("Opening camera: [%s]", cam.c_str());
    }
    else
    {
        std::cout << "Need camera to open! Stopping.\n";
        exit(1);
    }

    if (vm.count("im_topic"))
    {
        im_topic = vm["im_topic"].as<std::string>();
        ROS_INFO("Publishing image on topic [%s]", im_topic.c_str());
    }
}

int main(int argc, char *argv[])
{
    std::cout << "----------------------\nInitializing ROS...\n";
    ros::init(argc, argv, "blaser_image_pub");
    ros::NodeHandle nh;

    parse_args(argc, argv);
    cv::VideoCapture cap;
    cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('Y', 'U', 'Y', '2'));
    cap.set(CV_CAP_PROP_BUFFERSIZE, 1);
    cap.open(cam);

    cv::Mat frame;
    std::cout << "Waiting for first image..." << std::endl;
    // Wait until initialized to start all of ros things
    while (true)
    {
        if (cap.read(frame))
        {
            if (!frame.empty())
                break;
        }

        if (!ros::ok())
            exit(1);
    }
    std::cout << "Read one frame!\n";

    image_transport::ImageTransport itt(nh);
    auto impub = itt.advertise(im_topic.c_str(), 1);
    std::cout << "Initialized ROS.\n";

    std::cout << "Publishing images now.\n";
    ros::Time t0 = ros::Time::now();
    int ix = 0;
    ros::Rate r(90); //Max framerate at 90 fps
    while (ros::ok())
    {
        cap.read(frame);
        ros::Time t = ros::Time::now();

        if (frame.empty())
        {
            std::cerr << "ERROR! frame empty!\n";
            break;
        }

        sensor_msgs::Image ros_img;
        auto durr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame);
        durr.toImageMsg(ros_img);
        ros_img.header.stamp = t - ros::Duration(PTCLD_DELAY_OFFSET);
        ros_img.header.seq = ix++;

        impub.publish(ros_img);

        if ((t - t0).toSec() > 2) // calc framerate in 2s sliding window
        {
            std::cout << "framerate: " << ix / (t - t0).toSec() << std::endl;
            t0 = t;
            ix = 0;
        }

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
