//
//  pclGen.hpp
//  blaser-embedded
//
//  Created by Haowen Shi on 7/24/17.
//  Copyright © 2017 Haowen Shi. All rights reserved.
//

#ifndef pclGen_hpp
#define pclGen_hpp

#include <stdio.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string.h>
#include <pcl_ros/point_cloud.h>

#include "configuration.hpp"

using namespace std;

/* BEPcl
 * Point cloud calculation from camera raw frame.
 * Needs calibration information from configPath.
 * 
 * Usage: instantiate BEPcl with configPath to correctly
 * formatted yaml file including the following matrices:
 *   NAME                       DIM    ELEMENTS/MODEL
 *   ---------------------------------------------------
 *   1. camera_matrix           [3x3]  PINHOLE
 *   2. distortion_coefficients [1x5]  [k1 k2 p1 p2 k3]
 *   3. laser_plane             [1x3]  [A B C D]
 *   4. projection_matrix       [3x4]  PINHOLE
 * 
 * Example usage:
 *
 *   BEVideoStream *vs = ... // see BEVideoStream API
 *   (*vs).start();
 *   BEPcl pclPipeline = BEPcl("~/blaser-core/calib.yaml");
 *   pclPipeline.start(vs);
 *   vector<cv::Point3d> ptcld = pclPipeline.getPcl();
 */

class BEPcl
{
public:
    cv::Mat IntrinsicMat;
    cv::Mat DistortionMat;
    cv::Mat LaserPlane; // [A, B, C, D]
    int imageWidth = 0;
    int imageHeight = 0;

    std::vector<cv::Point2d> max2dPoints;
    std::vector<cv::Point2d> undistortedPoints;

    cv::Mat laserFrame;
    cv::Mat undistortedFrame;

    /* Naive thresh + sub pixeling tunables */
    int intensity_thresh = 160;
    double peak_perc = 0.7;

    /* COM based laser extraction tunables */
    double simpleCBPercent;
    bool simpleCBEnabled;
    cv::Scalar redMask1Lo;
    cv::Scalar redMask1Hi;
    cv::Scalar redMask2Lo;
    cv::Scalar redMask2Hi;
    cv::Mat redMaskDilateKernel;
    cv::Mat redMaskCloseKernel;

    cv::Scalar satMaskLo;
    cv::Scalar satMaskHi;
    cv::Mat satMaskDilateKernel;

    /* Noise filter tunables */
    bool outlierRemovalEnabled;
    double roiRowStart;
    double roiColSideCut;

#ifdef DEBUG_LASER_EXT
    cv::Mat debugCollageFrame;
#endif

private:
    /* Implementation for extracting points on laser line.
     * ensures output points to be UNDISTORTED.
     */
    void getLaser2DPts(const cv::Mat& frame);

    /* COM based Implementation for extracting points on laser line using
     * Daqian's algorithm (Sep 2019).
     * ensures output points to be UNDISTORTED.
     */
    void getLaser2DPts_COM(const cv::Mat& frame);

    /* Implementation for converting found 2D points to
     * 3D points with depth calculated from calibration
     * parameters.
     */
    void getWorld3DPts();

    bool parseConfig(string path);

    cv::Point2d warpSearchDirection(cv::Point2d pt);

    void getLaserChannel(const cv::Mat& colorMat, cv::Mat& plane);

public:
    /* Constructors */
    BEPcl(string configPath);

    pcl::PointXYZ reconstruct3DPoint(cv::Point2d pt);

    void getPclFromImg(const cv::Mat& frame);

    pcl::PointCloud<pcl::PointXYZ>::Ptr ptcld;
};

#endif /* pclGen_hpp */
