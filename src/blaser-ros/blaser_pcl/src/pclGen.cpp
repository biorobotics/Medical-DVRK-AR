//
//  pclGen.cpp
//  blaser-embedded
//
//  Created by Haowen Shi on 7/24/17.
//  Copyright © 2017 Haowen Shi. All rights reserved.
//

#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#include "pclGen.hpp"
#include "im_proc.hpp"
#include "visualization.hpp"

// BGR
#define CV_B 0
#define CV_G 1
#define CV_R 2

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

/* Helper functions */

// TODO
cv::Point2d BEPcl::warpSearchDirection(cv::Point2d pt)
{
    return pt;
}

pcl::PointXYZ BEPcl::reconstruct3DPoint(cv::Point2d pt)
{
    // Reconstructs 3D point from 2D laser point, assuming
    // pt is after undistortion!!!

    // TODO: vectorize this operation to make it faster on array of points!

    double A = LaserPlane.at<double>(0);
    double B = LaserPlane.at<double>(1);
    double C = LaserPlane.at<double>(2);
    double D = LaserPlane.at<double>(3);
    double fx = IntrinsicMat.at<double>(0, 0);
    double fy = IntrinsicMat.at<double>(1, 1);
    // account for optical center offset
    double px = pt.x, py = pt.y;
    // double px = (pt.x - IntrinsicMatOptim.at<double>(0, 2))/fx;
    // double py = (pt.y - IntrinsicMatOptim.at<double>(1, 2))/fx;

    // Documentation see blaser-notes.pdf
    double Z = -D / ((C + B * py + A * px)); // Depth
    // double Z = (-1 * D * fx * fy) /
    //     (C * fx * fy + B * py * fx + A * px * fy); // Depth
    Z = Z / 1000; // mm to m
    double X = px * Z;
    double Y = py * Z;

    return pcl::PointXYZ(X, Y, Z);
}

void BEPcl::getLaserChannel(const cv::Mat& colorMat, cv::Mat& plane)
{
    cv::Mat color_vec[3];

    /// RGB mode
    cv::split(colorMat, color_vec);

    /// R channel only mode
    int val;
    for (int rr = 0; rr < plane.rows; rr++)
    {
        for (int cc = 0; cc < plane.cols; cc++)
        {
            val = color_vec[CV_R].at<uchar>(rr, cc);
            //plane.at<uchar>(rr, cc) = max(min(val, 255), 0);
            plane.at<uchar>(rr, cc) = val;
        }
    }
}

/* BEPcl class methods */

bool BEPcl::parseConfig(string path)
{
    cout << "[debug]: reading camera calibration parameters\n";
    cout << "[debug]: configuration path: " << path << "\n";
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        // file not opened
        cout << "[warn]: file " << path << " not found, config not loaded\n";
        return false;
    }
    fs["camera_matrix"] >> IntrinsicMat;
    fs["distortion_coefficients"] >> DistortionMat;
    fs["laser_plane"] >> LaserPlane;
    fs["image_width"] >> imageWidth;
    fs["image_height"] >> imageHeight;

    cout << "--- Calibration Data ---\n";
    cout << "Camera Matrix:\n";
    cout << IntrinsicMat << "\n";
    cout << "Distortion Coefficients:\n";
    cout << DistortionMat << "\n";
    cout << "Laser Plane:\n";
    cout << LaserPlane << "\n";
    cout << "Camera Resolution:\n";
    cout << imageWidth << "x" << imageHeight << "\n";
    cout << "------------------------\n";
    return true;
}

void BEPcl::getLaser2DPts_COM(const cv::Mat& frame)
{
    // frame is CV_8UC3, BGR order

    /* Get bgr channel values into eigen matrix */
    // cv::Mat bgr[3];
    // cv::split(frame, bgr);
    
    // Eigen::MatrixXd ch_b, ch_g, ch_r;

    // cv::cv2eigen(bgr[0], ch_b);
    // cv::cv2eigen(bgr[1], ch_g);
    // cv::cv2eigen(bgr[2], ch_r);

    max2dPoints.clear();
    undistortedPoints.clear();

#ifdef DEBUG_LASER_EXT
    frame.copyTo(
        debugCollageFrame(
            cv::Rect(
                COLLAGE_VIS_WS_OFFSET_C(0, imageWidth),
                COLLAGE_VIS_WS_OFFSET_R(0, imageHeight),
                imageWidth,
                imageHeight
            )
        )
    );
    cv::putText(debugCollageFrame,
                "Original + Laser Line",
                COLLAGE_VIS_TXTPOS_OFFSET_PT(0, 0, imageHeight, imageWidth),
                cv::FONT_HERSHEY_SIMPLEX,
                1,
                COLLAGE_TXT_COLOR
                );
    // Draw ROI
    vector<cv::Point> contour;
    contour.push_back(cv::Point(
        roiColSideCut * imageWidth, imageHeight));
    contour.push_back(cv::Point(
        roiColSideCut * imageWidth, roiRowStart * imageHeight));
    contour.push_back(cv::Point(
        (1 - roiColSideCut) * imageWidth, roiRowStart * imageHeight));
    contour.push_back(cv::Point(
        (1 - roiColSideCut) * imageWidth, imageHeight));

    const cv::Point *pts = (const cv::Point*) cv::Mat(contour).data;
    int npts = cv::Mat(contour).rows;
    cv::polylines(debugCollageFrame,
        &pts, &npts, 1, true, cv::Scalar(0, 0, 255), 2);
#endif

    cv::Mat intermediate;
    cv::Mat blurred[3];

    // 1. Median blur to filter pepper-salt noise
    cv::medianBlur(frame, intermediate, 3);

    // 2. Gaussian blur
    cv::GaussianBlur(intermediate, intermediate, cv::Size(7, 7), 0);
    cv::split(intermediate, blurred);

    /* Perform color correction */
    if (simpleCBEnabled)
    {
        SimplestCB(intermediate, intermediate, simpleCBPercent);
    }

#ifdef DEBUG_LASER_EXT
    intermediate.copyTo(
        debugCollageFrame(
            cv::Rect(
                COLLAGE_VIS_WS_OFFSET_C(1, imageWidth),
                COLLAGE_VIS_WS_OFFSET_R(0, imageHeight),
                imageWidth,
                imageHeight
            )
        )
    );
    cv::putText(debugCollageFrame,
        "Color Corrected",
        COLLAGE_VIS_TXTPOS_OFFSET_PT(0, 1, imageHeight, imageWidth),
        cv::FONT_HERSHEY_SIMPLEX,
        1,
        COLLAGE_TXT_COLOR
    );
#endif

    /* 3. Get masks */
    cv::Mat hsv_img;
    cv::Mat red_mask_1;
    cv::Mat red_mask_2;
    cv::Mat sat_mask;

    cv::cvtColor(intermediate, hsv_img, cv::COLOR_BGR2HSV);

    // Get red mask
    cv::inRange(hsv_img, redMask1Lo, redMask1Hi, red_mask_1);
    cv::inRange(hsv_img, redMask2Lo, redMask2Hi, red_mask_2);

    red_mask_1 |= red_mask_2;

    cv::morphologyEx(red_mask_1, red_mask_2,
        cv::MORPH_DILATE, redMaskDilateKernel);
    cv::morphologyEx(red_mask_2, red_mask_2,
        cv::MORPH_CLOSE, redMaskCloseKernel);

    // Get saturation mask
    cv::inRange(hsv_img, satMaskLo, satMaskHi, sat_mask);
    cv::morphologyEx(sat_mask, sat_mask,
        cv::MORPH_DILATE, satMaskDilateKernel);

#ifdef DEBUG_LASER_EXT
    cv::Mat mask_frame;
    cv::merge(
        vector<cv::Mat>{
            sat_mask,
            cv::Mat::zeros(sat_mask.size(), CV_8UC1),
            red_mask_2
        }, mask_frame);
    mask_frame.copyTo(
        debugCollageFrame(
            cv::Rect(
                COLLAGE_VIS_WS_OFFSET_C(0, imageWidth),
                COLLAGE_VIS_WS_OFFSET_R(1, imageHeight),
                imageWidth,
                imageHeight
            )
        )
    );
    cv::putText(debugCollageFrame,
        "Region Segmentation",
        COLLAGE_VIS_TXTPOS_OFFSET_PT(1, 0, imageHeight, imageWidth),
        cv::FONT_HERSHEY_SIMPLEX,
        1,
        COLLAGE_TXT_COLOR
    );
#endif

    /* 4. Subtract mean and apply mask */
    cv::Mat im_sub_mean;
    cv::Mat total_mask;

    cv::bitwise_or(red_mask_2, sat_mask, total_mask);
    blurred[2].copyTo(im_sub_mean, total_mask);

    cv::subtract(im_sub_mean, cv::mean(im_sub_mean), im_sub_mean);

#ifdef DEBUG_LASER_EXT
    cv::Mat visc_frame;
    cv::merge(
        vector<cv::Mat>{
            total_mask - 100,
            cv::Mat::zeros(sat_mask.size(), CV_8UC1),
            blurred[2]
        }, visc_frame);
    visc_frame.copyTo(
        debugCollageFrame(
            cv::Rect(
                COLLAGE_VIS_WS_OFFSET_C(1, imageWidth),
                COLLAGE_VIS_WS_OFFSET_R(1, imageHeight),
                imageWidth,
                imageHeight
            )
        )
    );
    cv::putText(debugCollageFrame,
        "Mean Subtracted",
        COLLAGE_VIS_TXTPOS_OFFSET_PT(1, 1, imageHeight, imageWidth),
        cv::FONT_HERSHEY_SIMPLEX,
        1,
        COLLAGE_TXT_COLOR
    );
#endif

    /* 5. Find max */
    Eigen::MatrixXd ch_r_sub_mean;
    cv::cv2eigen(im_sub_mean, ch_r_sub_mean);

    Eigen::VectorXi col_max(imageWidth);
    Eigen::VectorXi val_max(imageWidth);
    double max_pixel;
    cv::minMaxIdx(im_sub_mean, NULL, &max_pixel, NULL, NULL);

    Eigen::MatrixXf::Index max_index;
    for (int cc = 0; cc < imageWidth; cc++)
    {
        int rr;
        int j;
        int k;
        double weighed_sum;
        double val_sum;

        if (cc < roiColSideCut * imageWidth ||
            cc > (1 - roiColSideCut) * imageWidth) continue;

        val_max[cc] = (int)ch_r_sub_mean.col(cc).maxCoeff(&max_index);
        col_max[cc] = (int)max_index;

        j = col_max[cc] - 1;
        while (j >= 0 &&
            im_sub_mean.at<uchar>(j, cc) > 0.8 * val_max[cc])
        {
            j -= 1;
        }
        k = col_max[cc] + 1;

        // ROI exclude
        if (k < roiRowStart * imageHeight) continue;

        while (k < imageHeight &&
            im_sub_mean.at<uchar>(k, cc) > 0.8 * val_max[cc])
        {
            k += 1;
        }

        weighed_sum = 0.;
        val_sum = 0.;

        for (int rr = j + 1; rr < k; rr++)
        {
            weighed_sum += im_sub_mean.at<uchar>(rr, cc) * (rr - j);
            val_sum += im_sub_mean.at<uchar>(rr, cc);
        }

        if (val_sum <= 0) continue;
        rr = int(weighed_sum / val_sum + j);

        max2dPoints.push_back(cv::Point2d(cc, rr));
    }

    if (max2dPoints.size() == 0)
    {
        return;
    }

#ifdef DEBUG_LASER_EXT
    // Draw detected (undistorted) 2d laser points
    for(auto const& pt: max2dPoints)
    {
        cv::circle(debugCollageFrame, pt, 1, cv::Scalar(0, 255, 0), -1);
    }
#endif

    // Eliminate distortion of found 2D points
    cv::undistortPoints(max2dPoints, undistortedPoints,
        IntrinsicMat, DistortionMat);
}

void BEPcl::getLaser2DPts(const cv::Mat& frame)
{
    max2dPoints.clear();
    undistortedPoints.clear();

    // 2D ROI - Alone Blaser Yc axis.
    int roi_cols_start = (int)(0.2 * laserFrame.cols);
    int roi_cols_end = (int)(0.8 * laserFrame.cols);

    //cv::Mat greyFrame;
    getLaserChannel(frame, laserFrame);

    cv::Point2d wpt;

    for (int cc = roi_cols_start; cc < roi_cols_end; cc++)
    { // COL
        int vmax = -1;
        int imax;
        int lmin = 256;
        int rmin = 256;
        cv::Mat col = laserFrame.col(cc);

        // This loop gets the maximum,
        // as well as the minimum on the left and right of the max.
        lmin = laserFrame.at<uchar>(0, cc);
        for (int r = 0; r < laserFrame.rows; ++r)
        {
            //wpt = warpSearchDirection(cv::Point2d(cc, r));
            int intensity = laserFrame.at<uchar>(cv::Point2d(cc, r));
            if (intensity < rmin)
            {
                rmin = intensity;
            }
            if (intensity > vmax)
            {
                vmax = intensity;
                imax = r;

                if (rmin < lmin)
                {
                    lmin = rmin;
                }
                rmin = intensity;
            }
        } //end minmax loop

        if (vmax < intensity_thresh)
        {
            continue;
        }

        double prominence = vmax - (lmin > rmin ? lmin : rmin);
        double v_thresh = vmax - 0.5 * prominence;

        // Search for left bound
        // Give it 5 pixel buffer b/c errors
        int lb;
        for (lb = imax; lb >= 0; --lb)
        {
            if (laserFrame.at<uchar>(lb, cc) < v_thresh)
            {
                break;
            }
        }

        // Search for right bound
        // Give it 5 pixel buffer b/c errors
        int rb;
        for (rb = imax; rb < laserFrame.rows; ++rb)
        {
            if (laserFrame.at<uchar>(rb, cc) < v_thresh)
            {
                break;
            }
        }

        // Do center of mass calculation
        double m0 = 0, m1 = 0;
        for (int r = lb; r <= rb; ++r)
        {
            int v_r = laserFrame.at<uchar>(r, cc);

            m0 += v_r - v_thresh;
            m1 += (v_r - v_thresh) * (r - imax);
        }

        //printf("i: %d\tlb,rb: %d, %d\t ms: %f, %f\n", cc, lb, rb, m0, m1);

        max2dPoints.push_back(cv::Point2d(cc, m1 / m0 + imax));
    }

    if (max2dPoints.size() == 0) return;

    // Eliminate distortion of found 2D points
    cv::undistortPoints(max2dPoints, undistortedPoints,
        IntrinsicMat, DistortionMat);
}

void BEPcl::getWorld3DPts()
{
    ptcld->points.clear();
    ptcld->points.reserve(undistortedPoints.size());
    // Algorithm delegated to reconstruct3DPoint
    for (cv::Point2d pt : undistortedPoints)
    {
        // only for valid points
        auto reconstPt = reconstruct3DPoint(pt);
        ptcld->points.push_back(reconstPt);
    }

    //    // dealing with last points, which should be the detected corner
    //    edge_point_3d_mm = ptcld[pts2d.size()-1]; // get the last point
    //
    //    cout << "edge_2d(mm) getWorld3DPts<<"
    //                << pts2d[pts2d.size()-1].x << ",\t"
    //                << pts2d[pts2d.size()-1].y << "\r\n";
    //
    //    cout << "edge_3d(mm)<<"
    //            << edge_point_3d_mm.x << ",\t"
    //            << edge_point_3d_mm.y << ",\t"
    //            << edge_point_3d_mm.z << "\r\n";
    //
    //    return ptcld;
}

void BEPcl::getPclFromImg(const cv::Mat& frame)
{
    if (frame.size().width != imageWidth || frame.size().height != imageHeight)
    {
        ROS_WARN_ONCE("Warning: resolution does not match calibration file.");
        return;
    }

    cv::undistort(frame, undistortedFrame, IntrinsicMat, DistortionMat);

    // Laser stripe extraction
#if LASER_EXT_METHOD == LASER_EXT_METHOD_THRESH
    getLaser2DPts(frame);
#else
    getLaser2DPts_COM(frame);
#endif

    // Noise filters
    if (outlierRemovalEnabled)
    {
        // TODO
    }

    // To camera frame 3D points
    getWorld3DPts();
}

BEPcl::BEPcl(string configPath)
    : ptcld(new PointCloud())
{
    if (!parseConfig(configPath))
    {
        throw std::invalid_argument("invalid filename, exiting...");
    }

    laserFrame = cv::Mat::zeros(imageHeight, imageWidth, CV_8U);

    simpleCBPercent = 1.0;
    simpleCBEnabled = true;
    redMask1Lo = {92, 14, 0};
    redMask1Hi = {180, 255, 255};
    redMask2Lo = {0, 100, 150};
    redMask2Hi = {43, 191, 202};
    redMaskDilateKernel = cv::Mat::ones(cv::Size(1, 3), CV_8U);
    redMaskCloseKernel = cv::Mat::ones(cv::Size(25, 35), CV_8U);

    satMaskLo = {0, 0, 230};
    satMaskHi = {255, 30, 255};
    satMaskDilateKernel = cv::Mat::ones(cv::Size(11, 7), CV_8U);

    outlierRemovalEnabled = true;
    roiRowStart = 0.078125; // 37.5px for 640p
    roiColSideCut = 0.015625; // 10px for 640p

#ifdef DEBUG_LASER_EXT
    debugCollageFrame = cv::Mat::zeros(imageHeight*2, imageWidth*2, CV_8UC3);
#endif
}
