#include <stdio.h>
#include "pclGen.hpp"

int main(int argc, char** argv)
{
  if (argc < 1){
    std::cout << "gib filename" << std::endl;
    return 0;
  }

  std::string imname(argv[1]);
  BEPcl pcl_gen("/home/biorobotics_sara/blaser-ros/src/blaser-ros/blaser_pcl/config/calib_A1001_640x480.yaml");

  cv::Mat im = cv::imread(imname);
  pcl_gen.getPclFromImg(im);

  std::cout << "distorted 2D points\n";
  for (int i=0; i<pcl_gen.max2dPoints.size(); ++i)
  {
    std::cout << pcl_gen.max2dPoints[i] << std::endl;
  }

  std::cout << "undistorted 2D points\n";
  for (int i=0; i<pcl_gen.undistortedPoints.size(); ++i)
  {
    std::cout << pcl_gen.undistortedPoints[i] << std::endl;
  }

  std::cout << "3D points\n";
  for (int i=0; i<pcl_gen.ptcld->size(); ++i)
  {
    std::cout << (*pcl_gen.ptcld)[i] << std::endl;
  }
}

