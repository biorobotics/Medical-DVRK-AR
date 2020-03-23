#ifndef __CONFIGURATION__HPP__
#define __CONFIGURATION__HPP__

#include <opencv2/opencv.hpp>

/*************************************************************\
|* Reserved values, do not change.                           *|
\*************************************************************/
// Old naive threshold method
#define LASER_EXT_METHOD_THRESH (0)
// Mask segmentation method
#define LASER_EXT_METHOD_COM (1)

/*************************************************************\
|* Debug options, enabling might cause degraded performance. *|
\*************************************************************/
// DEBUG_LASER_EXT enables debug collage output
#define DEBUG_LASER_EXT
// DYN_RECONFIG_LASER_PLANE_VIS option to publish laser plane
#define DYN_RECONFIG_LASER_PLANE_VIS
// DYN_RECONFIG_RAY_VIS option to publish laser lines for identified raycasts
#define DYN_RECONFIG_RAY_VIS
// Set whether last frame should be retained for reconfigure convenience
#define DYN_RECONFIG_UPDATE

/*************************************************************\
|* Laser line extraction configurations.                     *|
\*************************************************************/
// Set which laser extraction method to use
#define LASER_EXT_METHOD LASER_EXT_METHOD_COM

#endif /* __CONFIGURATION__HPP__ */