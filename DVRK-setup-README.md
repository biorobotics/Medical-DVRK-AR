# Setup instruction for DVRK environment
All the versions tips below are not solid requirements, but following them will ensure the installation goes well without any problem

## Install Ubuntu 16.04

## Install ROS Kinetic
Follow the instructions here https://wiki.ros.org/kinetic/Installation/Ubuntu

## Install dvrk-ros
Follow the instructions here https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/CatkinBuild


### Install cisstNetlib, cisst, SAW components and cisst-ros bridge
These pacages are required by dvrk-ros dependency

#### Install cmake
To compile and make, you need to install cmake.  <br />
According to https://github.com/jhu-cisst/cisst/issues/74, you may encounter problem when building cisst, if you are using cmake version=3.11.4, so please avoid that version.  <br />
If the build of cisst still fails, refer to https://github.com/jhu-cisst/cisst/commit/c83c3394205e68911e9f837f1b79be41a10960f7 <br />
You can either cherry-pick the commit c83c339 or manually add the the following two lines to the file cmake/cisstMacros.cmake <br />

```sh
# make sure source file is not used before libraries are build
set_source_files_properties (${SWIG_INTERFACE_FILE} PROPERTIES DEPENDS "${MODULE_LINK_LIBRARIES}") 
```

## Git clone this Medical-DVRK-AR repo, as well as blaser-ros and VREP according to the VREP-READ.md

CHANG-FEB03
