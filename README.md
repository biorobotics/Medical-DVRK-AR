# Medical-DVRK-AR
MRSD Class project


## Prepare urdf file
(You need at first build a workspace following https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/CatkinBuild)

Open another terminal, source your devel/setup.bash in your workspace
```sh
source ~/your_work_space/devel/setup.bash
```

Convert xacro file to urdf file. It's in the model folder in src
```sh
source ~/your_work_space//src/dvrk-ros/dvrk_model/model
rosrun xacro xacro your_model_file.xacro > your_model_file.urdf
```

## Run Coppelia


Download EDU version of Coppelia (Player version doesn't support import urdf file)
http://coppeliarobotics.com/ubuntuVersions

Extract it

Open a terminal, run roscore
```sh
roscore
```


Open another terminal, source your devel/setup.bash in your workspace, run the .sh file
```sh
source ~/your_work_space/devel/setup.bash
~/your_Coppelia_root_folder/coppeliaSim.sh
```


## Input URDF file in Coppelia
Top bar > plugin > import > import urdf > choose the file you converted
