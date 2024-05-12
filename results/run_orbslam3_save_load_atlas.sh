#!/bin/bash

cd ~/r09522848/vs_code/ORB_SLAM3_0125

###############
# # run SLAM and save Atlas (~ 2000)
# ./Examples/RGB-D/rgbd_data_realsense ./Vocabulary/ORBvoc.txt /home/roboticslab/r09522848/datasets/data0320_engBuilding2F_test03/SurfelMeshing_RealSenseD435i.yaml ~/r09522848/datasets/data0320_engBuilding2F_test03 ~/r09522848/datasets/data0320_engBuilding2F_test03/associated_short16.txt

# # load Atlas and run SLAM (1300 ~ )
# ./Examples/RGB-D/rgbd_data_realsense ./Vocabulary/ORBvoc.txt /home/roboticslab/r09522848/datasets/data0320_engBuilding2F_test01/SurfelMeshing_RealSenseD435i.yaml ~/r09522848/datasets/data0320_engBuilding2F_test01 ~/r09522848/datasets/data0320_engBuilding2F_test01/associated13.txt

##########
# # run SLAM and save Atlas (~ 2000)
# ./Examples/RGB-D/rgbd_data_realsense ./Vocabulary/ORBvoc.txt /home/roboticslab/r09522848/datasets/data0320_engBuilding2F_test06/SurfelMeshing_RealSenseD435i.yaml ~/r09522848/datasets/data0320_engBuilding2F_test06 ~/r09522848/datasets/data0320_engBuilding2F_test06/associated.txt

# # load Atlas and run SLAM (1300 ~ )
# ./Examples/RGB-D/rgbd_data_realsense ./Vocabulary/ORBvoc.txt /home/roboticslab/r09522848/datasets/data0320_engBuilding2F_test05/SurfelMeshing_RealSenseD435i.yaml ~/r09522848/datasets/data0320_engBuilding2F_test05 ~/r09522848/datasets/data0320_engBuilding2F_test05/associated.txt

# run SLAM and save Atlas
./Examples/RGB-D/rgbd_data_realsense ./Vocabulary/ORBvoc.txt ./Examples/RGB-D/RealSense_D435i.yaml ~/r09522848/datasets/data0130_outdoor_ntu_small_loop ~/r09522848/datasets/data0130_outdoor_ntu_small_loop/associated_short04.txt

# load Atlas and run SLAM
./Examples/RGB-D/rgbd_data_realsense ./Vocabulary/ORBvoc.txt ./Examples/RGB-D/RealSense_D435i_o.yaml ~/r09522848/datasets/data0130_outdoor_ntu_small_loop ~/r09522848/datasets/data0130_outdoor_ntu_small_loop/associated_short06.txt
