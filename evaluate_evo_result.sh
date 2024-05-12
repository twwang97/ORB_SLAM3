#!/bin/bash
read -p 'Enter your (dataset) directory: ' pathDir
# e.g.,  pathDir = rgbd_dataset_freiburg1_desk2
echo $pathDir
pathAlgorithm=orbslam3

cd ~/r09522848/vs_code/ORB_SLAM3_0125
mkdir -p results/"$pathAlgorithm"/"$pathDir"/
# cp results/CameraTrajectory.txt results/"$pathAlgorithm"/"$pathDir"/CameraTrajectory.txt
# cp results/pCameraTrajectory.txt results/"$pathAlgorithm"/"$pathDir"/CameraTrajectory.txt
# cp results/KeyFrameTrajectory.txt results/"$pathAlgorithm"/"$pathDir"/KeyFrameTrajectory.txt
cp /home/roboticslab/r09522848/datasets/"$pathDir"/groundtruth.txt results/"$pathAlgorithm"/"$pathDir"/groundtruth.txt
cd results/"$pathAlgorithm"/"$pathDir"/
evo_ape tum groundtruth.txt CameraTrajectory.txt -va
# evo_ape tum groundtruth.txt pCameraTrajectory.txt -va