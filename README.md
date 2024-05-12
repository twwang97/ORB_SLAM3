# ORB-SLAM3

* This program is forked from [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) released in 2021. 

---

## Building ORB-SLAM3 library
Refer to this bash script ```./build.sh``` in this [ORB-SLAM3 github](https://github.com/UZ-SLAMLab/ORB_SLAM3)

---

## Running ORB-SLAM3 with D435i camera
```
./Examples/RGB-D/rgbd_realsense_D435i Vocabulary/ORBvoc.txt ./Examples/RGB-D/RealSense_D435i.yaml
```

---

## Running ORB-SLAM3 with datasets
```
./Examples/RGB-D/rgbd_data_realsense ./Vocabulary/ORBvoc.txt ./Examples/RGB-D/RealSense_D435i.yaml ~/r09522848/datasets/data_outdoor_straight_zigzag_0130 ~/r09522848/datasets/data_outdoor_straight_zigzag_0130/associated.txt
./Examples/RGB-D/rgbd_tum ./Vocabulary/ORBvoc.txt ./Examples/RGB-D/TUM1.yaml ~/r09522848/datasets/rgbd_dataset_freiburg1_desk2 ~/r09522848/datasets/rgbd_dataset_freiburg1_desk2/associated.txt
./Examples/RGB-D/rgbd_data_realsense ./Vocabulary/ORBvoc.txt ./Examples/RGB-D/RealSense_D435i.yaml ~/r09522848/datasets/data0130_outdoor_ntu_small_loop ~/r09522848/datasets/data0130_outdoor_ntu_small_loop/associated.txt

```

#### Evaluation
```
./euroc_eval_examples
./tum_vi_eval_examples
```

#### evo
```
evo_ape tum  ~/r09522848/ORB_SLAM3_0720/Datasets/rgbd_dataset_freiburg3_walking_halfsphere/groundtruth.txt CameraTrajectory.txt -r full -va
```

* You may refer to this bash script ```evaluate_evo_result.sh``` .
See the official [evo](https://github.com/MichaelGrupp/evo) for more details. 

