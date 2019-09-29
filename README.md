# MaskUKF

This is the official repository of MaskUKF, an instance segmentation aided Unscented Kalman Filter for 6D object pose tracking.

<p align="center"><img src="https://github.com/robotology/mask-ukf/blob/master/assets/scheme.png" alt="in hand object tracking"/></p>

**NEW**: Code/scripts for evaluation and testing available. Dataset for real-time scenario to be released soon.

## Overview
- [Dependencies](#dependencies)
- [Instructions for evaluation](#instructions-for-evaluation)
- [Instructions for testing](#instructions-for-testing)
- [Structure of the results data](#structure-of-the-results-data)
- [Results](#results)

## Dependencies
Code has been tested on `Arch Linux` with the following dependencies with the indicated version. Please note that the indicated version **is not** the minimum required version.

### For evaluation
- [`Eigen 3 (3.3.7-2)`](http://eigen.tuxfamily.org/index.php?title=Main_Page)

### For testing
- [`armadillo (http://pointclouds.org/)`](http://arma.sourceforge.net/)
- [`BayesFilters (0.9.100)`](https://github.com/robotology/bayes-filters-lib/tree/devel)
- [`Eigen 3 (3.3.7-2)`](http://eigen.tuxfamily.org/index.php?title=Main_Page)
- [`mlpack (4.0.0)`](https://www.mlpack.org/)
- [`OpenCV (3.4.0)`](https://opencv.org/)
- [`PCL (1.9.1)`](http://pointclouds.org/)

> Note: while we use `Eigen` for all the mathematical computations, the library `ml-pack` relies on `armadillo`

### Optional
- `OpenMP (8.0.0-1)` (optional for faster execution)
> We use `OpenMP` for faster evaluation of the `UKF` measurement model and for faster evaluation of the `ADD-S` metric. If possible, you should use a version of `mlpack` compiled against `OpenMP` to obtain faster execution of the outlier rejection procedure.

## Instructions for evaluation
1. Clone the repository, build and install
   ```
   git clone https://robotology/mask-ukf
   cd mask-ukf
   mkdir build
   cd build
   cmake ../ -DCMAKE_PREFIX_PATH=<INSTALL_PATH> [-DUSE_OPENMP=ON]
   make install
   ```
   > Build with `OpenMP` is optional.

   `<INSTALL_PATH>` is the path where the executables will be installed. Please make sure that this path is reachable in    your environment, e.g. by setting 
   ```
   export PATH=$PATH:<INSTALL_PATH>/bin
   ```
   in your environment.

2. Download the zip file [results.zip](https://figshare.com/account/verify_email/NTA2NzI.2i19MtTVnD1I6kqqnaGjHvbIFjo) file containing the results of the algorithms `MaskUKF`, `ICP` and `DenseFusion` on the `YCB Video Dataset`. We provide the output for the algorithm `DenseFusion` on all the frames of the dataset (not only on the key frames).

   ```
   wget https://ndownloader.figshare.com/files/17811737
   ```

3. Extract the zip file
   ```
   unzip results.zip -d <mask-ukf>/results
   ```
   where `<mask-ukf>` is the folder where the repository was cloned. More details on the content of the results data [here](https://github.com/robotology/mask-ukf#structure-of-the-results-data).

4. Execute the evaluation using the scripts provided in `<mask-ukf>/results/scripts`:
   - the `add-s` folder contains scripts for the `ADD-S` metric both `<2 cm` and `AUC`
   - the `rmse` folder contains scripts for the `RMSE` metric
   - the `rmse_velocity` folder contains scripts for the `RMSE` for the linear and angular velocity
   
   Each script file name is of the form `eval_<alg>_<scenario>_<segmentation>.sh` where:
     - `<alg>` can be `mask-ukf`, `desnefusion` or `icp`
     - `<scenario>` can be `nrt` (i.e. masks available at each frame) or `rt` (i.e. masks from `Mask R-CNN` at 5 fps)
     - `<segmentation>` can be `gt` (i.e. ground truth), `mrcnn` (i.e. `Mask R-CNN`) or `posecnn` (i.e. masks from segmentation network of `PoseCNN`)
       
   Not all combinations of `<alg>`, `<scenario>` and `<segmentation>` are available. For example, `ADD-S` results for `DenseFusion` are available in their [repository](https://github.com/j96w/DenseFusion#results).
   
## Structure of the results data
The results data is organized in folders according to the following structure

```
<alg>/<scenario>/<segmentation>/validation/<class_name>/<video_id>
```

where 
- `<alg>` can be `mask-ukf`, `icp` or `dense_fusion`
- `<scenario>` can be `nrt` (i.e. masks available at each frame) or `rt` (i.e. masks from `Mask R-CNN` at 5 fps)
- `<segmentation>` can be `gt` (i.e. ground truth masks), `mrcnn` (i.e. masks from `Mask R-CNN`) or `posecnn` (i.e. masks from segmentation network of `PoseCNN`)
- `<class_name>` is the name of one of the classes belonging to the testing set of the YCB Video Dataset
- `<video_id>` is the video id of one of the video belonging to the testing set of the YCB Video Dataset

Please note that not all the combinations are available. For example, `DenseFusion` is available only in the `nrt` scenario.

Within each folder, two files are available:
- `object-tracking_estimate.txt` contains, for each frame, the Cartesian position, the axis angle representation of the orientation, the Cartesian velocity, the angular rates associated to the Euler 'ZYX' representation and the index of the frame
- `object-tracking_ground_truth.txt` contains, for each frame, starting from column no. 3, the Cartesian position and the axis angle representation of the orientation

For `nrt` data, the index of the frame corresponds to the same index of the corresponding frame in sequence `<video_id>` from the YCB Video Dataset. For `rt` data, instead, the `ID` of the frame corresponds to the number of frames processed from the beginning of the real-time experiment. Frames might be missing in `DenseFusion` sequences due to missing frames in the `PoseCNN` segmentation.

For `dense_fusion` and `icp` the velocities are not available and are substituted with zeros.

## Results

### ADD-S (masks available at each frame)

<p align="center"><img src="https://github.com/robotology/mask-ukf/blob/master/assets/adds.png" alt="in hand object tracking" width="750" height="420"/></p>

### RMSE (masks available at each frame)

<p align="center"><img src="https://github.com/robotology/mask-ukf/blob/master/assets/rmse.png" alt="in hand object tracking" width="750" height="420"/></p>


### ADD-S and RMSE (masks from Mask R-CNN at 5 fps)

<p align="center"><img src="https://github.com/robotology/mask-ukf/blob/master/assets/rt.png" alt="in hand object tracking" width="450" height="450"/></p>

