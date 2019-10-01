# MaskUKF

This is the official repository of MaskUKF, an instance segmentation aided Unscented Kalman Filter for 6D object pose tracking.

<p align="center"><img src="https://github.com/robotology/mask-ukf/blob/master/assets/scheme.png" alt="in hand object tracking"/></p>

**NEW**: Code/scripts for evaluation and testing available for both real-time and non-real-time scenario.

### Overview
- [Dependencies](#dependencies)
- [Instructions for evaluation](#instructions-for-evaluation)
- [Instructions for testing](#instructions-for-testing)
- [Structure of the results data](#structure-of-the-results-data)
- [Results](#results)

### Dependencies
Code has been tested on `Arch Linux` with the following dependencies with the indicated version. Please note that the indicated version **is not** the minimum required version.

#### For evaluation
- [`Eigen 3 (3.3.7-2)`](http://eigen.tuxfamily.org/index.php?title=Main_Page)

#### For testing
- [`armadillo (9.500.2-1)`](http://arma.sourceforge.net/)
- [`BayesFilters (0.9.100)`](https://github.com/robotology/bayes-filters-lib/tree/devel)
- [`Eigen 3 (3.3.7-2)`](http://eigen.tuxfamily.org/index.php?title=Main_Page)
- [`ICUBcontrib (1.13.0)`](https://github.com/robotology/icub-contrib-common)
- [`mlpack (4.0.0)`](https://www.mlpack.org/)
- [`OpenCV (3.4.0)`](https://opencv.org/)
- [`PCL (1.9.1)`](http://pointclouds.org/)
- [`YARP (3.2.1)`](https://github.com/robotology/yarp)

> Note: while we use `Eigen` for all the mathematical computations, the library `mlpack` relies on `armadillo`

#### Optional
- `OpenMP (8.0.0-1)` (optional for faster execution)
> We use `OpenMP` for faster evaluation of the `UKF` measurement model and for faster evaluation of the `ADD-S` metric. If possible, you should use a version of `mlpack` compiled against `OpenMP` to obtain faster execution of the outlier rejection procedure.

### Instructions for evaluation
These instructions allow downloading precomputed results of algorithms `MaskUKF`, `DenseFusion` and `ICP` and evaluating the `ADD-S` and `RMSE` metrics. 

If you need to test the actual algorithm and recompute the results please follow the [Instructions for testing](#instructions-for-testing) section. In case you recomputed the results, **you can skip to point (4)** for the actual evaluation of the metrics.

1. Clone the repository, build and install
   ```
   git clone https://github.com/robotology/mask-ukf
   cd mask-ukf
   mkdir build
   cd build
   cmake -DCMAKE_PREFIX_PATH=<INSTALL_PATH> [-DUSE_OPENMP=ON] ../
   make install
   ```
   > Build with `OpenMP` is optional.

   `<INSTALL_PATH>` is the path where the executables will be installed. Please make sure that this path is reachable in    your environment, e.g. by setting 
   ```
   export PATH=$PATH:<INSTALL_PATH>/bin
   ```
   in your environment.

2. Download the zip file containing the results of the algorithms `MaskUKF`, `ICP` and `DenseFusion` on the `YCB Video Dataset`. We provide the output for the algorithm `DenseFusion` on all the frames of the dataset (not only on the key frames).

   ```
   wget https://zenodo.org/record/3466491/files/results.zip
   ```

3. Extract the zip file
   ```
   unzip results.zip -d <mask-ukf>/results
   ```
   where `<mask-ukf>` is the folder where the repository was cloned. More details on the content of the results data [here](https://github.com/robotology/mask-ukf#structure-of-the-results-data).

4. Execute the evaluation using the scripts provided in `<mask-ukf>/results/scripts` (or `~/robot-code/mask-ukf/results/scripts` if you followed instructions in the [Instructions for testing](#instructions-for-testing) section):
   - the `add-s` folder contains scripts for the `ADD-S` metric both `<2 cm` and `AUC`
   - the `rmse` folder contains scripts for the `RMSE` metric
   - the `rmse_velocity` folder contains scripts for the `RMSE` for the linear and angular velocity
   
   Each script file name is of the form `eval_<alg>_<scenario>_<segmentation>.sh` where:
     - `<alg>` can be `mask-ukf`, `desnefusion` or `icp`
     - `<scenario>` can be `nrt` (i.e. masks available at each frame) or `rt` (i.e. masks from `Mask R-CNN` at 5 fps)
     - `<segmentation>` can be `gt` (i.e. ground truth), `mrcnn` (i.e. `Mask R-CNN`) or `posecnn` (i.e. masks from segmentation network of `PoseCNN`)
       
   Not all combinations of `<alg>`, `<scenario>` and `<segmentation>` are available. For example, `ADD-S` results for `DenseFusion` are available in their [repository](https://github.com/j96w/DenseFusion#results).
   
### Instructions for testing
These instructions allow building the code implementing the `MaskUKF` algorithm and the `ICP` procedure used as baseline. Additionally, they allow testing the algorithms and producing the numerical results required to evaluate the `ADD-S` and `RMSE` metrics.

For ease of retrieval of configuration files and contexts used by the algorithms, in the following we assume that all the relevant code is built and installed with `CMake` using the option `-DCMAKE_INSTALL_PREFIX=$ROBOT_INSTALL` where `$ROBOT_INSTALL` is a folder of your choice. We further assume that an environment variable `YARP_DATA_DIRS` pointing to `${ROBOT_INSTALL}/share/ICUBcontrib` exists and that the variable `PATH` is extended so as to point to `${ROBOT_INSTALL}/bin`. E.g. your `.bashrc` should contain something like
```
export PATH=${PATH}:${ROBOT_INSTALL}/bin
export YARP_DATA_DIRS=${YARP_DATA_DIRS}:${ROBOT_INSTALL}/share/ICUBcontrib
```
If these instructions are not clear to you, fell free to fire up an [issue](https://github.com/robotology/mask-ukf/issues).

1. Build and install **OR** install precompiled version of libraries `armadillo`, `Eigen`, `mlpack`, `OpenCV` and `PCL`.

2. Build and install `YARP`
   ```
   mkdir -p ~/robot-code
   cd ~/robot-code
   git clone https://github.com/robotology/yarp
   cd yarp
   git checkout v3.2.1
   mkdir build && cd build && cmake -DCMAKE_INSTALL_PREFIX=$ROBOT_INSTALL ../
   make install
   ```

3. Install `ICUBcontrib metapackage`
   ```
   mkdir -p ~/robot-code
   cd ~/robot-code
   git clone https://github.com/robotology/icub-contrib-common
   cd icub-contrib-common
   git checkout v1.13.0
   mkdir build && cd build && cmake -DCMAKE_INSTALL_PREFIX=$ROBOT_INSTALL ../
   make install
   ```

4. Build and install `BayesFilters` filtering library
   ```
   mkdir -p ~/robot-code
   cd ~/robot-code
   git clone https://github.com/robotology/bayes-filters-lib
   cd bayes-filters-lib
   git checkout 6af232e
   mkdir build && cd build && cmake -DCMAKE_INSTALL_PREFIX=$ROBOT_INSTALL ../
   make install
   ```

5. Build and install `MaskUKF` and baseline `ICP` implementations
   ```
   mkdir -p ~/robot-code
   git clone https://github.com/robotology/mask-ukf
   cd mask-ukf
   mkdir build && cd build
   cmake -DCMAKE_INSTALL_PREFIX=$ROBOT_INSTALL -DBUILD_OBJECT_TRACKING=ON [-DUSE_OPENMP=ON] ../
   make install
   ```
   > Build with `OpenMP` is optional.
   
6. Download the zip file [dataset_nrt.zip](https://istitutoitalianotecnologia-my.sharepoint.com/:u:/r/personal/nicola_piga_iit_it/Documents/dataset_nrt.zip?csf=1&e=nS5CIN) containing the dataset for **non-real-time scenario**. 

   The dataset consists of a restructured version of the `YCB Video Dataset` containing `RGB` images, png masks (ground truth masks, `PoseCNN` masks and `Mask R-CNN` masks) and 6D ground truth poses in accessible formats (no `MATLAB .mat` files involved). Extract the dataset as follows:
    ```
    unzip dataset_nrt.zip -d ~/robot-code/mask-ukf/datasets
    ```
7. Download and extract the dataset for **real-time scenario** (46.7 GB).

   The dataset consists of a restructured version of the `YCB Video Dataset` containing `RGB` images, png `Mask R-CNN` masks and 6D ground truth poses in [`YARP data player`](https://www.yarp.it/yarpdataplayer.html) compatible format. The player allows simulating a real-time scenario with images at 30 fps and masks at 5 fps (maximum frequency declared by the authors of `Mask R-CNN`).
   ```
   wget https://zenodo.org/record/3465685/files/dataset_rt.zip
   unzip dataset_rt.zip -d ~/robot-code/mask-ukf/datasets
   ```
   
8. Execute the algorithms on the `YCB Video Dataset` using the scripts provided in `~robot-code/mask-ukf/testing/<scenario>` where `<scenario>` can be `nrt` for non-real-time or `rt` for real-time. At the moment `rt` scripts cannot be used as the real-time dataset is in the process of being released. Scripts can be run on all the objects of the `YCB Video Dataset` testing set 
   ```
   bash test_<alg>.sh <segmentation>
   ```
   or on a single object
   ```
   bash test_<alg>_single.sh <segmentation> <class_name>
   ```
   where `<alg>` can be `mask-ukf` or `icp`, `<segmentation>` can be `gt` (i.e. ground truth), `mrcnn` (i.e. `Mask R-CNN`) or `posecnn` (i.e. masks from segmentation network of `PoseCNN`) and `<class_name>` is the class name (e.g. `002_master_chef_can`). Scripts for `real-time-scenario` are available with `<segmentation>=mrcnn` only.
   
   During testing, a viewer based on the `YARP` library will be available in order to inspect the current estimate of the object (the viewer shows a contour representing the projection onto the camera plane of the 6D pose of the object).
   
   Results are saved in `~/robot-code/mask-ukf/results` according to the structure explained in the [Structure of the results data](#structure-of-the-results-data) section. Each execution of the testing script **removes** any previously existing results. Evaluation of `ADD-S` and `RMSE` metrics is described in point (4) of the [Instructions for evaluation](#instructions-for-evaluation) section.
  
   
### Structure of the results data
The results data is organized in folders according to the following structure

```
<alg>/<scenario>/<segmentation>/validation/<class_name>/<video_id>
```

where 
- `<alg>` can be `mask-ukf`, `icp` or `dense_fusion`
- `<scenario>` can be `nrt` (i.e. masks available at each frame) or `rt` (i.e. masks from `Mask R-CNN` at 5 fps)
- `<segmentation>` can be `gt` (i.e. ground truth masks), `mrcnn` (i.e. masks from `Mask R-CNN`) or `posecnn` (i.e. masks from segmentation network of `PoseCNN`)
- `<class_name>` is the name of one of the classes belonging to the testing set of the `YCB Video Dataset`
- `<video_id>` is the video id of one of the video belonging to the testing set of the `YCB Video Dataset`

Please note that not all the combinations are available. For example, `DenseFusion` is available only in the `nrt` scenario.

Within each folder, two files are available:
- `object-tracking_estimate.txt` contains, for each frame, the Cartesian position, the axis angle representation of the orientation, the Cartesian velocity, the angular rates associated to the Euler 'ZYX' representation and the index of the frame
- `object-tracking_ground_truth.txt` contains, for each frame, starting from column no. 3, the Cartesian position and the axis angle representation of the orientation

For `nrt` data, the index of the frame corresponds to the same index of the corresponding frame in sequence `<video_id>` from the YCB Video Dataset. For `rt` data, instead, the `ID` of the frame corresponds to the number of frames processed from the beginning of the real-time experiment. Frames might be missing in `DenseFusion` sequences due to missing frames in the `PoseCNN` segmentation.

For `dense_fusion` and `icp` the velocities are not available and are substituted with zeros.

### Results

#### ADD-S (masks available at each frame)

<p align="center"><img src="https://github.com/robotology/mask-ukf/blob/master/assets/adds.png" alt="in hand object tracking" width="750" height="420"/></p>

#### RMSE (masks available at each frame)

<p align="center"><img src="https://github.com/robotology/mask-ukf/blob/master/assets/rmse.png" alt="in hand object tracking" width="750" height="420"/></p>


#### ADD-S and RMSE (masks from Mask R-CNN at 5 fps)

<p align="center"><img src="https://github.com/robotology/mask-ukf/blob/master/assets/rt.png" alt="in hand object tracking" width="450" height="450"/></p>

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.3466491.svg)](https://doi.org/10.5281/zenodo.3466491)
[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.3465685.svg)](https://doi.org/10.5281/zenodo.3465685)


