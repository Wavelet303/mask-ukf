# MaskUKF

This is the official repository of MaskUKF, an instance segmentation aided Unscented Kalman Filter for 6D object pose tracking.

<p align="center"><img src="https://github.com/robotology/mask-ukf/blob/master/assets/readme_fig.png" alt="in hand object tracking" width="449" height="220"/></p>

News: 26/09/2019 - code for evaluation available. Code of the actual implementation to be released soon.

## Dependencies
- `Eigen 3`
- `OpenMP` (optional for faster execution)

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

`<INSTALL_PATH>` is the path where the executables will be installed. Please make sure that this path is reachable in your environment, e.g. by setting 
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
where `<mask-ukf>` is the folder where the repository was cloned.

4. Execute the evaluation using the scripts provided in `<mask-ukf>/results/scripts`:
   - the `add-s` folder contains scripts for the `ADD-S` metric both `<2 cm` and `AUC`
   - the `rmse` folder contains scripts for the `RMSE` metric
   - the `rmse_velocity` folder contains scripts for the `RMSE` for the linear and angular velocity
   
   Each script file name is of the form `eval_<alg>_<scenario>_<segmentation>.sh` where:
     - `<alg>` can be `mask-ukf`, `desnefusion` or `icp`
     - `<scenario>` can be `nrt` (i.e. masks available at each frame) or `rt` (i.e. masks from `Mask R-CNN` at 5 fps)
     - `<segmentation>` can be `gt` (i.e. ground truth), `mrcnn` (i.e. `Mask R-CNN`) or `posecnn` (i.e. masks from segmentation network of `PoseCNN`)
       
   Not all combinations of `<alg>`, `<scenario>` and `<segmentation>` are available. For example, `ADD-S` results for `DenseFusion` are available in their [repository](https://github.com/j96w/DenseFusion#results).
