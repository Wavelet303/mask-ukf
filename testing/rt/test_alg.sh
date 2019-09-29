ALG_NAME=$1
SEG_TYPE=$2
OBJ_NAME=$3
OBJ_NAME_NO_ID=`echo $OBJ_NAME | cut -c5-`
DATA=../../datasets/dataset_rt/${OBJ_NAME}
OUTPUT=../../results/$ALG_NAME/rt/${SEG_TYPE}/validation/${OBJ_NAME}

for video_id in `cat ../objects/${OBJ_NAME} | sed -n 2p`;
do
    VIDEO_PATH=${DATA}/${video_id}/otd
    vmtouch -vt ${VIDEO_PATH}
    mkdir -p ${OUTPUT}/${video_id}
    rm -f $OUTPUT/$video_id/*.txt
    yarpdataplayer --withExtraTimeCol 2 &
    sleep 2
    echo "load ${VIDEO_PATH}" | yarp rpc /yarpdataplayer/rpc:i
    sleep 2
    if [ "$ALG_NAME" == "mask-ukf" ]; then
        object-tracking --from config_ycbvideo.ini --algorithm UKF --POINT_CLOUD_FILTERING::outlier_rejection true --OBJECT::object_name $OBJ_NAME --OBJECT::path ${VIDEO_PATH} --LOG::enable_log true --LOG::absolute_log_path $OUTPUT/$video_id &
    else
        object-tracking --from config_ycbvideo.ini --algorithm ICP --POINT_CLOUD_FILTERING::outlier_rejection false --OBJECT::object_name $OBJ_NAME --OBJECT::path ${VIDEO_PATH} --LOG::enable_log true --LOG::absolute_log_path $OUTPUT/$video_id &
    fi
    yarpview --name /object-tracking/viewer/validation:i &
    sleep 2
    yarp connect /depthCamera/rgbImage:o /object-tracking/rgbImage:i
    yarp connect /depthCamera/depthImage:o /object-tracking/depthImage:i
    yarp connect /instanceSegmenter/maskImage:o /object-tracking/mask:i
    yarp connect /object-tracking/segmentation:o /object-tracking/validator2d/rgbImage:i
    yarp connect /object-tracking/validator2d/validationImage:o /object-tracking/viewer/validation:i
    yarp connect /object-tracking-ground-truth/${OBJ_NAME_NO_ID}:o /object-tracking/ground-truth:i
    echo "run filter" | yarp rpc /object-tracking/cmd:i
    sleep 1
    echo "play" | yarp rpc /yarpdataplayer/rpc:i
    sleep 1
    status=`echo "getFrame depth" | yarp rpc /yarpdataplayer/rpc:i`
    while [ ${status: -2} != "-1" ]
    do
	status=`echo "getFrame depth" | yarp rpc /yarpdataplayer/rpc:i`
	sleep 1
    done
    echo "disable_log" |  yarp rpc /object-tracking/cmd:i
    echo "quit" | yarp rpc /yarpdataplayer/rpc:i
    killall -9 yarpview
    killall -9 object-tracking
    sleep 1
    yarp clean --timeout 0.1
done
