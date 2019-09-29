ALG_NAME=$1
SEG_TYPE=$2
OBJ_NAME=$3
OBJ_NAME_NO_ID=`echo $OBJ_NAME | cut -c5-`
DATA=../../datasets/dataset_nrt
OUTPUT=../../results/$ALG_NAME/nrt/${SEG_TYPE}/validation/${OBJ_NAME}

for video_id in `cat ../objects/${OBJ_NAME} | sed -n 2p`;
do
    mkdir -p $OUTPUT/$video_id;
    rm -f $OUTPUT/$video_id/*.txt
    yarpview --name /object-tracking/viewer/validator:i &
    if [ "$ALG_NAME" == "mask-ukf" ]; then
        object-tracking --from config_ycbvideonrt.ini --algorithm UKF --POINT_CLOUD_FILTERING::outlier_rejection true --SEGMENTATION::masks_set $SEG_TYPE --OBJECT::object_name $OBJ_NAME --OBJECT::path $DATA/$video_id --LOG::enable_log true --LOG::absolute_log_path $OUTPUT/$video_id --autostart true;
    else
        object-tracking --from config_ycbvideonrt.ini --algorithm ICP --POINT_CLOUD_FILTERING::outlier_rejection false --SEGMENTATION::masks_set $SEG_TYPE --OBJECT::object_name $OBJ_NAME --OBJECT::path $DATA/$video_id --LOG::enable_log true --LOG::absolute_log_path $OUTPUT/$video_id --autostart true;
    fi
    killall -9 yarpview
    cp $DATA/$video_id/gt_$OBJ_NAME_NO_ID/data.log $OUTPUT/$video_id/object-tracking_ground_truth.txt;
done
