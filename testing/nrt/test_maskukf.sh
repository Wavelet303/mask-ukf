cd ../objects
OBJ_NAMES=`ls`
cd ../nrt
for obj_name in $OBJ_NAMES;
do
    bash test_maskukf_single.sh $1 $obj_name
done
