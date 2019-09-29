cd ../objects
OBJ_NAMES=`ls`
cd ../rt
for obj_name in $OBJ_NAMES;
do
    bash test_maskukf_single.sh $1 $obj_name
done
