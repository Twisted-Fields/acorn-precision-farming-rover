FOLDER=/home/taylor/Software/kernel_acorn
FOLDER=$FOLDER/net/can/
PATCH_FILE=$PWD/isotp.patch
cd $FOLDER
echo $PWD
patch -p3 < $PATCH_FILE
