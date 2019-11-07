 #！/bin/bash

# unzip all ".zip" files to color images and disparity images
for files in `ls ./*.zip`
do
    unzip $files
done

# move and rename color-depth images
i=1000
for files in `ls ./c_*.jpg`
do
    echo $files"  to  ""Images/"$i".jpg"
    mv $files "Images/"$i".jpg"
    i=$(($i+1))
done

i=1000
for files in `ls ./d_*.jpg`
do
    echo $files"  to  ""Depths/"$i".jpg"
    mv $files "Depths/"$i".jpg"
    i=$(($i+1))
done

