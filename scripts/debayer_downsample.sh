
bag=$1

if [[ -f $bag ]]
then
  image_topic=$2
  set -- "${@:3}"

  tmp=${image_topic#*/}
  image_namespace=${tmp%/*}

  image_mono=$image_namespace/image_mono
  image_downscaled=$image_namespace/downscaled/image_mono

  rosparam set /use_sim_time true

  rosbag play $bag --clock & 
  rosrun nodelet nodelet standalone image_proc/debayer image_raw:=$image_topic image_mono:=$image_mono &  
  rosrun nodelet nodelet standalone image_proc/resize image:=$image_mono _scale_width:=0.25 _scale_height:=0.25 ~image:=$image_downscaled & 
  rosbag record $image_downscaled $@

else
  echo "Path to bag does not exist. Exiting"
  echo "Usage: ./debayeranddownsample.sh [path_to_bag] [raw_image_topic] [rest of topics to keep]"
fi
