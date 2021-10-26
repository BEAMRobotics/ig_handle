bag=$1
image_topic=$2
topics=`rosbag info -y -k topics $bag  | grep topic: | awk '{gsub("- topic:", "");print}' | tr '\n' ' '`


if [[ -f $bag ]]
then
  set -- "${@:3}"
  tmp=${image_topic#*/}
  image_namespace=${tmp%/*}
  image_mono=/$image_namespace/image_mono

  rosparam set /use_sim_time true

  rosbag play $bag --clock & 
  rosrun nodelet nodelet standalone image_proc/debayer image_raw:=$image_topic image_mono:=$image_mono &  
  rosbag record $image_downscaled $topics -O mono.bag

else
  echo "Path to bag does not exist. Exiting"
  echo "Usage: ./debayer.sh [path_to_bag] [raw_image_topic]"
fi
