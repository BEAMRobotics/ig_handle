bag=$1
downscale_factor=$2
image_topic=$3
topics=`rosbag info -y -k topics $bag  | grep topic: | awk '{gsub("- topic:", "");print}' | tr '\n' ' '`

if [[ -f $bag ]]
then
  set -- "${@:4}"
  tmp=${image_topic#*/}
  image_namespace=${tmp%/*}
  image_downscaled=$image_topic/downscaled
  rosparam set /use_sim_time true

  rosbag play $bag --clock &
  rosrun nodelet nodelet standalone image_proc/resize image:=$image_topic _scale_width:=$downscale_factor _scale_height:=$downscale_factor ~image:=$image_downscaled & 
  rosbag record $topics $image_downscaled -O downscaled.bag

else
  echo "Path to bag does not exist. Exiting"
  echo "Usage: ./downscale.sh [path_to_bag] [downscale_factor] [image_topic]"
fi
