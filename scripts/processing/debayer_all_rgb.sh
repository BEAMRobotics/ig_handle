bag_input=$1
bag_output=$2
topics=`rosbag info -y -k topics $bag_input  | grep topic: | awk '{gsub("- topic:", "");print}' | tr '\n' ' '`
cam1="/F1/image_raw"
cam2="/F2/image_raw"
cam3="/F3/image_raw"
cam4="/F4/image_raw"
topics_filterd=${topics}
topics_filterd=${topics_filterd//${cam1}/}
topics_filterd=${topics_filterd//${cam2}/}
topics_filterd=${topics_filterd//${cam3}/}
topics_filterd=${topics_filterd//${cam4}/}

if [[ -f $bag_input ]]
then
  echo "processing bag: $bag_input"
else
  echo "Path to bag does not exist. Exiting"
  echo "Usage: ./debayer_all_rgb.sh [path_to_input_bag] [path_to_output_bag]"
  exit 0
fi


if [[ -z "$bag_output" ]]
then
  echo "Path to bag does not exist. Exiting"
  echo "Usage: ./debayer_all_rgb.sh [path_to_input_bag] [path_to_output_bag]"
  exit 0
fi

rosparam set /use_sim_time true &
rosbag play $bag_input -r 0.25 --clock &
rosrun nodelet nodelet manager __name:=nodelet_manager &
rosrun nodelet nodelet load image_proc/debayer nodelet_manager __name:=nodelet1 image_raw:=/F1/image_raw image_color:=/F1/image_color &  
rosrun nodelet nodelet load image_proc/debayer nodelet_manager __name:=nodelet2 image_raw:=/F2/image_raw image_color:=/F2/image_color &  
rosrun nodelet nodelet load image_proc/debayer nodelet_manager __name:=nodelet3 image_raw:=/F3/image_raw image_color:=/F3/image_color &  
rosrun nodelet nodelet load image_proc/debayer nodelet_manager __name:=nodelet4 image_raw:=/F4/image_raw image_color:=/F4/image_color &  
rosbag record $topics_filterd /F1/image_color /F2/image_color /F3/image_color /F4/image_color -O $bag_output