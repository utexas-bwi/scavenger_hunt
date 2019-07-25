if [ "$1" == "bender" ]; then
  export ROS_IP=10.8.0.42
  export ROS_MASTER_URI=http://10.8.0.50:11311
elif [ "$1" == "pickles" ]; then
  export ROS_IP=10.8.0.34
  export ROS_MASTER_URI=http://10.8.0.50:11311
elif [ "$1" == "kane" ]; then
  export ROS_IP=10.8.0.50
  export ROS_MASTER_URI=http://localhost:11311
else
  echo "Unrecognized machine [$1]"
  exit 1
fi
