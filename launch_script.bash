gnome-terminal -- bash -c '
  echo "[INFO] Running joy_node...";
  ros2 run joy joy_node;
  '

gnome-terminal -- bash -c '
  echo "[INFO] Running grp5_node...";
  source ros_workspace/install/setup.bash;
  ros2 run grp5_package grp5_node;
  '

gnome-terminal -- bash -c '
  echo "admin" | sudo -S chmod 666 /dev/ttyACM*;
  MicroXRCEAgent serial --dev /dev/ttyACM* -b 115200 -v6
  '
