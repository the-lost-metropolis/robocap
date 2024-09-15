sudo apt update
cd /home/developer/repo
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install