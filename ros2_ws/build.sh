# Install conan deps
set -e

source /opt/ros/dashing/setup.bash

mkdir -p src/.build

conan install --build missing -if src/.build .

colcon build --cmake-args --event-handlers 
#-DCMAKE_VERBOSE_MAKEFILE:BOOL=ON 