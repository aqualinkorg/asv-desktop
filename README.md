### Working inside of the ROS2 Dev Container:
All subsequent steps should be performed within the development container. To enter the container:
```bash
# TIP: You can create an alias for this or modify the script to make the args a default
./run.sh --nvidia --priv --host --ipc
```

### To build:
From the top-level directory:
```bash
cd conan-packages
./build.sh
cd ..

cd ros2_ws
./build.sh
```

### To run a node:
From ros2_ws:
```bash
source ./install/setup.bash
ros2 run <package_name> <exe_name>
```

To run the empty mockbot example:
```bash
source ./install/setup.bash
ros2 run mockbot mockbot
```