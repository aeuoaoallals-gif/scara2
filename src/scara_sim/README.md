## Install and Start

1. Download pkg
```bash
mkdir -p some_ws/src
git clone https://github.com/cyberbanana777/scara_sim
```

2. Build 
```bash
cd ..
colcon build --symlink-install && source install/local_setup.bash
```

3. Launch simulation
```bash
ros2 launch scara_sim scara_sim.launch.py 
```

4. Run effort commands. In the new terminal:
```bash
cd some_ws
source install/local_setup.bash
ros2 launch scara_sim effort_publisher.launch.py
```
