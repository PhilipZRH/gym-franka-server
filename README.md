# Gym Franka Server
The is the server-side code intended to run on a realtime kernel with ```ros-noetic``` and ```franka-ros``` installed. We implement a cartesian impedance controller to control the robot.

We provide a gymnasium environment to interface with the server: [Gym Franka Client](https://github.com/frankaemika/gym-franka).

## Getting started
1. Setup franka-ros for ros-noetic as guided in https://frankaemika.github.io/docs/installation_linux.html.
2. Clone this repository.
3. Run ```bash ./setup.sh``` from the project's directory.
4. Start a new terminal so the workspace is properly sourced.

## Additional setups
1. To provent the CPU from entering ```Powersave``` mode and causing communication delays, install indicator-cpufreq by running ```sudo apt-get install indicator-cpufreq``` and change to ```Performance``` mode.

2. Configure the environment variables to match your networking setup.
    ```bash
    export SERVER_IP=192.168.xx.xx  # The IP of the server (this PC)
    export FCI_IP=192.168.xx.xx  # The IP of the franka control box
    ```

## Launching the server
```
python3 gym_franka_server.py
```

## Move to Home Position
```bash
roslaunch gym_franka_server home.launch robot_ip:=$FCI_IP
```

## Controlling with a Mobile Device
We provide a teleoperation example. The iOS app is available on the App Store: [XR-Controller](https://apps.apple.com/us/app/xr-controller/id6733221572).
```bash
roslaunch gym_franka_server ci_socket.launch robot_ip:=$FCI_IP address:=$SERVER_IP port:=6789 client:=iOS
```