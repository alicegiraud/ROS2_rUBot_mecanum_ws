## **ROS2 environment on Docker based computers**

Computers with DualBoot (Windows-Linux) we need to use a Docker based setup to run ROS2 Humble.

We have created a `ros2-humble-biorobub.zip` file containing:
- Dockerfile: with all the ROS2 Humble packages needed and some tools like VSCode installed.
- docker-compose.yml: to launch the container with the proper configuration.
- cyclonedds_pc.xml: CycloneDDS unicast configuration file for the PC.
- cyclonedds_robot.xml: CycloneDDS unicast configuration file for the robot.
- entrypointpc.sh: entrypoint script to setup the environment variables when the container starts.

**Professor** has to create a Docker Image with the custom configuration on Dockerfile and upload to my DockerHub account (https://hub.docker.com/u/manelpuig).
- Download to `~/Desktop/rob` folder, the `ros2-humble-biorobub.zip` file from: https://github.com/manelpuig/ROS2_rUBot_mecanum_ws/blob/main/network_config/ros2-humble-biorobub.zip
- Unzip and create the Image:
    ````bash
    docker build -t ros2-humble-biorobub-pc:latest .
    ````
- Tag the image for Docker Hub
    ````bash
    docker tag ros2-humble-biorobub-pc:latest manelpuig/ros2-humble-biorobub-pc:latest
    ````
- Login to Docker Hub from terminal and follow instructions to autenticate
    ````bash
    docker login
    ````
- Push the image to Docker Hub
    ````bash
    docker push manelpuig/ros2-humble-biorobub-pc:latest
    ````
- Once the image is pushed properly to my Dockerhub account, I have to delete the local image tag:
    ````bash
    docker rmi ros2-humble-biorobub-pc:latest
    ````
**Students** in the lab they only need to:
- Unzip the `ros2-humble-biorobub.zip` file in a `~/Desktop/rob` folder on Linux PC
- review on:
    - `docker-compose.yml` file: `ROS_DOMAIN_ID` variable to match your robot.
    - `cyclonedds_pc.xml` file: IPs to match your PC and robot.
    - `cyclonedds_robot.xml` file: IPs to match your robot and PC.
- Open a terminal in the `~/Desktop/rob/ros2-humble-biorobub` folder and run:
    ````bash
    xhost +local:root            # allow X11 for graphs in container
    cd ~/Desktop/ros2-humble-biorobub
    docker-compose up -d
    docker exec -it pc_humble bash
    code .                     # open VSCode inside the container
    ros2 topic list
    ````
- Open `.bashrc` file inside the container and verify it contains:
    ````bash
    source /opt/ros/humble/setup.bash
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
    source ~/Desktop/ROS2_rUBot_mecanum_ws/install/setup.bash
    cd ~/Desktop/ROS2_rUBot_mecanum_ws
    export GAZEBO_MODEL_PATH=~/Desktop/ROS2_rUBot_mecanum_ws/src/my_robot_bringup/models:$GAZEBO_MODEL_PATH
    export QT_QPA_PLATFORM=xcb           # Best for RVIZ2
    export ROS_DOMAIN_ID=1               # group/domain ID
    export ROS_LOCALHOST_ONLY=0          # allow communication with other machines
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI=file:///home/student/Desktop/ROS2_rUBot_mecanum_ws/network_config/cyclonedds_pc.xml
    ````

- To stop the container:
    ````bash
    docker-compose down
    ````
- To see the Images and Containers:
    ````bash
    docker ps -a               # containers
    docker images              # images
    ````

You are ready to work with ROS2 Humble on Docker!