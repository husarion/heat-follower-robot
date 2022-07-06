FROM ros:galactic


SHELL ["/bin/bash", "-c"]

RUN apt update && apt install -y \
    python3-pip \
    python3-colcon-common-extensions 

RUN apt upgrade -y 
RUN pip install opencv-python
RUN apt-get update
RUN apt-get install ffmpeg libsm6 libxext6  -y
RUN apt-get install ros-galactic-cv-bridge -y
RUN apt-get install ros-galactic-vision-opencv -y
WORKDIR /app
RUN mkdir -p ros2_ws/src
COPY ./py_pkg app/ros2_ws/src/py_pkg
RUN cd app/ros2_ws
RUN colcon build
RUN source install/setup.bash

COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD [ "ros2", "run", "py_pkg", "thermal_subscriber"]
