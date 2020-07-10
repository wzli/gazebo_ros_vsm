FROM ros:melodic

RUN apt update && apt install -y \
  screen \
  vim \
  python-pip \
  python-catkin-tools \
  curl \
  # gazebo stuff
  gazebo9 \
  libgazebo9-dev \
  ros-melodic-gazebo-ros \
  ros-melodic-xacro \
  # gzweb stuff
  pkg-config \
  psmisc \
  xvfb \
  libjansson-dev \
  libboost-dev \
  imagemagick \
  libtinyxml-dev \
  && curl -sL https://deb.nodesource.com/setup_10.x | bash - \
  && apt install -y nodejs \
  && rm -rf /var/lib/apt/lists/*

# build gzweb
ENV GZWEB_WS=/gzweb
WORKDIR ${GZWEB_WS}
RUN curl -SL https://github.com/osrf/gzweb/archive/master.zip -o gzweb.zip \
  && unzip gzweb.zip && rm gzweb.zip && mv gzweb-master/* . \
  && . /usr/share/gazebo/setup.sh \
  && xvfb-run -s "-screen 0 1280x1024x24" ./deploy.sh -m
  #&& xvfb-run -s "-screen 0 1280x1024x24" ./deploy.sh -m local

ENV CATKIN_WS=/opt/catkin_ws

#WORKDIR ${CATKIN_WS}/src
## setup turtlebot3 sim
#ADD https://github.com/ROBOTIS-GIT/turtlebot3_simulations/archive/master.zip .
#RUN unzip '*.zip' && rm *.zip \
#  && apt update && rosdep update \
#  && rosdep install --from-paths . --ignore-src -r -y \
#  && rm -rf /var/lib/apt/lists/*
#
#RUN cd turtlebot3_simulations-master/turtlebot3_gazebo \
#  && find models/ -name "materials" -exec cp -rn {} media \; \
#  && find models/ -name "meshes" -exec cp -rn {} media \; \
#  && mv media models

# build catkin package 
WORKDIR ${CATKIN_WS}
RUN mkdir src
RUN . /opt/ros/melodic/setup.sh && catkin init && catkin build -DCMAKE_BUILD_TYPE=Release --force-color
