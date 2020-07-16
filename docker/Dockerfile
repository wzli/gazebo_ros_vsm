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
  ros-melodic-gazebo-ros-pkgs \
  ros-melodic-gazebo-ros-control \
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

# download gazebo models
RUN curl -sL https://github.com/osrf/gazebo_models/archive/master.zip -o models.zip \
  && unzip models.zip && rm models.zip && mv -n gazebo_models-master/* /usr/share/gazebo-9/models/
    
# build gzweb
ENV GZWEB_WS=/gzweb
WORKDIR ${GZWEB_WS}
RUN curl -sL https://github.com/osrf/gzweb/archive/master.zip -o gzweb.zip \
  && unzip gzweb.zip && rm gzweb.zip && mv gzweb-master/* . \
  && . /usr/share/gazebo/setup.sh \
  && xvfb-run -s "-screen 0 1280x1024x24" ./deploy.sh -m local -t

# install husky sim
ENV GZWEB_ASSETS=${GZWEB_WS}/http/client/assets
ENV ROS_SHARE=/opt/ros/melodic/share
RUN apt update && apt install -y \
  ros-melodic-husky-simulator \
  ros-melodic-husky-navigation \
  && rm -rf /var/lib/apt/lists/* \
  && cp -rn ${ROS_SHARE}/husky_description/meshes/* ${GZWEB_ASSETS}/husky/meshes \
  && find ${ROS_SHARE}/husky_description -name "*.urdf*" -exec sed -i 's/package:\/\/husky_description/model:\/\/husky/g' {} +

# build catkin packages
ENV CATKIN_WS=/opt/catkin_ws
WORKDIR ${CATKIN_WS}
COPY gazebo_vsm src/gazebo_vsm
COPY gazebo_vsm_demo src/gazebo_vsm_demo
RUN . /opt/ros/melodic/setup.sh && catkin init && catkin build -DCMAKE_BUILD_TYPE=Release --force-color

CMD bash -c "source ${CATKIN_WS}/devel/setup.bash && roslaunch gazebo_vsm_demo gazebo_vsm_demo.launch" \
  & cd ${GZWEB_WS} && npm start
