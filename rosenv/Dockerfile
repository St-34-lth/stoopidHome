# Base image
FROM ros:noetic-robot

# Copy necessary files to the container
COPY system_packages.txt  ros_entrypoint.sh  /root/

SHELL ["/bin/bash", "-c"]

# Install system packages
RUN apt-get update && \
    xargs -a /root/system_packages.txt apt-get install -y --no-install-recommends && \
    rm -rf /var/lib/apt/lists/* /root/system_packages.txt

ENV DISPLAY=:100
ENV EDITOR=nano

# Add ROS package sources and keys
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    wget -qO - https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

#make entry script executable
RUN chmod +x /root/ros_entrypoint.sh

# RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=/usr/bin sh
RUN apt-get update && apt-get install git -y

COPY pip_reqs.txt /root/

# Install pyenv, set up the environment, install Python 3.10, and install Python packages
# Install pyenv, Python 3.10, and required Python packages
COPY ./.bashrc /root/
RUN git clone https://github.com/pyenv/pyenv.git ~/.pyenv

RUN export PATH="$HOME/.pyenv/bin:$PATH" && \
    eval "$(pyenv init -)" && \
    eval "$(pyenv virtualenv-init -)" && \
    echo 'export PATH="$HOME/.pyenv/bin:$PATH"' >> /root/.bashrc && \
    echo 'eval "$(pyenv init -)"' >> /root/.bashrc && \
    echo 'eval "$(pyenv virtualenv-init -)"' >> /root/.bashrc && \
    pyenv install 3.10 && \
    pyenv global 3.10 && \
    pip3 install --extra-index-url https://rospypi.github.io/simple/ rospy && \
    pip3 install --extra-index-url https://rospypi.github.io/simple/ tf2_ros && \
    pip3 install -r /root/pip_reqs.txt

ENV PYENV_ROOT="$HOME/.pyenv"
ENV PATH="$PYENV_ROOT/shims:$PYENV_ROOT/bin:$PATH"
RUN mkdir /root/stoopidHome

COPY ./stoopidHome /root/stoopidHome/

RUN sudo rosdep fix-permissions && \
    rosdep update


# # Set the working directory
WORKDIR /root/stoopidHome/

RUN . /opt/ros/noetic/setup.bash && \
    cd /root/stoopidHome/ && catkin_make



# Set the entrypoint to the script
ENTRYPOINT ["/bin/bash", "/root/ros_entrypoint.sh"]

