# Base image
FROM ros:noetic-robot

# Copy necessary files to the container

COPY system_packages.txt test_reqs.tx ./project_files/root/ /root/


SHELL ["/bin/bash", "-c"]

# Install system packages and wget
RUN apt-get update && \
    xargs -a /root/system_packages.txt apt-get install -y --no-install-recommends && \
    rm -rf /var/lib/apt/lists/* /root/system_packages.txt
    

# Add FNM to the path explicitly
RUN  curl -fsSL https://fnm.vercel.app/install | bash -s -- --install-dir "/root/.fnm" --skip-shell && \  
    export PATH="/root/.fnm/:$PATH" && \
    eval "$(/root/.fnm/fnm env --use-on-cd --shell bash)" && \
    fnm install 20 && \
    fnm use 20

ENV PATH="/root/.fnm/:${PATH}" 
ENV DISPLAY=:100
ENV EDITOR=nano
RUN apt-get update && apt-get install git -y
RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=/usr/bin sh

ENV PYENV_ROOT="$HOME/.pyenv"
ENV PATH="$PYENV_ROOT/shims:$PYENV_ROOT/bin:$PATH"

RUN export PATH="$HOME/.pyenv/bin:$PATH" && \
    eval "$(pyenv init -)" && \
    eval "$(pyenv virtualenv-init -)" && \
    echo 'export PATH="$HOME/.pyenv/bin:$PATH"' >> /root/.bashrc && \
    echo 'eval "$(pyenv init -)"' >> /root/.bashrc && \
    echo 'eval "$(pyenv virtualenv-init -)"' >> /root/.bashrc && \
    pyenv install 3.10 && \
    pyenv global 3.10 && \
    pip3 install -r /root/test_reqs.txt

#install server packages
RUN cd /root/app/ && \ 
    eval "$(/root/.fnm/fnm env --use-on-cd --shell bash)" && \
    /root/.fnm/fnm  use 20 && \
    npm install 

#install client packages
RUN cd /root/app/dashboard && \
    eval "$(/root/.fnm/fnm env --use-on-cd --shell bash)" && \
    /root/.fnm/fnm use 20 && \ 
    npm install --force

#install arduino-cli note: installs the latest version

RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=/usr/bin sh


# Set the working directory
WORKDIR /root/
# Set environment variables

# Set the entrypoint to the script
ENTRYPOINT ["/bin/bash",  "/root/entrypoint.sh" ]

