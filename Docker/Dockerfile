#Download base image ubuntu 18.04
FROM ubuntu:18.04

# Update Ubuntu Software repository
RUN apt-get update -y
RUN apt-get upgrade -y

# Set the locale
RUN apt-get clean && apt-get update && apt-get install -y locales
RUN locale-gen en_US.UTF-8
ENV LC_ALL="en_US.UTF-8"
ENV LC_CTYPE="en_US.UTF-8"

# Install Dep
RUN apt-get install tzdata

RUN apt-get install --no-install-recommends git ninja-build gperf \
  ccache dfu-util wget curl\
  python3-pip python3-setuptools python3-tk python3-wheel xz-utils file \
  make gcc gcc-multilib -y

# Set Path
ENV PATH="$PATH:/root/.local/bin"

# Install new Cmake
RUN pip3 install --user cmake

# Update dtc
RUN curl http://de.archive.ubuntu.com/ubuntu/pool/main/d/device-tree-compiler/device-tree-compiler_1.4.7-1_amd64.deb -o tmp/dtc.deb
RUN dpkg -i tmp/dtc.deb
RUN apt-get install -f

# Install the Zephyr Software Development Kit (SDK)
RUN curl -L https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.10.3/zephyr-sdk-0.10.3-setup.run -o tmp/zephyr-sdk-0.10.3-setup.run && chmod +x tmp/zephyr-sdk-0.10.3-setup.run
RUN tmp/zephyr-sdk-0.10.3-setup.run -- -d /root/zephyr-sdk-0.10.3
ENV ZEPHYR_TOOLCHAIN_VARIANT=zephyr
ENV ZEPHYR_SDK_INSTALL_DIR=/root/zephyr-sdk-0.10.3
RUN rm tmp/zephyr-sdk-0.10.3-setup.run

# Bootstrap west
RUN pip3 install --user west

#Clone the Zephyr Repositories
RUN /root/.local/bin/west init /root/zephyrproject
RUN cd /root/zephyrproject && /root/.local/bin/west update

#Install Python Dependencies
RUN cd /root/zephyrproject && pip3 install --user -r zephyr/scripts/requirements.txt



# DEBUGGER
RUN pip3 install --user pyocd
RUN apt-get install udev -y
RUN echo 'ATTRS{product}=="*CMSIS-DAP*", MODE="660", GROUP="plugdev", TAG+="uaccess"' > /etc/udev/rules.d/50-mbed.rules

ENV ZEPHYR_SDK_INSTALL_DIR=/root/zephyr-sdk-0.10.3
#create a directory for our project & setup a shared workfolder between the host and docker container
RUN mkdir -p /usr/src/app
VOLUME ["/usr/src/app"]

EXPOSE 4444
EXPOSE 3333

