FROM ubuntu:20.04

# ensures that there are no prompts during build
ARG DEBIAN_FRONTEND=noninteractive

# environment dependencies
RUN apt-get update && apt-get install -y -q --no-install-recommends \
    sudo \
    apt-utils \
    locales \
    curl \
    wget \
    git \
    vim \
    python3-pip

RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# configure non-root user
ARG USERNAME=test_user

# create the dev workspace folder and make it owned by the user
RUN mkdir -p /home/$USERNAME/dev_ws

# setup working directory
WORKDIR /home/$USERNAME/dev_ws
