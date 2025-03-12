FROM ubuntu:24.04

RUN apt update && apt upgrade -y

RUN apt install locales -y; \
    locale-gen en_US en_US.UTF-8; \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8; \
    export LANG=en_US.UTF-8

RUN apt install software-properties-common -y; \
    add-apt-repository universe -y

RUN apt update && apt install curl -y; \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update && apt install ros-dev-tools -y

RUN apt update && apt upgrade -y

RUN apt install ros-jazzy-desktop -y

RUN apt install ros-jazzy-ros-gz -y

RUN apt install -y git-all cmake gcc g++

RUN apt install -y libllvm-19-ocaml-dev libllvm19 llvm-19 llvm-19-dev llvm-19-doc llvm-19-examples llvm-19-runtime; \
    apt install -y clang-19 clang-tools-19 clang-19-doc libclang-common-19-dev libclang-19-dev libclang1-19 clang-format-19 python3-clang-19 clangd-19 clang-tidy-19

