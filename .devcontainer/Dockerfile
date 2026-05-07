FROM mcr.microsoft.com/devcontainers/cpp:1-ubuntu-22.04

# Install system packages via apt. gcc-13 is sourced from the ubuntu-toolchain-r/test
# PPA because Ubuntu 22.04's default gcc-11 lacks the C++20 <format> header.
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install --no-install-recommends software-properties-common \
    && add-apt-repository -y ppa:ubuntu-toolchain-r/test \
    && apt-get update \
    && apt-get -y install --no-install-recommends \
    libtool \
    autoconf \
    automake \
    git-lfs \
    pkg-config \
    ninja-build \
    bison \
    flex \
    wget \
    curl \
    gcc-13 \
    g++-13

# Make gcc-13/g++-13 the default so any tooling that calls bare `gcc`/`g++`
# (e.g. VS Code's CMake Tools) picks them up without needing CC/CXX env vars.
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 130 \
    --slave /usr/bin/g++ g++ /usr/bin/g++-13 \
    --slave /usr/bin/gcov gcov /usr/bin/gcov-13

# Install miniconda for PDAL
# Note: Installing PDAL with vcpkg is possible, but it builds from source and is very slow.
# Using the precompiled conda package is much faster.
RUN wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /tmp/miniconda.sh && \
    chmod +x /tmp/miniconda.sh && \
    /tmp/miniconda.sh -b -p /opt/miniconda && \
    rm /tmp/miniconda.sh

# Add conda to PATH for all users
ENV PATH="/opt/miniconda/bin:$PATH"

# Configure conda
RUN conda config --remove channels defaults && \
    conda config --add channels conda-forge && \
    conda config --set channel_priority strict

# Accept Conda Terms of Service
RUN conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main && \
    conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r

# Install PDAL via conda
RUN conda install pdal -y --quiet && \
    conda clean -a -y --quiet