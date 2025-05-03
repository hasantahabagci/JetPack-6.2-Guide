# OpenCV 4.10.0 Build & Installation

This README provides instructions to clone, build, and install OpenCV 4.10.0 with CUDA/cuDNN, GStreamer, and Python 3.10 bindings on Ubuntu (or Debian-based) systems using Ninja.

## Prerequisites

* Linux (Ubuntu 20.04+ or equivalent)
* NVIDIA GPU with CUDA toolkit installed
* cuDNN installed and configured
* Python 3.10

## Dependencies

```bash
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    git \
    ninja-build \
    pkg-config \
    libgtk-3-dev \
    libavcodec-dev libavformat-dev libswscale-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libjpeg-dev libpng-dev libtiff-dev \
    libopenexr-dev libtbb2 libtbb-dev \
    libdc1394-22-dev \
    python3.10-dev python3-numpy
```

> **Note:** Ensure `nvcc --version` and `ldconfig -p | grep cudnn` succeed before proceeding.

## Clone Repositories

```bash
mkdir -p ~/src && cd ~/src
# OpenCV core
git clone --branch 4.10.0 --depth 1 https://github.com/opencv/opencv.git
# OpenCV contrib modules
git clone --branch 4.10.0 --depth 1 https://github.com/opencv/opencv_contrib.git
```

## Build & Install

```bash
# Create build directory
mkdir -p ~/src/opencv/build && cd ~/src/opencv/build

# Configure with CMake and Ninja
cmake -G Ninja \
    -D CMAKE_BUILD_TYPE=Release \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D OPENCV_EXTRA_MODULES_PATH=~/src/opencv_contrib/modules \
    -D WITH_CUDA=ON \
    -D CUDA_ARCH_BIN=8.7 \
    -D WITH_CUDNN=ON \
    -D OPENCV_DNN_CUDA=ON \
    -D ENABLE_FAST_MATH=1 \
    -D CUDA_FAST_MATH=1 \
    -D WITH_GSTREAMER=ON \
    -D BUILD_TESTS=OFF \
    -D BUILD_PERF_TESTS=OFF \
    -D BUILD_EXAMPLES=OFF \
    -D BUILD_opencv_python2=OFF \
    -D BUILD_opencv_python3=ON \
    -D PYTHON_DEFAULT_EXECUTABLE=/usr/bin/python3.10 \
    -D OPENCV_ENABLE_NONFREE=ON \
    ..

# Compile and install
ninja
sudo ninja install
sudo ldconfig
```

## Verification

Run the following to verify the installation:

```bash
python3.10 -c "import cv2; print('Installed OpenCV version:', cv2.__version__)"
```

If successful, you should see:

```
Installed OpenCV version: 4.10.0
```

---

*Generated on May 4, 2025*
