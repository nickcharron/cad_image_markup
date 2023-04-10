#!/bin/bash
set -e

DEPS_DIR="/tmp/image_cad_markup_dependencies"

main()
{
    install_routine $1
}

install_routine()
{
    sudo -v
    install_eigen3
    install_ceres
    install_pcl
    install_gflags
    install_catch2
    install_opencv
}

install_ceres()
{
    CERES_DIR="ceres-solver-1.14.0"
    BUILD_DIR="build"

    sudo apt-get -qq install libgoogle-glog-dev libatlas-base-dev > /dev/null
    # this install script is for local machines.
    if (find /usr/local/lib -name libceres.so | grep -q /usr/local/lib); then
        echo "Ceres is already installed."
    else
        echo "Installing Ceres 1.14.0 ..."
        mkdir -p "$DEPS_DIR"
        cd "$DEPS_DIR"

        if [ ! -d "$CERES_DIR" ]; then
          wget "http://ceres-solver.org/$CERES_DIR.tar.gz"
          tar zxf "$CERES_DIR.tar.gz"
          rm -rf "$CERES_DIR.tar.gz"
        fi

        cd $CERES_DIR
        if [ ! -d "$BUILD_DIR" ]; then
          mkdir -p $BUILD_DIR
          cd $BUILD_DIR
          cmake ..
          make -j1
          make test
        fi

        cd $DEPS_DIR/$CERES_DIR/$BUILD_DIR
        sudo make -j1 install
    fi
}

install_pcl()
{
  PCL_VERSION="1.11.1"
  PCL_DIR="pcl"
  BUILD_DIR="build"

  mkdir -p $DEPS_DIR
  cd $DEPS_DIR

  if [ ! -d "$PCL_DIR" ]; then
    echo "pcl not found... cloning"
    git clone git@github.com:BEAMRobotics/pcl.git
  fi

  cd $PCL_DIR
  if [ ! -d "$BUILD_DIR" ]; then
    echo "Existing build of PCL not found.. building from scratch"
    mkdir -p $BUILD_DIR
    cd $BUILD_DIR

    PCL_CMAKE_ARGS="-DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-std=c++11"
    cmake .. ${PCL_CMAKE_ARGS} > /dev/null
    make -j2
  fi

  cd $DEPS_DIR/$PCL_DIR/$BUILD_DIR
  sudo make -j2 install
}

install_catch2()
{
  echo "Installing Catch2..."
  CATCH2_DIR="Catch2"
  BUILD_DIR="build"
  mkdir -p $DEPS_DIR
  cd $DEPS_DIR

  if [ ! -d "$DEPS_DIR/$CATCH2_DIR" ]; then
    git clone https://github.com/catchorg/Catch2.git --branch v2.13.2 $DEPS_DIR/$CATCH2_DIR
  fi

  cd $CATCH2_DIR
  if [ ! -d "$BUILD_DIR" ]; then
    mkdir -p $BUILD_DIR
    cd $BUILD_DIR
    cmake -DCMAKE_CXX_STANDARD=11 ..
    make -j$(nproc)
  fi

  cd $DEPS_DIR/$CATCH2_DIR/$BUILD_DIR
  sudo make -j$(nproc) install
}

install_eigen3()
{
  echo "Installing Eigen 3.3.7..."
  EIGEN_DIR="eigen-3.3.7"
  BUILD_DIR="build"
  mkdir -p $DEPS_DIR
  cd $DEPS_DIR

  if [ ! -d "$EIGEN_DIR" ]; then
    wget https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.bz2
    tar xjf eigen-3.3.7.tar.bz2
    rm -rf eigen-3.3.7.tar.bz2
  fi

  cd $EIGEN_DIR
  if [ ! -d "$BUILD_DIR" ]; then
    mkdir -p $BUILD_DIR
    cd $BUILD_DIR
    cmake ..
    make
  fi

  cd $DEPS_DIR/$EIGEN_DIR/$BUILD_DIR
  sudo make -j1 install
}

install_gflags()
{
  GFLAGS_DIR="gflags"
  BUILD_DIR="build"
  mkdir -p $DEPS_DIR
  cd $DEPS_DIR

  if [ ! -d "$GFLAGS_DIR" ]; then
    git clone https://github.com/gflags/gflags.git
  fi

  cd $GFLAGS_DIR
  if [ ! -d "$BUILD_DIR" ]; then
    mkdir -p $BUILD_DIR
    cd $BUILD_DIR
    cmake - GFLAGS_BUILD_SHARED_LIBS - GFLAGS_INSTALL_SHARED_LIBS ..
    make 
  fi

  cd $DEPS_DIR/$GFLAGS_DIR/$BUILD_DIR
  sudo make -j$(nproc) install
}

install_opencv()
{
  BUILD_DIR="build"
  mkdir -p $DEPS_DIR
  cd $DEPS_DIR

  # Install minimal prerequisites (Ubuntu 18.04 as reference)
  sudo apt update && sudo apt install -y cmake g++ wget unzip
  
  # Download and unpack sources
  wget -O opencv.zip https://github.com/opencv/opencv/archive/master.zip
  unzip opencv.zip
  rm -rf opencv.zio
  
  # Create build directory
  mkdir -p build && cd build
  cmake  ../opencv-master
  cmake --build .
  sudo make install
}
main $1

