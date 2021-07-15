# cad_image_markup

## Install Instructions:

### Clone reposityory

```
cd root_directory
git clone https://github.com/nickcharron/cad_image_markup.git
```

### Install dependencies

The following software needs to be install before building

* Eigen (min. 3.3.7)
* gflags
* Catch2 (min. 2.13.2)
* PCL (min. 1.11.0)
* Ceres (min. 1.14)
* OpenCV (min. 3.2.0)

To install dependencies, we have made a script called install_deps.bash. You can call the script using:


```
cd root_directory/cad_image_markup/scripts
sudo bash install_deps.sh
```

### Build

```
cd root_directory/cad_image_markup
mkdir build
cd build
cmake ..
make -j8
```

## Running program

There is one main executable to run:

```
cd root_directory/cad_image_markup/build
./cad_image_markup_main [args]
```

The argument descriptions can be displayed using:

```
./cad_image_markup_main --help
```


