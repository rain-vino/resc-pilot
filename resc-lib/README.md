# Motion Generator Lib

Motion Generator Lib (mogenlib) is a part of mogen.

Currently, it can be used for RL trainning.

## Installation

**Note:** tested on Ubuntu 20.04 only

### Prerequisites

* pybind11
* eigen3

### Install Using CMake

```shell
cd /path_to_mogen/mogenlib
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
```

