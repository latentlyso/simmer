
Simmer
======

Simmer is a C++ library that serves as a simulation engine for the development of models of evacuation dynamics.

Building Simmer
===============

Simmer is built using [CMake](https://www.cmake.org/). The following builds Simmer, as well as an exmaple application that demostrates its typical use pattern in client code. It then runs the app on an instance of input geometry.

	$ cd path/to/uncompressed
	$ mkdir build && cd build
	$ cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=g++ ../src
	$ make -j4
	$ simmer/bin/simmerApp -g ../demo/geom.xml -o ../demo/otpt.xml -p ../demo/plot.svg

Sufficient compiler support for C++20 is required; here, a recent version of `GCC` is assumed. Only the `g` and `o` flags are required in the last line. To only build Simmer, replace `../src` with `../src/simmer` in the third line.

