# Recursive Bayesian Estimation

Header only C++ Baysian filtering library. Currently includes standard
Kalman Filter and Extended Kalman Filter implementations.

## Submodules

This repository uses git submodules; to make sure these are cloned
along with the main code, use:

    git clone --recursive https://github.com/sgvandijk/rbest.git

If you already cloned this repository you can pull in the submodules
with:

    git submodule init && git submodule update

## Dependencies

* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) - Fast
  linear algebra
* [SDL 2](https://www.libsdl.org/) - Cross-platform input and graphics
  library
* [SDL2_gfx](http://www.ferzkopp.net/wordpress/2016/01/02/sdl_gfx-sdl2_gfx/) -
  SDL2 support functions

All these are available in the Ubuntu repositories

## Building

    mkdir build
    cd build
    cmake ..
    make

## Examples

This library comes with a few graphical examples to show the usage and
performance of the filters. After building you can run these from
`build/examples`:

    cd examples
    ./flyer
    ./tank

Try controlling the agent using the arrow keys, press `p` to pause and
`q` to quit.
