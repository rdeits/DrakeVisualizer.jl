# DrakeVisualizer

[![Build Status](https://travis-ci.org/rdeits/DrakeVisualizer.jl.svg?branch=master)](https://travis-ci.org/rdeits/DrakeVisualizer.jl)

This package provides a Julia interface to the Drake Visualizer, part of the [Drake](http://drake.mit.edu) project and built on top of [Director](https://github.com/RobotLocomotion/director), a highly customizable 3D interface for robotics visualization and interaction.

# Installation

On Linux and macOS, this package will attempt to download the pre-built binaries of `director` from <http://people.csail.mit.edu/patmarion/software/director/>. If this fails, you'll need to install those binaries yourself, or compile `director` from source.

On Ubuntu, the precompiled binaries of `director` require the following packages to be installed via `apt-get`:

    libvtk5-qt4-dev
    python-vtk

On macOS, the binaries should already include all dependencies.

# Launching the Viewer

You can launch the viewer application with

    julia> DrakeVisualizer.new_window()

which is just a convenience wrapper around a call to the `drake-visualizer` executable, included in the `director` binaries or source installation.

# Usage:

This package makes use of [GeometryTypes.jl](https://github.com/JuliaGeometry/GeometryTypes.jl) to represent robot geometries and [CoordinateTransformations.jl](https://github.com/FugroRoames/CoordinateTransformations.jl) to represent spatial transformations. Check out [demo.ipynb](https://github.com/rdeits/DrakeVisualizer.jl/blob/master/demo.ipynb) for some examples of usage.
