# DrakeVisualizer

[![Build Status](https://travis-ci.org/rdeits/DrakeVisualizer.jl.svg?branch=master)](https://travis-ci.org/rdeits/DrakeVisualizer.jl)
[![codecov](https://codecov.io/gh/rdeits/DrakeVisualizer.jl/branch/master/graph/badge.svg)](https://codecov.io/gh/rdeits/DrakeVisualizer.jl)

This package provides a Julia interface to the Drake Visualizer, part of the [Drake](http://drake.mit.edu) project and built on top of [Director](https://github.com/RobotLocomotion/director), a highly customizable 3D interface for robotics visualization and interaction.

# Installation

On Linux and macOS, this package will attempt to download the pre-built binaries of `director` from <http://people.csail.mit.edu/patmarion/software/director/>. If this fails, you'll need to install those binaries yourself, or compile `director` from source.

On Ubuntu, the precompiled binaries of `director` require the following packages to be installed via `apt-get`:

    libvtk5-qt4-dev
    python-vtk
    python-numpy

On macOS, the binaries should already include all dependencies.

# Troubleshooting

If you have issues with the Director application itself (like your geometry not showing up), you may have an out-of-date version of the Director binaries. To clear the downloaded binaries, you can run:

    julia> DrakeVisualizer.delete_director_binaries()

After which you will need to re-download the binaries with:

    julia> Pkg.build("DrakeVisualizer")

## Linux: Configuring Large UDP Packets

`DrakeVisualizer` uses [LCM](http://lcm-proj.github.io/) for communication, and LCM uses UDP under the hood. Very large LCM messages (like those created when loading a robot with lots of mesh geometries) can result in UDP packets being dropped, which will result in you not seeing anything in the visualizer. If that happens to you, you'll need to follow the instructions in [this comment](https://github.com/rdeits/DrakeVisualizer.jl/issues/19#issuecomment-267429751). Edit `/etc/sysctl.conf` and add:

    net.core.rmem_max=2097152
    net.core.rmem_default=2097152

# Launching the Viewer

You can launch the viewer application with

    julia> DrakeVisualizer.new_window()

which is just a convenience wrapper around a call to the `drake-visualizer` executable, included in the `director` binaries or source installation.

# Usage:

This package makes use of [GeometryTypes.jl](https://github.com/JuliaGeometry/GeometryTypes.jl) to represent robot geometries and [CoordinateTransformations.jl](https://github.com/FugroRoames/CoordinateTransformations.jl) to represent spatial transformations. Check out [demo.ipynb](https://github.com/rdeits/DrakeVisualizer.jl/blob/master/demo.ipynb) for some examples of usage.

## Geometric Primitives

Geometric primitives from GeometryTypes.jl can be visualized directly:

```julia
using DrakeVisualizer
using GeometryTypes
box = HyperRectangle(Vec(0.,0,0), Vec(1.,1,1))
model = Visualizer(box)
```

![box](https://cloud.githubusercontent.com/assets/591886/19826370/3efea352-9d56-11e6-9d6b-695035c5baae.png)

```julia
sphere = HyperSphere(Point(0., 0, 0), 1.0)
model = Visualizer(sphere)
```

![sphere](https://cloud.githubusercontent.com/assets/591886/19826371/414ebec6-9d56-11e6-99e3-73a3bad190b9.png)

Once a Visualizer model has been created, it can be rendered at arbitrary positions and orientations:

```julia
using CoordinateTransformations
draw(model, [Translation(1.,0,0)])
```

DrakeVisualizer can also render mesh data:

```julia
using MeshIO
using FileIO
cat = load(joinpath(Pkg.dir("GeometryTypes"), "test", "data", "cat.obj"))
Visualizer(cat)
```

![cat mesh](https://cloud.githubusercontent.com/assets/591886/19826425/faebbb9e-9d57-11e6-852f-71c91f9ff757.png)

And it can even generate 3D contours from functions:

```julia
# First, we'll define our function:
f = x -> sum(sin(5 * x))

# Then we pick a region of interest in which to sample the function.
# This region starts at (-1, -1, -1) and extends to (1, 1, 1):
lower_bound = Vec(-1.,-1,-1)
upper_bound = Vec(1., 1, 1)

# contour_mesh constructs a mesh representing an approximation of
# the surface in 3D space for which f(x) = 0
mesh = contour_mesh(f, lower_bound, upper_bound)
Visualizer(mesh)
```

![custom function contour mesh](https://cloud.githubusercontent.com/assets/591886/19826595/a1e09484-9d5c-11e6-9268-314059767224.png)


For more visualizations, including moving and rotating visualized elements, and visualizing the level sets of functions, check out [demo.ipynb](https://github.com/rdeits/DrakeVisualizer.jl/blob/master/demo.ipynb).
