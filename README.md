# DrakeVisualizer

This package provides a Julia interface to the Drake Visualizer, part of the [Drake](http://drake.mit.edu) project and built on top of [Director](https://github.com/RobotLocomotion/director), a highly customizable 3D interface for robotics visualization and interaction.

# Installation

You'll need to install the `drake-visualizer` application separately. If you're using Drake already, that's just a matter of making sure you have the `WITH_DIRECTOR` option turned on. If you're not using Drake, you can follow the `superbuild` instructions from [RobotLocomotion/Director](https://github.com/RobotLocomotion/director) to build Director and `drake-visualizer`.

# Usage:

This package makes use of [GeometryTypes.jl](https://github.com/JuliaGeometry/GeometryTypes.jl) to represent robot geometries and [CoordinateTransformations.jl](https://github.com/FugroRoames/CoordinateTransformations.jl) to represent spatial transformations. Check out [demo.ipynb](https://github.com/rdeits/DrakeVisualizer.jl/blob/master/demo.ipynb) for some examples of usage.
