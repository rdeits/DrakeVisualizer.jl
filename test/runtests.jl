using DrakeVisualizer
using GeometryTypes
using CoordinateTransformations
using Meshing
using Base.Test
using IJulia
import Iterators: product
using Polyhedra
using CDDLib

include("lazytree.jl")
include("comms_t.jl")
include("visualizer.jl")
include("polyhedra.jl")
