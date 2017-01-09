__precompile__()

module DrakeVisualizer

using PyLCM
using GeometryTypes
import GeometryTypes: origin, radius
import Meshing
import PyCall: pyimport, PyObject, PyNULL, PyVector
import Rotations: Rotation, Quat
import CoordinateTransformations: Transformation,
                                  transform_deriv,
                                  IdentityTransformation,
                                  AbstractAffineMap,
                                  AffineMap,
                                  Translation,
                                  compose
import ColorTypes: RGB, RGBA, Colorant, red, green, blue, alpha
import StaticArrays: SVector, StaticArray, SMatrix
import Base: convert, length, show, isempty, empty!, delete!
import DataStructures: OrderedDict
import JSON

export GeometryData,
        Link,
        Robot,
        Visualizer,
        HyperRectangle,
        HyperEllipsoid,
        HyperCylinder,
        HyperSphere,
        HyperCube,
        contour_mesh,
        load!,
        draw!,
        delete!,
        publish!

const drakevis = PyNULL()
const drake_visualizer_executable_name = "drake-visualizer"

include("lazytree.jl")
include("contour_meshes.jl")
include("geometry_types.jl")
include("visualizer.jl")
include("serialization.jl")

function __init__()
    lcmtypes_path = abspath(joinpath(dirname(@__FILE__), "lcmtypes"))
    println("adding: $(lcmtypes_path) to the python path")
    unshift!(PyVector(pyimport("sys")["path"]), lcmtypes_path)
    copy!(drakevis, pyimport("drakevis"))
end

end
