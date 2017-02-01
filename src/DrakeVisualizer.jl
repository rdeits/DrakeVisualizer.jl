__precompile__()

module DrakeVisualizer

using LCMCore
using GeometryTypes
import GeometryTypes: origin, radius
import Meshing
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
        PointCloud,
        contour_mesh,
        load!,
        draw!,
        delete!,
        publish!,
        batch,
        draw

const drake_visualizer_executable_name = "drake-visualizer"

function new_window()
    installed_visualizer_path = joinpath(dirname(@__FILE__), "..", "deps", "usr", "bin", "$drake_visualizer_executable_name")
    if isfile(installed_visualizer_path)
        # If we built drake-visualizer, then use it
        (stream, proc) = open(`$installed_visualizer_path`)
    else
        # Otherwise let the system try to find it
        (stream, proc) = open(`$drake_visualizer_executable_name`)
    end
    proc
end

function any_open_windows()
    @static if is_apple()
        return success(spawn(`pgrep $drake_visualizer_executable_name`))
    elseif is_linux()
        return success(spawn(`pgrep -f $drake_visualizer_executable_name`))
    else
        warn("DrakeVisualizer.any_open_windows not implemented for $(Sys.KERNEL). This function will always return false.")
        return false
    end
end

include("lcmtypes/comms_t.jl")
include("lazytree.jl")
include("contour_meshes.jl")
include("geometry_types.jl")
include("visualizer.jl")
include("serialization.jl")

end
