__precompile__()

module DrakeVisualizer

using LCMCore
using GeometryTypes
using FileIO
import GeometryTypes: origin, radius, raw
import Meshing
import MeshIO
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
        MeshFile,
        PointCloud,
        Point,
        Vec,
        Triad,
        PolyLine,
        ArrowHead,
        contour_mesh,
        settransform!,
        setgeometry!,
        addgeometry!,
        load!,
        draw!,
        delete!,
        batch

const drake_visualizer_executable_name = "drake-visualizer"

function new_window(; script::Union{AbstractString, Void} = nothing)
    installed_visualizer_path = joinpath(dirname(@__FILE__), "..", "deps", "usr", "bin", "$drake_visualizer_executable_name")
    drake_visualizer = if isfile(installed_visualizer_path)
        # If we built drake-visualizer, then use it
        installed_visualizer_path
    else
        # Otherwise let the system try to find it
        drake_visualizer_executable_name
    end
    command = script == nothing ? `$drake_visualizer` : `$drake_visualizer --script $script`
    (stream, proc) = open(command)
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

function delete_director_binaries(skip_confirmation=false)
    root = joinpath(dirname(dirname(@__FILE__)), "deps")
    binary_paths = [
        joinpath(root, "usr"),
        joinpath(root, "downloads"),
        joinpath(root, "src"),
        joinpath(root, "builds"),
        joinpath(root, "deps.jl")
    ]
    if !skip_confirmation
        println("CAUTION: This will delete all downloaded binaries of Director.")
        println("The following paths will be deleted:")
        for path in binary_paths
            println(path)
        end
        print("After doing this, you will need to run 'Pkg.build(\"DrakeVisualizer\")'. Proceed? (y/n) ")
        choice = chomp(readline())
        if lowercase(choice[1]) != 'y'
            println("Canceled.")
            return
        end
    end
    for path in binary_paths
        println("Removing $path")
        rm(path, force=true, recursive=true)
    end
end

include("lcmtypes/comms_t.jl")
include("lazytree.jl")
include("contour_meshes.jl")
include("geometry_types.jl")
include("visualizer.jl")
include("serialization.jl")

@deprecate load!(args...) setgeometry!(args...)
@deprecate draw!(args...) settransform!(args...)

end
