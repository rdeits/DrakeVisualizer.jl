__precompile__()

module DrakeVisualizer

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
import StaticArrays: SVector, StaticArray, SMatrix, StaticVector
import Base: convert, length, show, isempty, empty!, delete!
import DataStructures: OrderedDict
import ZMQ
import MsgPack

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

function new_window(args...; kw...)
    error("The function new_window() has been removed in the process of simplifying the connection between the visualizer and its window. Calling `Visualizer()` will now automatically launch a window which will respond only to that visualizer's commands.")
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

struct Window
    url::String
    proc::Base.Process
end

const DEFAULT_PORT = 53730
const NUM_PORTS_TO_TRY = 256

function find_available_port(host)
    port = DEFAULT_PORT
    for i in 1:NUM_PORTS_TO_TRY
        try
            socket = connect(host, port)
            close(socket)
            port += 1
        catch e
            if e isa Base.UVError && e.prefix == "connect" && e.code == -111
                return port
            end
        end
    end
    error("Could not find an available port from $DEFAULT_PORT to $(DEFAULT_PORT + 255)")
end

function Window(;url=nothing, script=nothing)
    # installed_visualizer_path = joinpath(dirname(@__FILE__), "..", "deps", "usr", "bin", "$drake_visualizer_executable_name")
    installed_visualizer_path = "/home/rdeits/locomotion/director/build/install/bin/drake-visualizer"
    drake_visualizer = if isfile(installed_visualizer_path)
        # If we built drake-visualizer, then use it
        installed_visualizer_path
    else
        # Otherwise let the system try to find it
        drake_visualizer_executable_name
    end
    if script === nothing
        command = `$drake_visualizer`
    else
        command = `$drake_visualizer --script $script`
    end

    if url === nothing
        host = "127.0.0.1"
        port = find_available_port(host)
        url = "tcp://$host:$port"
    end

    command = `$command --treeviewer-url=$url`
    (stream, proc) = open(command)
    Window(url, proc)
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

include("lazytree.jl")
include("contour_meshes.jl")
include("geometry_types.jl")
include("visualizer.jl")
include("serialization.jl")

end
