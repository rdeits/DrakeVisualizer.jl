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
                                  AbstractAffineMap
import ColorTypes: RGBA, Colorant, red, green, blue, alpha
import StaticArrays: SVector
import Base: convert, length, show
import DataStructures: OrderedDict

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
        draw,
        reload!,
        clear

const drakevis = PyNULL()
const drake_visualizer_executable_name = "drake-visualizer"

destructure{T,N}(A::Array{T,N}) = reinterpret(eltype(T), A, (size(T)..., size(A)...))

type GeometryData{TransformType <: Transformation, GeometryType <: AbstractGeometry}
    geometry::GeometryType
    transform::TransformType
    color::RGBA{Float64}
end

GeometryData{TransformType <: Transformation, GeometryType <: AbstractGeometry}(geometry::GeometryType,
    transform::TransformType=IdentityTransformation(),
    color=RGBA{Float64}(1., 0, 0, 0.5)) = GeometryData{TransformType, GeometryType}(geometry, transform, convert(RGBA{Float64}, color))

typealias Link Vector{GeometryData}
typealias LinkDict{KeyType} OrderedDict{KeyType, Link}

include("contour_meshes.jl")
include("geometry_types.jl")
include("lcm_conversions.jl")

convert(::Type{Link}, geom::GeometryData) = Link([geom])
convert(::Type{Link}, geometry::AbstractGeometry) = Link(GeometryData(geometry))
convert{D <: LinkDict}(::Type{D}, link::Link) = OrderedDict(1 => link)
convert{K}(::Type{LinkDict}, links::Associative{K}) = convert(OrderedDict{K, Link}, links)
convert{D <: LinkDict}(::Type{D}, links::AbstractArray{Link}) = OrderedDict{Int, Link}(zip(1:length(links), links))
convert{D <: LinkDict}(::Type{D}, geom::GeometryData) = convert(LinkDict, convert(Link, geom))
convert{D <: LinkDict}(::Type{D}, geometry::AbstractGeometry) = convert(LinkDict, GeometryData(geometry))


type Visualizer{KeyType}
    links::LinkDict{KeyType}
    robot_id_number::Int
    lcm::LCM

    function Visualizer(links::LinkDict{KeyType},
                        robot_id_number::Integer=1,
                        lcm::LCM=LCM();
                        load_now::Bool=true)
        vis = new(links, robot_id_number, lcm)
        if load_now
            reload!(vis)
        end
        vis
    end
end

Visualizer{KeyType}(links::LinkDict{KeyType}, robot_id_number::Integer, lcm::LCM) =
    Visualizer{KeyType}(links, robot_id_number, lcm)

Visualizer(data, robot_id_number::Integer=1, lcm::LCM=LCM()) =
    Visualizer(convert(LinkDict, data), robot_id_number, lcm)

show(io::IO, vis::Visualizer) =
    print(io, "Visualizer with robot_id_number: $(vis.robot_id_number)")

function reload!(vis::Visualizer)
    msg = drakevis[:lcmt_viewer_load_robot]()
    msg[:num_links] = length(vis.links)
    for (key, link) in vis.links
        push!(msg["link"], to_lcm(link, to_link_name(key), vis.robot_id_number))
    end
    publish(vis.lcm, "DRAKE_VIEWER_ADD_ROBOT", msg)
end

function clear(lcm::LCM=LCM())
    msg = drakevis[:lcmt_viewer_load_robot]()
    msg[:num_links] = 0
    publish(lcm, "DRAKE_VIEWER_LOAD_ROBOT", msg)
end

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

function draw(vis::Visualizer, link_transforms::AbstractVector)
    @assert length(link_transforms) == length(vis.links)
    draw(vis, Dict(zip(keys(vis.links), link_transforms)))
end

function draw(vis::Visualizer, link_transforms::Associative)
    msg = drakevis[:lcmt_viewer_draw]()
    msg[:timestamp] = convert(Int64, time_ns())
    msg[:num_links] = length(link_transforms)
    for (key, origin) in link_transforms
        @assert key in keys(vis.links)
        push!(msg["link_name"], to_link_name(key))
        push!(msg["robot_num"], vis.robot_id_number)
        push!(msg["position"], convert(SVector{3, Float64}, origin(SVector{3, Float64}(0,0,0))))
        push!(msg["quaternion"], to_lcm_quaternion(origin))
    end
    publish(vis.lcm, "DRAKE_VIEWER_DRAW", msg)
end

function __init__()
    lcmtypes_path = abspath(joinpath(dirname(@__FILE__), "lcmtypes"))
    println("adding: $(lcmtypes_path) to the python path")
    unshift!(PyVector(pyimport("sys")["path"]), lcmtypes_path)
    copy!(drakevis, pyimport("drakevis"))
end

end
