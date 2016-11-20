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
import Base: convert, length
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
        reload_model

const drakevis = PyNULL()

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
typealias Robot{KeyType} OrderedDict{KeyType, Link}

include("contour_meshes.jl")
include("geometry_types.jl")
include("lcm_conversions.jl")

convert(::Type{Link}, geom::GeometryData) = Link([geom])
convert(::Type{Link}, geometry::AbstractGeometry) = Link(GeometryData(geometry))
convert(::Type{Robot}, link::Link) = OrderedDict(1 => link)
convert{K}(::Type{Robot}, links::Associative{K}) = convert(OrderedDict{K, Link}, links)
convert(::Type{Robot}, links::AbstractArray{Link}) = OrderedDict{Int, Link}(zip(1:length(links), links))
convert(::Type{Robot}, geom::GeometryData) = convert(Robot, convert(Link, geom))
convert(::Type{Robot}, geometry::AbstractGeometry) = convert(Robot, GeometryData(geometry))


type Visualizer{KeyType}
    links::Robot{KeyType}
    robot_id_number::Int
    lcm::LCM

    function Visualizer(links::Robot{KeyType},
                        robot_id_number::Integer=1,
                        lcm::LCM=LCM();
                        load_now::Bool=true)
        vis = new(links, robot_id_number, lcm)
        if load_now
            reload_model(vis)
        end
        vis
    end
end

Visualizer{KeyType}(links::Robot{KeyType}, robot_id_number::Integer, lcm::LCM) =
    Visualizer{KeyType}(links, robot_id_number, lcm)

Visualizer(data, robot_id_number::Integer=1, lcm::LCM=LCM()) =
    Visualizer(convert(Robot, data), robot_id_number, lcm)

function Visualizer{KeyType}(robots::AbstractVector{Robot{KeyType}}, lcm::LCM=LCM())
    visualizers = [Visualizer{KeyType}(robot, i, lcm; load_now=false)
                   for (i, robot) in enumerate(robots)]
    reload_model(visualizers)
    visualizers
end

function reload_model(vis::Visualizer)
    reload_model(SVector(vis))
end

to_link_name(key) = string(key)

function reload_model{V <: Visualizer}(visualizers::AbstractArray{V})
    msg = drakevis[:lcmt_viewer_load_robot]()
    msg[:num_links] = 0
    for vis in visualizers
        msg[:num_links] = msg[:num_links] + length(vis.links)
        for (key, link) in vis.links
            push!(msg["link"], to_lcm(link, to_link_name(key), vis.robot_id_number))
        end
    end
    publish(visualizers[1].lcm, "DRAKE_VIEWER_LOAD_ROBOT", msg)
end

function new_window()
    installed_visualizer_path = joinpath(dirname(@__FILE__), "..", "deps", "usr", "bin", "drake-visualizer")
    if isfile(installed_visualizer_path)
        # If we built drake-visualizer, then use it
        (stream, proc) = open(`$installed_visualizer_path`)
    else
        # Otherwise let the system try to find it
        (stream, proc) = open(`drake-visualizer`)
    end
    proc
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
