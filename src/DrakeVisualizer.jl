__precompile__()

module DrakeVisualizer

using PyLCM
using GeometryTypes
import GeometryTypes: origin, radius
import PyCall: pyimport, PyObject, PyNULL
import AffineTransforms: AffineTransform, rotationparameters, tformeye
import Quaternions: qrotation, Quaternion
import ColorTypes: RGBA, Colorant, red, green, blue, alpha
import FixedSizeArrays: destructure
import Base: convert, length

export GeometryData,
        Link,
        Robot,
        Visualizer,
        draw,
        reload


const lcmdrake = PyNULL()

# GeometryTypes doesn't define an Ellipsoid type yet, so we'll make one ourselves!
type HyperEllipsoid{N, T} <: GeometryPrimitive{N, T}
    center::Point{N, T}
    radii::Vec{N, T}
end

origin{N, T}(geometry::HyperEllipsoid{N, T}) = geometry.center
radii{N, T}(geometry::HyperEllipsoid{N, T}) = geometry.radii

type HyperCylinder{N, T} <: GeometryTypes.GeometryPrimitive{N, T}
    length::T # along last axis
    radius::T
    # origin is at center
end

length(geometry::HyperCylinder) = geometry.length
radius(geometry::HyperCylinder) = geometry.radius

type GeometryData{T, GeometryType <: AbstractGeometry}
    geometry::GeometryType
    transform::AffineTransform{T, 3}
    color::RGBA{Float64}
end
GeometryData{T, GeometryType <: AbstractGeometry}(geometry::GeometryType,
    transform::AffineTransform{T, 3}=tformeye(3),
    color=RGBA{Float64}(1., 0, 0, 0.5)) = GeometryData(geometry, transform, convert(RGBA{Float64}, color))

type Link
    geometry_data::Vector{GeometryData}
    name::String
end
Link{T <: GeometryData}(geometry_data::Vector{T}) = Link(geometry_data, "link")

type Robot
    links::Vector{Link}
end

convert(::Type{Link}, geom::GeometryData) = Link([geom])
convert(::Type{Robot}, link::Link) = Robot([link])
convert(::Type{Robot}, links::AbstractArray{Link}) = Robot(convert(Vector{Link}, links))
convert(::Type{Robot}, geom::GeometryData) = convert(Robot, convert(Link, geom))
convert(::Type{Robot}, geometry::AbstractGeometry) = convert(Robot, GeometryData(geometry))

to_lcm(q::Quaternion) = Float64[q.s; q.v1; q.v2; q.v3]
to_lcm(color::Colorant) = Float64[red(color); green(color); blue(color); alpha(color)]

center(geometry::HyperRectangle) = minimum(geometry) + 0.5 * widths(geometry)
center(geometry::HyperCube) = minimum(geometry) + 0.5 * widths(geometry)
center(geometry::HyperSphere) = origin(geometry)
center(geometry::HyperEllipsoid) = origin(geometry)

function fill_geometry_data!(msg::PyObject, geometry::AbstractMesh, transform::AffineTransform)
    msg[:type] = msg[:MESH]
    msg[:position] = transform.offset
    msg[:quaternion] = to_lcm(qrotation(rotationparameters(transform.scalefwd)))
    msg[:string_data] = ""
    float_data = Float64[length(vertices(geometry));
        length(faces(geometry));
        vec(destructure(vertices(geometry)));
        vec(destructure(map(f -> convert(Face{3, Int, -1}, f), faces(geometry))))]
    msg[:float_data] = float_data
    msg[:num_float_data] = length(float_data)
end

function fill_geometry_data!(msg::PyObject, geometry::HyperRectangle, transform::AffineTransform)
    msg[:type] = msg[:BOX]
    msg[:position] = transform.offset + transform.scalefwd * convert(Vector, center(geometry))
    msg[:quaternion] = to_lcm(qrotation(rotationparameters(transform.scalefwd)))
    msg[:string_data] = ""
    float_data = convert(Vector, widths(geometry))
    msg[:float_data] = float_data
    msg[:num_float_data] = length(float_data)

end

function fill_geometry_data!(msg::PyObject, geometry::HyperCube, transform::AffineTransform)
    msg[:type] = msg[:BOX]
    msg[:position] = transform.offset + transform.scalefwd * convert(Vector, center(geometry))
    msg[:quaternion] = to_lcm(qrotation(rotationparameters(transform.scalefwd)))
    msg[:string_data] = ""
    float_data = convert(Vector, widths(geometry))
    msg[:float_data] = float_data
    msg[:num_float_data] = length(float_data)
end

function fill_geometry_data!(msg::PyObject, geometry::HyperSphere, transform::AffineTransform)
    msg[:type] = msg[:SPHERE]
    msg[:position] = transform.offset + transform.scalefwd * convert(Vector, center(geometry))
    msg[:quaternion] = to_lcm(qrotation(rotationparameters(transform.scalefwd)))
    msg[:string_data] = ""
    msg[:float_data] = [radius(geometry)]
    msg[:num_float_data] = 1
end

function fill_geometry_data!(msg::PyObject, geometry::HyperEllipsoid, transform::AffineTransform)
    msg[:type] = msg[:ELLIPSOID]
    msg[:position] = transform.offset + transform.scalefwd * convert(Vector, center(geometry))
    msg[:quaternion] = to_lcm(qrotation(rotationparameters(transform.scalefwd)))
    msg[:string_data] = ""
    msg[:float_data] = convert(Vector, radii(geometry))
    msg[:num_float_data] = 3
end

function fill_geometry_data!(msg::PyObject, geometry::HyperCylinder{3}, transform::AffineTransform)
    msg[:type] = msg[:CYLINDER]
    msg[:position] = transform.offset
    msg[:quaternion] = to_lcm(qrotation(rotationparameters(transform.scalefwd)))
    msg[:string_data] = ""
    msg[:float_data] = [radius(geometry); length(geometry)]
    msg[:num_float_data] = 2
end

function to_lcm{T, GeomType}(geometry_data::GeometryData{T, GeomType})
    msg = lcmdrake[:lcmt_viewer_geometry_data]()
    msg[:color] = to_lcm(geometry_data.color)

    fill_geometry_data!(msg, geometry_data.geometry, geometry_data.transform)
    msg
end

function to_lcm(link::Link, robot_id_number::Real)
    msg = lcmdrake[:lcmt_viewer_link_data]()
    msg[:name] = link.name
    msg[:robot_num] = robot_id_number
    msg[:num_geom] = length(link.geometry_data)
    for geometry_data in link.geometry_data
        push!(msg["geom"], to_lcm(geometry_data))
    end
    msg
end

function to_lcm(robot::Robot, robot_id_number::Real)
    msg = lcmdrake[:lcmt_viewer_load_robot]()
    msg[:num_links] = length(robot.links)
    for link in robot.links
        push!(msg["link"], to_lcm(link, robot_id_number))
    end
    msg
end

type Visualizer
    robot::Robot
    robot_id_number::Int
    lcm::LCM

    function Visualizer(robot::Robot, robot_id_number::Integer=1, lcm::LCM=LCM())
        vis = new(robot, robot_id_number, lcm)
        reload(vis)
        vis
    end

end

Visualizer(data, robot_id_number::Integer=1, lcm::LCM=LCM()) = Visualizer(convert(Robot, data), robot_id_number, lcm)

function reload(vis::Visualizer)
    msg = to_lcm(vis.robot, vis.robot_id_number)
    publish(vis.lcm, "DRAKE_VIEWER_LOAD_ROBOT", msg)
end

function launch()
    (stream, proc) = open(`drake-visualizer`)
    proc
end

function draw{T <: AffineTransform}(model::Visualizer, link_origins::Vector{T})
    msg = lcmdrake[:lcmt_viewer_draw]()
    msg[:timestamp] = convert(Int64, time_ns())
    msg[:num_links] = length(link_origins)
    for (i, origin) in enumerate(link_origins)
        push!(msg["link_name"], model.robot.links[i].name)
        push!(msg["robot_num"], model.robot_id_number)
        push!(msg["position"], origin.offset)
        push!(msg["quaternion"], to_lcm(qrotation(rotationparameters(origin.scalefwd))))
    end
    publish(model.lcm, "DRAKE_VIEWER_DRAW", msg)
end

function __init__()
    copy!(lcmdrake, pyimport("drake"))
end

end
