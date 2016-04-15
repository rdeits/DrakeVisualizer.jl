module DrakeVisualizer

using PyLCM
using GeometryTypes
import PyCall: pyimport, PyObject
import AffineTransforms: AffineTransform, rotationparameters, tformeye
import Quaternions: qrotation, Quaternion
import ColorTypes: RGBA, Colorant, red, green, blue, alpha
import FixedSizeArrays: destructure
import Base: convert

export GeometryData,
        Link,
        Robot,
        Visualizer,
        load,
        draw

type GeometryData{T, GeometryType <: AbstractGeometry}
    geometry::GeometryType
    transform::AffineTransform{T, 3}
    color::RGBA{Float64}
end
GeometryData{T, GeometryType <: AbstractGeometry}(geometry::GeometryType,
    transform::AffineTransform{T, 3},
    color::RGBA{Float64}) = GeometryData{T, GeometryType}(geometry, transform, color)
GeometryData{T, GeometryType <: AbstractGeometry}(geometry::GeometryType,
    transform::AffineTransform{T, 3}=tformeye(3),
    color=RGBA{Float64}(1., 0, 0, 0.5)) = GeometryData(geometry, transform, convert(RGBA{Float64}, color))

type Link
    geometry_data::Vector{GeometryData}
    name::ASCIIString
end
Link{T <: GeometryData}(geometry_data::Vector{T}) = Link(geometry_data, "link")

type Robot
    links::Vector{Link}
end

convert(::Type{Link}, geom::GeometryData) = Link([geom])
convert(::Type{Robot}, link::Link) = Robot([link])
convert(::Type{Robot}, links::Vector{Link}) = Robot(links)
convert(::Type{Robot}, geom::GeometryData) = convert(Robot, convert(Link, geom))
convert(::Type{Robot}, geometry::AbstractGeometry) = convert(Robot, GeometryData(geometry))

to_lcm(q::Quaternion) = Float64[q.s; q.v1; q.v2; q.v3]
to_lcm(color::Colorant) = Float64[red(color); green(color); blue(color); alpha(color)]

center(geometry::HyperRectangle) = minimum(geometry) + 0.5 * widths(geometry)
center(geometry::HyperCube) = minimum(geometry) + 0.5 * widths(geometry)
center(geometry::HyperSphere) = origin(geometry)

function fill_geometry_data!(msg::PyObject, geometry::AbstractMesh, transform::AffineTransform)
    msg[:type] = msg[:MESH]
    msg[:position] = transform.offset
    msg[:quaternion] = to_lcm(qrotation(rotationparameters(transform.scalefwd)))
    msg[:string_data] = ""
    msg[:float_data] = Float64[length(vertices(geometry));
        length(faces(geometry));
        vec(destructure(vertices(geometry)));
        vec(destructure(map(f -> convert(Face{3, Int, -1}, f), faces(geometry))))]
end

function fill_geometry_data!(msg::PyObject, geometry::HyperRectangle, transform::AffineTransform)
    msg[:type] = msg[:BOX]
    msg[:position] = transform.offset + convert(Vector, center(geometry))
    msg[:quaternion] = to_lcm(qrotation(rotationparameters(transform.scalefwd)))
    msg[:string_data] = ""
    msg[:float_data] = convert(Vector, widths(geometry))
end

function fill_geometry_data!(msg::PyObject, geometry::HyperCube, transform::AffineTransform)
    msg[:type] = msg[:BOX]
    msg[:position] = transform.offset + convert(Vector, center(geometry))
    msg[:quaternion] = to_lcm_data(qrotation(rotationparameters(transform.scalefwd)))
    msg[:string_data] = ""
    msg[:float_data] = convert(Vector, widths(geometry))
end

function fill_geometry_data!(msg::PyObject, geometry::HyperSphere, transform::AffineTransform)
    msg[:type] = msg[:SPHERE]
    msg[:position] = transform.offset + convert(Vector, center(geometry))
    msg[:quaternion] = to_lcm_data(qrotation(rotationparameters(transform.scalefwd)))
    msg[:string_data] = ""
    msg[:float_data] = [radius(geometry)]
end

function to_lcm{T, GeomType}(geometry_data::GeometryData{T, GeomType})
    msg = lcmdrake[:lcmt_viewer_geometry_data]()
    msg[:color] = to_lcm(geometry_data.color)

    fill_geometry_data!(msg, geometry_data.geometry, geometry_data.transform)
    msg[:num_float_data] = length(msg[:float_data])
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
    lcm::LCM

    Visualizer(lcm::LCM=LCM()) = new(lcm)
end

type VisualizerModel
    robot::Robot
    vis::Visualizer
    robot_id_number::Int
end

function load(vis::Visualizer, robot::Robot, robot_id_number=1)
    msg = to_lcm(robot, robot_id_number)
    publish(vis.lcm, "DRAKE_VIEWER_LOAD_ROBOT", msg)
    return VisualizerModel(robot, vis, robot_id_number)
end

load(vis::Visualizer, robot, robot_id_number=1) = load(vis, convert(Robot, robot), robot_id_number)

load(robot) = load(Visualizer(), robot)

function draw{T <: AffineTransform}(model::VisualizerModel, link_origins::Vector{T})
    msg = lcmdrake[:lcmt_viewer_draw]()
    msg[:timestamp] = convert(Int64, time_ns())
    msg[:num_links] = length(link_origins)
    for (i, origin) in enumerate(link_origins)
        push!(msg["link_name"], model.robot.links[i].name)
        push!(msg["robot_num"], model.robot_id_number)
        push!(msg["position"], origin.offset)
        push!(msg["quaternion"], to_lcm(qrotation(rotationparameters(origin.scalefwd))))
    end
    publish(model.vis.lcm, "DRAKE_VIEWER_DRAW", msg)
end

function __init__()
    const global lcmdrake = pyimport("drake")
end

end
