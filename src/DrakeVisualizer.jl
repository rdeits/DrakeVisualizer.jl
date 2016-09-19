__precompile__()

module DrakeVisualizer

using PyLCM
using GeometryTypes
import GeometryTypes: origin, radius
import Meshing
import PyCall: pyimport, PyObject, PyNULL
import Rotations: Rotation, Quat
import CoordinateTransformations: Transformation, transform_deriv, IdentityTransformation, AbstractAffineMap
import ColorTypes: RGBA, Colorant, red, green, blue, alpha
import StaticArrays: SVector
import Base: convert, length

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


const lcmdrake = PyNULL()

destructure{T,N}(A::Array{T,N}) = reinterpret(eltype(T), A, (size(T)..., size(A)...))

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

type GeometryData{TransformType <: Transformation, GeometryType <: AbstractGeometry}
    geometry::GeometryType
    transform::TransformType
    color::RGBA{Float64}
end

GeometryData{TransformType <: Transformation, GeometryType <: AbstractGeometry}(geometry::GeometryType,
    transform::TransformType=IdentityTransformation(),
    color=RGBA{Float64}(1., 0, 0, 0.5)) = GeometryData{TransformType, GeometryType}(geometry, transform, convert(RGBA{Float64}, color))

function contour_mesh(f::Function,
                      bounds::HyperRectangle,
                      isosurface_value::Number=0.0,
                      resolution::Real=maximum(widths(bounds))/20.0)
    # Extract an isosurface from a vector-input function in 3 dimensions and
    # return it as a mesh. This function mostly exists to work around bugs in
    # Meshing.jl: specifically its weird handling of non-zero isosurfaces
    # and its incorrect mesh point scaling:
    # https://github.com/JuliaGeometry/GeometryTypes.jl/issues/49
    window_width = widths(bounds)
    sdf = SignedDistanceField(x -> f(SVector{3, Float64}(x)) - isosurface_value,
                              bounds,
                              resolution)

    # We've already accounted for the isosurface value in the construction of
    # the SDF, so we set the iso value here to 0.
    mesh = HomogenousMesh(sdf, 0.0)

    # Rescale the mesh points and then construct a new mesh using the rescaled
    # points and the original faces.
    lb = minimum(bounds)
    rescaled_points = Point{3,Float64}[Vec(v-1) ./ (Vec(size(sdf))-1) .* window_width .+ lb for v in vertices(mesh)]
    HomogenousMesh(rescaled_points, mesh.faces)
end

contour_mesh(f::Function,
             lower_bound::Vec,
             upper_bound::Vec,
             isosurface_value::Number=0.0,
             resolution::Real=maximum(upper_bound - lower_bound)/20.0) = (
    contour_mesh(f,
                 HyperRectangle(lower_bound, upper_bound - lower_bound),
                 isosurface_value,
                 resolution))

contour_mesh(f::Function,
             lower_bound::AbstractVector,
             upper_bound::AbstractVector,
             isosurface_value::Number=0.0,
             resolution::Real=maximum(upper_bound - lower_bound)/20.0) = (
    contour_mesh(f,
                 Vec{3, Float64}(lower_bound[1], lower_bound[2], lower_bound[3]),
                 Vec{3, Float64}(upper_bound[1], upper_bound[2], upper_bound[3]),
                 isosurface_value,
                 resolution))

type Link
    geometry_data::Vector{GeometryData}
    name::String
end
Link{T <: GeometryData}(geometry_data::Vector{T}) = Link(geometry_data, "link")

type Robot
    links::Vector{Link}
end

convert(::Type{Link}, geom::GeometryData) = Link([geom])
convert(::Type{Link}, geometry::AbstractGeometry) = Link(GeometryData(geometry))
convert(::Type{Robot}, link::Link) = Robot([link])
convert(::Type{Robot}, links::AbstractArray{Link}) = Robot(convert(Vector{Link}, links))
convert(::Type{Robot}, geom::GeometryData) = convert(Robot, convert(Link, geom))
convert(::Type{Robot}, geometry::AbstractGeometry) = convert(Robot, GeometryData(geometry))

to_lcm(q::Quat) = SVector{4, Float64}(q.w, q.x, q.y, q.z)
to_lcm_quaternion(matrix::AbstractMatrix) = to_lcm(Quat(matrix))
to_lcm_quaternion(matrix::UniformScaling) = to_lcm(Quat(1.0, 0, 0, 0))
to_lcm_quaternion(transform::AbstractAffineMap) = to_lcm_quaternion(transform_deriv(transform, SVector{3, Float64}(0,0,0)))
to_lcm_quaternion(transform::IdentityTransformation) = to_lcm_quaternion(Quat(1.0, 0, 0, 0))

to_lcm(color::Colorant) = Float64[red(color); green(color); blue(color); alpha(color)]

center(geometry::HyperRectangle) = minimum(geometry) + 0.5 * widths(geometry)
center(geometry::HyperCube) = minimum(geometry) + 0.5 * widths(geometry)
center(geometry::HyperSphere) = origin(geometry)
center(geometry::HyperEllipsoid) = origin(geometry)

function fill_geometry_data!(msg::PyObject, geometry::AbstractMesh, transform::Transformation)
    msg[:type] = msg[:MESH]
    msg[:position] = transform(SVector{3, Float64}(0, 0, 0))
    msg[:quaternion] = to_lcm_quaternion(transform)
    msg[:string_data] = ""
    float_data = Float64[length(vertices(geometry));
        length(faces(geometry));
        vec(destructure(vertices(geometry)));
        vec(destructure(map(f -> convert(Face{3, Int, -1}, f), faces(geometry))))]
    msg[:float_data] = float_data
    msg[:num_float_data] = length(float_data)
end

function fill_geometry_data!(msg::PyObject, geometry::HyperRectangle, transform::Transformation)
    msg[:type] = msg[:BOX]
    msg[:position] = convert(SVector{3, Float64}, transform(center(geometry)))
    msg[:quaternion] = to_lcm_quaternion(transform)
    msg[:string_data] = ""
    float_data = convert(Vector, widths(geometry))
    msg[:float_data] = float_data
    msg[:num_float_data] = length(float_data)

end

function fill_geometry_data!(msg::PyObject, geometry::HyperCube, transform::Transformation)
    msg[:type] = msg[:BOX]
    msg[:position] = convert(SVector{3, Float64}, transform(center(geometry)))
    msg[:quaternion] = to_lcm_quaternion(transform)
    msg[:string_data] = ""
    float_data = convert(Vector, widths(geometry))
    msg[:float_data] = float_data
    msg[:num_float_data] = length(float_data)
end

function fill_geometry_data!(msg::PyObject, geometry::HyperSphere, transform::Transformation)
    msg[:type] = msg[:SPHERE]
    msg[:position] = convert(SVector{3, Float64}, transform(center(geometry)))
    msg[:quaternion] = to_lcm_quaternion(transform)
    msg[:string_data] = ""
    msg[:float_data] = [radius(geometry)]
    msg[:num_float_data] = 1
end

function fill_geometry_data!(msg::PyObject, geometry::HyperEllipsoid, transform::Transformation)
    msg[:type] = msg[:ELLIPSOID]
    msg[:position] = convert(SVector{3, Float64}, transform(center(geometry)))
    msg[:quaternion] = to_lcm_quaternion(transform)
    msg[:string_data] = ""
    msg[:float_data] = convert(Vector, radii(geometry))
    msg[:num_float_data] = 3
end

function fill_geometry_data!(msg::PyObject, geometry::HyperCylinder{3}, transform::Transformation)
    msg[:type] = msg[:CYLINDER]
    msg[:position] = transform(SVector{3, Float64}(0, 0, 0))
    msg[:quaternion] = to_lcm_quaternion(transform)
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
        reload_model(vis)
        vis
    end

end

Visualizer(data, robot_id_number::Integer=1, lcm::LCM=LCM()) = Visualizer(convert(Robot, data), robot_id_number, lcm)

function reload_model(vis::Visualizer)
    msg = to_lcm(vis.robot, vis.robot_id_number)
    publish(vis.lcm, "DRAKE_VIEWER_LOAD_ROBOT", msg)
end

function launch()
    (stream, proc) = open(`drake-visualizer`)
    proc
end

function draw{T <: Transformation}(model::Visualizer, link_origins::Vector{T})
    msg = lcmdrake[:lcmt_viewer_draw]()
    msg[:timestamp] = convert(Int64, time_ns())
    msg[:num_links] = length(link_origins)
    for (i, origin) in enumerate(link_origins)
        push!(msg["link_name"], model.robot.links[i].name)
        push!(msg["robot_num"], model.robot_id_number)
        push!(msg["position"], convert(SVector{3, Float64}, origin(SVector{3, Float64}(0,0,0))))
        # push!(msg["position"], origin.offset)
        push!(msg["quaternion"], to_lcm_quaternion(origin))
        # push!(msg["quaternion"], to_lcm(qrotation(rotationparameters(origin.scalefwd))))
    end
    publish(model.lcm, "DRAKE_VIEWER_DRAW", msg)
end

function __init__()
    copy!(lcmdrake, pyimport("drake"))
end

end
