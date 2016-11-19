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
    sdf = SignedDistanceField(x -> f(SVector{3, Float64}(x[1], x[2], x[3])) - isosurface_value,
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

typealias Link Vector{GeometryData}
typealias Robot{KeyType} OrderedDict{KeyType, Link}

convert(::Type{Link}, geom::GeometryData) = Link([geom])
convert(::Type{Link}, geometry::AbstractGeometry) = Link(GeometryData(geometry))
convert(::Type{Robot}, link::Link) = OrderedDict(1 => link)
convert{K}(::Type{Robot}, links::Associative{K}) = convert(OrderedDict{K, Link}, links)
convert(::Type{Robot}, links::AbstractArray{Link}) = OrderedDict{Int, Link}(zip(1:length(links), links))
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

function fill_geometry_data!(msg::PyObject, geometry::GeometryPrimitive, transform::Transformation)
    fill_geometry_data!(msg, GLNormalMesh(geometry), transform)
end

function fill_geometry_data!(msg::PyObject, geometry::HyperRectangle, transform::Transformation)
    msg[:type] = msg[:BOX]
    msg[:position] = SVector{3, Float64}(transform(center(geometry))...)
    msg[:quaternion] = to_lcm_quaternion(transform)
    msg[:string_data] = ""
    float_data = convert(Vector, widths(geometry))
    msg[:float_data] = float_data
    msg[:num_float_data] = length(float_data)

end

function fill_geometry_data!(msg::PyObject, geometry::HyperCube, transform::Transformation)
    msg[:type] = msg[:BOX]
    msg[:position] = SVector{3, Float64}(transform(center(geometry))...)
    msg[:quaternion] = to_lcm_quaternion(transform)
    msg[:string_data] = ""
    float_data = convert(Vector, widths(geometry))
    msg[:float_data] = float_data
    msg[:num_float_data] = length(float_data)
end

function fill_geometry_data!(msg::PyObject, geometry::HyperSphere, transform::Transformation)
    msg[:type] = msg[:SPHERE]
    msg[:position] = SVector{3, Float64}(transform(center(geometry))...)
    msg[:quaternion] = to_lcm_quaternion(transform)
    msg[:string_data] = ""
    msg[:float_data] = [radius(geometry)]
    msg[:num_float_data] = 1
end

function fill_geometry_data!(msg::PyObject, geometry::HyperEllipsoid, transform::Transformation)
    msg[:type] = msg[:ELLIPSOID]
    msg[:position] = SVector{3, Float64}(transform(center(geometry))...)
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
    msg = drakevis[:lcmt_viewer_geometry_data]()
    msg[:color] = to_lcm(geometry_data.color)

    fill_geometry_data!(msg, geometry_data.geometry, geometry_data.transform)
    msg
end

function to_lcm(link::Link, name::String, robot_id_number::Real)
    msg = drakevis[:lcmt_viewer_link_data]()
    msg[:name] = name
    msg[:robot_num] = robot_id_number
    msg[:num_geom] = length(link)
    for geometry_data in link
        push!(msg["geom"], to_lcm(geometry_data))
    end
    msg
end

function to_lcm(robot::Robot, robot_id_number::Real)
    msg = drakevis[:lcmt_viewer_load_robot]()
    msg[:num_links] = length(robot.links)
    for (key, link) in robot.links
        push!(msg["link"], to_lcm(link, string(key), robot_id_number))
    end
    msg
end

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

function reload_model{V <: Visualizer}(visualizers::AbstractArray{V})
    msg = drakevis[:lcmt_viewer_load_robot]()
    msg[:num_links] = 0
    for vis in visualizers
        msg[:num_links] = msg[:num_links] + length(vis.links)
        for (key, link) in vis.links
            push!(msg["link"], to_lcm(link, string(key), vis.robot_id_number))
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
        push!(msg["link_name"], string(key))
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
