to_lcm(q::Quat) = SVector{4, Float64}(q.w, q.x, q.y, q.z)
to_lcm_quaternion(matrix::AbstractMatrix) = to_lcm(Quat(matrix))
to_lcm_quaternion(matrix::UniformScaling) = to_lcm(Quat(1.0, 0, 0, 0))
to_lcm_quaternion(transform::AbstractAffineMap) = to_lcm_quaternion(transform_deriv(transform, SVector{3, Float64}(0,0,0)))
to_lcm_quaternion(transform::IdentityTransformation) = to_lcm_quaternion(Quat(1.0, 0, 0, 0))

to_lcm(color::Colorant) = Float64[red(color); green(color); blue(color); alpha(color)]

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
