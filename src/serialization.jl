function serialize(vis::CoreVisualizer, queue::CommandQueue)
    utime = time_ns()
    deletes = Dict{String, Any}[]
    loads = Dict{String, Any}[]
    draws = Dict{String, Any}[]
    for path in queue.delete
        push!(deletes, Dict("path" => path))
    end
    for path in queue.load
        visdata = vis.tree[path].data
        if length(visdata.geometries) > 0
            push!(loads, serialize(path, visdata.geometries))
        end
    end
    for path in queue.draw
        visdata = vis.tree[path].data
        tform = serialize(visdata.transform)
        push!(draws,
              Dict{String, Any}("path" => path,
                                "transform" => tform
              )
        )
    end
    data = Dict{String, Any}(
        "utime" => utime,
        "delete" => deletes,
        "load" => loads,
        "draw" => draws
    )
end

function serialize(path::AbstractVector, geomdatas::Vector{GeometryData})
    params = serialize.(geomdatas)
    if length(params) == 1
        Dict("path" => serialize(path),
             "geometry" => params[1])
    else
        Dict("path" => serialize(path),
             "geometries" => params)
    end
end

function serialize(geomdata::GeometryData)
    params = serialize(geomdata.geometry)
    params["color"] = serialize(geomdata.color)
    transform = compose(geomdata.transform,
                        intrinsic_transform(geomdata.geometry))
    if transform != IdentityTransformation()
        params["transform"] = serialize(transform)
    end
    params
end

intrinsic_transform(geom::Nullable) = isnull(geom) ? IdentityTransformation() : intrinsic_transform(get(geom))
intrinsic_transform(geomdata::GeometryData) = intrinsic_transform(geomdata.geometry)
intrinsic_transform(geom::AbstractMesh) = IdentityTransformation()
intrinsic_transform(geom::AbstractGeometry) = Translation(center(geom)...)
intrinsic_transform(geom::PointCloud) = IdentityTransformation()
intrinsic_transform(triad::Triad) = IdentityTransformation()

serialize(color::Colorant) = (red(color),
                              green(color),
                              blue(color),
                              alpha(color))
serialize(p::Path) = string.(p)
serialize(v::Vector) = v
serialize(v::Vec) = convert(Vector, v)
serialize(v::Point) = convert(Vector, v)
serialize(v::StaticArray) = convert(Vector, v)
serialize{N, T, Offset}(face::Face{N, T, Offset}) =
    convert(Vector, convert(Face{N, T, -1}, face))
serialize(g::HyperRectangle) = Dict("type" => "box", "lengths" => serialize(widths(g)))
serialize(g::HyperSphere) = Dict("type" => "sphere", "radius" => radius(g))
serialize(g::HyperEllipsoid) = Dict("type" => "ellipsoid", "radii" => serialize(radii(g)))
serialize(g::HyperCylinder{3}) = Dict("type" => "cylinder",
                                      "length" => length(g),
                                      "radius" => radius(g))
serialize(g::HyperCube) = Dict("type" => "box", "lengths" => widths(g))
serialize(g::GeometryPrimitive) = serialize(GLNormalMesh(g))
serialize(g::Triad) = Dict{String, Any}("type" => "triad")

function serialize(g::AbstractMesh)
    Dict("type" => "mesh_data",
         "vertices" => serialize.(vertices(g)),
         "faces" => serialize.(faces(g)))
end

function serialize(g::PointCloud)
    params = Dict("type" => "pointcloud",
                  "points" => serialize.(g.points),
                  "channels" => Dict{String, Any}())
    for (channel, values) in g.channels
        params["channels"][string(channel)] = serialize.(values)
    end
    params
end

function serialize(tform::Transformation)
    Dict{String, Vector{Float64}}("translation" => translation(tform),
                      "quaternion" => quaternion(tform))
end

quaternion(::IdentityTransformation) = SVector(1., 0, 0, 0)
quaternion(tform::AbstractAffineMap) = quaternion(transform_deriv(tform, SVector(0., 0, 0)))
quaternion(matrix::UniformScaling) = quaternion(IdentityTransformation())
quaternion(matrix::AbstractMatrix) = quaternion(Quat(matrix))
quaternion(quat::Quat) = SVector(quat.w, quat.x, quat.y, quat.z)

translation(tform::Transformation) = tform(SVector(0., 0, 0))
