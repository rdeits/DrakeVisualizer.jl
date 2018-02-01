import MsgPack: pack

function serialize(vis::CoreVisualizer, queue::CommandQueue)
    timestamp = time_ns()
    delete_cmds = []
    setgeometry_cmds = []
    settransform_cmds = []
    for path in queue.delete
        push!(delete_cmds, Dict("path" => path))
    end
    for path in queue.setgeometry
        visdata = vis.tree[path].data
        push!(setgeometry_cmds, (path, visdata.geometries))
        # push!(setgeometry_cmds, serialize(path, visdata.geometries))
    end
    for path in queue.settransform
        visdata = vis.tree[path].data
        push!(settransform_cmds,
              Dict{String, Any}("path" => path,
                                "transform" => visdata.transform
              )
        )
    end
    buf = IOBuffer()
    # write(buf, "treeviewer2.0 ")
    pack(buf,
      Dict{String, Any}(
        "timestamp" => timestamp,
        "delete" => delete_cmds,
        "setgeometry" => setgeometry_cmds,
        "settransform" => settransform_cmds
    ))
    take!(buf)
end

# """
# Fallback method. This is needed because we're going to overload packing
# of Arrays to use the msgpack-numpy format, and we don't want to change
# the behavior of MsgPack.pack on Arrays
# """
# _pack(s::IO, x) = MsgPack.pack(s, x)

function pack(s::IO, geompaths::Tuple{AbstractVector, AbstractVector{<:GeometryData}})
    path, geomdatas = geompaths
    pack(s,
        Dict(
             "path" => path,
             "geometries" => geomdatas
             )
        )
end

numpy_dtype_str(::Type{Float64}) = "f8"
numpy_dtype_str(::Type{Float32}) = "f4"
numpy_dtype_str(::Type{Int64}) = "i8"
numpy_dtype_str(::Type{Int32}) = "i4"

"""
Pack arrays into the style expected by msgpack-numpy. Note that we reverse the
order of the sizes because numpy will interpret the data as row-major instead
of column-major.
"""
function msgpack_numpy_format(x::AbstractArray)
    Dict(
         "nd" => true,
         "type" => numpy_dtype_str(eltype(x)),
         "shape" => reverse(size(x)),
         "data" => reinterpret(UInt8, x, (length(x) * sizeof(eltype(x)),))
        )
end

function msgpack_numpy_format(x::AbstractVector{<:StaticVector{N, T}}) where {N, T}
    msgpack_numpy_format(reinterpret(T, x, (N, length(x))))
end

function msgpack_numpy_format(x::AbstractVector{C}, alpha=false) where {C <: Colorant}
    if alpha
        data = Array{eltype(C)}(4, length(x))
        for i in 1:length(x)
            data[1, i] = red(x[i])
            data[2, i] = green(x[i])
            data[3, i] = blue(x[i])
            data[4, i] = alpha(x[i])
        end
    else
        data = Array{eltype(C)}(3, length(x))
        for i in 1:length(x)
            data[1, i] = red(x[i])
            data[2, i] = green(x[i])
            data[3, i] = blue(x[i])
        end
    end
    msgpack_numpy_format(data)
end

# function serialize(path::AbstractVector, geomdatas::AbstractVector{<:GeometryData})
#     params = serialize.(geomdatas)
#     if length(params) == 1
#         Dict("path" => serialize(path),
#              "geometry" => params[1])
#     else
#         Dict("path" => serialize(path),
#              "geometries" => params)
#     end
# end

function common_fields(g::GeometryData)
    transform = compose(g.transform,
                        intrinsic_transform(g.geometry))
    ("color" => g.color, "transform" => transform)
end

pack(s::IO, v::StaticVector) = pack(s, Tuple(v))
pack(s::IO, v::Symbol) = pack(s, String(v))
pack(s::IO, c::Colorant) = pack(s, (red(c), green(c), blue(c), alpha(c)))

pack(s::IO, g::GeometryData{<:HyperRectangle}) =
    pack(s, Dict("type" => "box", "lengths" => widths(g.geometry), common_fields(g)...))

pack(s::IO, g::GeometryData{<:HyperSphere}) =
    pack(s, Dict("type" => "sphere", "radius" => radius(g.geometry), common_fields(g)...))

function pack(s::IO, g::GeometryData{<:PointCloud{T}}) where T
    pack(s, Dict(
        "type" => "pointcloud",
        "points" => msgpack_numpy_format(reinterpret(T, g.geometry.points, (3, length(g.geometry.points)))),
        "channels" => Dict(
            [(name => msgpack_numpy_format(value)) for (name, value) in g.geometry.channels]),
        common_fields(g)...))
end

intrinsic_transform(g) = IdentityTransformation()
intrinsic_transform(g::Nullable) = isnull(g) ? IdentityTransformation() : intrinsic_transform(get(g))
intrinsic_transform(geomdata::GeometryData) = intrinsic_transform(geomdata.geometry)
intrinsic_transform(g::HyperRectangle) = Translation(center(g)...)
intrinsic_transform(g::HyperSphere) = Translation(center(g)...)
intrinsic_transform(g::HyperEllipsoid) = Translation(center(g)...)
intrinsic_transform(g::HyperCylinder) = Translation(center(g)...)
intrinsic_transform(g::HyperCube) = Translation(center(g)...)

function pack(s::IO, g::GeometryData{<:AbstractMesh})
    pack(s, Dict(
        "type" => "mesh_data",
        "vertices" => vertices(g.geometry),
        "faces" => faces(g.geometry),
        common_fields(g)...))
end

function pack(s::IO, faces::Vector{<:Face{N, T}}) where {N, T}
    pack(s,
         msgpack_numpy_format(
            [raw.(convert(Face{N, GeometryTypes.OffsetInteger{-1, Int}}, face)) for face in faces]))
end

# pack(s::IO, face::Face{N, T}) where {N, T} =
#     pack(s, Tuple(raw.(convert(Face{N, GeometryTypes.OffsetInteger{-1, Int}}, face))))

# serialize(color::Colorant) = (red(color),
#                               green(color),
#                               blue(color),
#                               alpha(color))
# serialize(p::Path) = string.(p)
# serialize(v::Vector) = v
# serialize(v::Vec) = convert(Vector, v)
# serialize(v::Point) = convert(Vector, v)
# serialize(v::StaticArray) = convert(Vector, v)
# serialize(face::Face{N, T}) where {N, T} =
#   raw.(convert(Face{N, GeometryTypes.OffsetInteger{-1, Int}}, face))
# serialize(g::HyperRectangle) = Dict("type" => "box", "lengths" => serialize(widths(g)))
# serialize(g::HyperSphere) = Dict("type" => "sphere", "radius" => radius(g))
# serialize(g::HyperEllipsoid) = Dict("type" => "ellipsoid", "radii" => serialize(radii(g)))
# serialize(g::HyperCylinder{3}) = Dict("type" => "cylinder",
#                                       "length" => length(g),
#                                       "radius" => radius(g))
# serialize(g::HyperCube) = Dict("type" => "box", "lengths" => widths(g))
# serialize(g::GeometryPrimitive) = serialize(GLNormalMesh(g))
# serialize(g::Triad) = Dict{String, Any}("type" => "triad",
#                                         "scale" => g.scale,
#                                         "tube" => g.tube)

# function serialize(g::AbstractMesh)
#     Dict("type" => "mesh_data",
#          "vertices" => serialize.(vertices(g)),
#          "faces" => serialize.(faces(g)))
# end

# function serialize(f::MeshFile)
#     Dict("type" => "mesh_file",
#          "filename" => f.filename,
#          "scale" => SVector(1.0, 1.0, 1.0))
# end

# function serialize(g::PointCloud)
#     params = Dict("type" => "pointcloud",
#                   "points" => serialize.(g.points),
#                   "channels" => Dict{String, Any}())
#     for (channel, values) in g.channels
#         params["channels"][string(channel)] = serialize.(values)
#     end
#     params
# end

# function serialize(g::PolyLine)
#     params = Dict("type" => "line",
#                   "points" => serialize.(g.points),
#                   "radius" => g.radius,
#                   "closed" => g.closed
#         )
#     if !isnull(g.start_head)
#         params["start_head"] = true
#         params["head_radius"] = get(g.start_head).radius
#         params["head_length"] = get(g.start_head).length
#     end
#     if !isnull(g.end_head)
#         params["end_head"] = true
#         params["head_radius"] = get(g.end_head).radius
#         params["head_length"] = get(g.end_head).length
#     end
#     params
# end

# function serialize(tform::Transformation)
#     Dict{String, Vector{Float64}}("translation" => translation(tform),
#                       "quaternion" => quaternion(tform))
# end

function pack(s::IO, tform::Transformation)
    pack(s, Dict(
        "translation" => translation(tform),
        "quaternion" => quaternion(tform)
        ))
end

quaternion(::IdentityTransformation) = SVector(1., 0, 0, 0)
quaternion(tform::AbstractAffineMap) = quaternion(transform_deriv(tform, SVector(0., 0, 0)))
quaternion(matrix::UniformScaling) = quaternion(IdentityTransformation())
quaternion(matrix::AbstractMatrix) = quaternion(Quat(matrix))
quaternion(quat::Quat) = SVector(quat.w, quat.x, quat.y, quat.z)

translation(tform::Transformation) = tform(SVector(0., 0, 0))
