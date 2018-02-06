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
struct MsgPackNumpyArray{A <: AbstractArray}
    data::A
end

function pack(s::IO, a::MsgPackNumpyArray)
    # TODO: this is inefficient because we have to create a new IOBuffer
    # to pack the data into, and then we pack that buffer into the Ext.
    raw_data = reinterpret(UInt8, a.data, (length(a.data) * sizeof(eltype(a.data)),)) 
    pack(s, MsgPack.Ext(0x50, MsgPack.pack(Dict(
             "nd" => true,
             "type" => numpy_dtype_str(eltype(a.data)),
             "shape" => reverse(size(a.data)),
             "data" => raw_data
            ))))
end

msgpack_numpy_format(x::AbstractArray) = MsgPackNumpyArray(x)

function msgpack_numpy_format(x::AbstractVector{<:StaticVector{N, T}}) where {N, T}
    MsgPackNumpyArray(reinterpret(T, x, (N, length(x))))
end

function msgpack_numpy_format(x::AbstractVector{Cin}, ::Type{Cout}=RGB{Float32}) where {Cin <: Colorant, Cout <: Colorant}
    xout = convert(Vector{Cout}, x)
    MsgPackNumpyArray(reinterpret(eltype(Cout), x, (length(Cout), length(x))))
end

function msgpack_numpy_format(faces::AbstractVector{<:Face{N, T}}) where {N, T}
    MsgPackNumpyArray(
        reinterpret(Int, [raw.(convert(Face{N, GeometryTypes.OffsetInteger{-1, Int}}, face)) for face in faces], (N, length(faces))))
end

pack(s::IO, v::StaticVector) = pack(s, Tuple(v))
pack(s::IO, v::Symbol) = pack(s, String(v))
pack(s::IO, c::Colorant) = pack(s, (red(c), green(c), blue(c), alpha(c)))
pack(s::IO, tform::Transformation) = pack(s, Dict("translation" => translation(tform), "quaternion" => quaternion(tform)))
pack(s::IO, g::GeometryData) = pack(s, Dict(vcat(geometry_fields(g.geometry), common_fields(g))))

common_fields(g::GeometryData) = ["color" => g.color,
                                  "transform" => compose(g.transform, intrinsic_transform(g.geometry))]

geometry_fields(g::HyperRectangle) = ["type" => "box", "lengths" => widths(g)]
geometry_fields(g::HyperSphere) = ["type" => "sphere", "radius" => radius(g)]
geometry_fields(g::HyperEllipsoid) = ["type" => "ellipsoid", "radii" => radii(g)]
geometry_fields(g::HyperCylinder{3}) = ["type" => "cylinder",
                                       "length" => length(g),
                                       "radius" => radius(g)]
geometry_fields(g::HyperCube) = ["type" => "box", "lengths" => widths(g)]
geometry_fields(g::Triad) = ["type" => "triad", "scale" => g.scale, "tube" => g.tube]
geometry_fields(g::PointCloud) = ["type" => "pointcloud",
                                  "points" => msgpack_numpy_format(g.points),
                                  "channels" => Dict(
                                    [(name => msgpack_numpy_format(value)) for (name, value) in g.channels])]
geometry_fields(g::AbstractMesh) = ["type" => "mesh_data",
                                    "vertices" => msgpack_numpy_format(vertices(g)),
                                    "faces" => msgpack_numpy_format(faces(g))]
geometry_fields(g::MeshFile) = ["type" => "mesh_file",
                                "filename" => g.filename,
                                "scale" => (1.0, 1.0, 1.0)]

function geometry_fields(g::PolyLine)
    params = ["type" => "line",
              "points" => msgpack_numpy_format(g.points),
              "radius" => g.radius,
              "closed" => g.closed]
    if !isnull(g.start_head)
        append!(params, ["start_head" => true,
                         "head_radius" => get(g.start_head).radius,
                         "head_length" => get(g.start_head).length])
    end
    if !isnull(g.end_head)
        append!(params, ["end_head" => true,
                         "head_radius" => get(g.end_head).radius,
                         "head_length" => get(g.end_head).length])
    end
    params
end

# Any unhandled geometries are converted to a mesh first
geometry_fields(g::GeometryPrimitive) = geometry_fields(GLNormalMesh(g))

intrinsic_transform(g) = IdentityTransformation()
intrinsic_transform(g::Nullable) = isnull(g) ? IdentityTransformation() : intrinsic_transform(get(g))
intrinsic_transform(geomdata::GeometryData) = intrinsic_transform(geomdata.geometry)
intrinsic_transform(g::HyperRectangle) = Translation(center(g)...)
intrinsic_transform(g::HyperSphere) = Translation(center(g)...)
intrinsic_transform(g::HyperEllipsoid) = Translation(center(g)...)
intrinsic_transform(g::HyperCylinder) = Translation(center(g)...)
intrinsic_transform(g::HyperCube) = Translation(center(g)...)

quaternion(::IdentityTransformation) = SVector(1., 0, 0, 0)
quaternion(tform::AbstractAffineMap) = quaternion(transform_deriv(tform, SVector(0., 0, 0)))
quaternion(matrix::UniformScaling) = quaternion(IdentityTransformation())
quaternion(matrix::AbstractMatrix) = quaternion(Quat(matrix))
quaternion(quat::Quat) = SVector(quat.w, quat.x, quat.y, quat.z)

translation(tform::Transformation) = tform(SVector(0., 0, 0))
