using .LazyTrees: LazyTree, data, children

type GeometryData{G <: AbstractGeometry, C <: Colorant}
    geometry::G
    color::C
end

GeometryData(g::AbstractGeometry) = GeometryData(g, RGBA{Float64}(1, 0, 0, 0.5))

type VisData
    transform::Transformation
    geometries::Vector{GeometryData}
end

VisData() = VisData(IdentityTransformation(), GeometryData[])

immutable PointCloud{Point} <: AbstractGeometry
    points::Vector{Point}
    channels::Dict{Symbol, Any}
end

PointCloud{Point}(points::AbstractVector{Point}) =
    PointCloud{Point}(points, Dict{Symbol, Any}())

typealias Path Vector{Symbol}

immutable CommandQueue
    delete::Vector{Path}
    load::Vector{Path}
    draw::Vector{Path}

    CommandQueue() = new(Path[], Path[], Path[])
end

isempty(queue::CommandQueue) = isempty(queue.delete) && isempty(queue.load) && isempty(queue.draw)

function empty!(queue::CommandQueue)
    empty!(queue.delete)
    empty!(queue.load)
    empty!(queue.draw)
end

immutable Visualizer
    lcm::LCM
    tree::LazyTree{Symbol, VisData}
    queue::CommandQueue

    function Visualizer(lcm::LCM=LCM())
        vis = new(lcm, LazyTree{Symbol, VisData}(), CommandQueue())
        function handle_msg(channel, msg)
            onresponse(vis, msg)
        end
        subscribe(lcm, "DRAKE_VIEWER2_RESPONSE", handle_msg, drakevis[:viewer2_comms_t])
        vis
    end
end

function publish!(f::Function, vis::Visualizer)
    f()
    publish!(vis)
end

load!(vis::Visualizer, path::AbstractVector) = push!(vis.queue.load, path)

function load!(vis::Visualizer, path::AbstractVector, geom)
    vis.tree[path].data.geometries = [geom]
    load!(vis, path)
end

load!(vis::Visualizer, path::AbstractVector, geom::AbstractGeometry) =
    load!(vis, path, GeometryData(geom))

draw!(vis::Visualizer, path::AbstractVector) = push!(vis.queue.draw, path)

function draw!(vis::Visualizer, path::AbstractVector, tform)
    vis.tree[path].data.transform = tform
    draw!(vis, path)
end

function delete!(vis::Visualizer, path::AbstractVector)
    delete!(vis.tree, path)
    push!(vis.queue.delete, path)
end

function publish!(vis::Visualizer)
    if !isempty(vis.queue)
        data = serialize(vis, vis.queue)
        msg = to_lcm(data)
        publish(vis.lcm, "DRAKE_VIEWER2_REQUEST", msg)

        # Wait at most 100ms for the first response
        handle(vis.lcm, Dates.Millisecond(100))

        # Clear the queue of any other messages
        while handle(vis.lcm, Dates.Millisecond(0))
            # nothing
        end

        true
    else
        false
    end
end

function onresponse(vis::Visualizer, msg)
    data = JSON.parse(msg[:data])
    if data["status"] == 0
        empty!(vis.queue)
    elseif data["status"] == 1
        for path in Trees.descendants(vis.tree)
            queue_load!(vis, path)
        end
    else
        error("unhandled: $data")
    end
end

function to_lcm(data::Associative)
    msg = drakevis[:viewer2_comms_t]()
    msg[:timestamp] = data["timestamp"]
    msg[:format] = "viewer2_json"
    msg[:format_version_major] = 1
    msg[:format_version_minor] = 0
    encoded = JSON.json(data)
    msg[:num_bytes] = length(encoded)
    msg[:data] = encoded
    msg
end

function serialize(vis::Visualizer, queue::CommandQueue)
    timestamp = time_ns()
    data = Dict(
        "timestamp" => timestamp,
        "delete" => [],
        "load" => [],
        "draw" => []
    )
    for path in queue.delete
        push!(data["delete"], Dict("path" => path))
    end
    for path in queue.load
        visdata = vis.tree[path].data
        if length(visdata.geometries) > 0
            push!(data["load"], serialize(path, visdata.geometries))
        end
    end
    for path in queue.draw
        visdata = vis.tree[path].data
        push!(data["draw"],
              Dict("path" => path,
                   "transform" => serialize(visdata.transform)
              )
        )
    end
    data
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
    transform = intrinsic_transform(geomdata.geometry)
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
serialize(g::HyperEllipsoid) = Dict("type" => "ellipsoid", "radii" => radii(g))
serialize(g::HyperCylinder{3}) = Dict("type" => "cylinder",
                                      "length" => length(g),
                                      "radius" => radius(g))
serialize(g::HyperCube) = Dict("type" => "box", "lengths" => widths(g))
serialize(g::GeometryPrimitive) = serialize(GLNormalMesh(g))

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
    Dict("translation" => translation(tform),
         "quaternion" => quaternion(tform))
end

quaternion(::IdentityTransformation) = SVector(1., 0, 0, 0)
quaternion(tform::AbstractAffineMap) = quaternion(transform_deriv(tform, SVector(0., 0, 0)))
quaternion(matrix::UniformScaling) = quaternion(IdentityTransformation())
quaternion(matrix::AbstractMatrix) = quaternion(Quat(matrix))
quaternion(quat::Quat) = SVector(quat.w, quat.x, quat.y, quat.z)

translation(tform::Transformation) = tform(SVector(0., 0, 0))
