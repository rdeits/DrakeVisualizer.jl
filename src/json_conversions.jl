using .Trees: LazyTree, data, children

# typealias Transform AffineMap{SMatrix{3, 3, Float64, 9}, SVector{3, Float64}}

type GeometryData{G <: AbstractGeometry, C <: Colorant}
    geometry::G
    color::C
end

type VisData
    transform::Nullable{Transformation}
    geometry::Nullable{GeometryData}
end

VisData() = VisData(Nullable{Transformation}(), Nullable{GeometryData}())

type PointCloud{Point, Color} <: AbstractGeometry
    points::Vector{Point}
    colors::Nullable{Vector{Color}}
    intensities::Nullable{Vector{Float64}}
end

PointCloud{Point}(points::AbstractVector{Point}) =
    PointCloud{Point, RGB{Float64}}(points, Nullable(), Nullable())

immutable CommandQueue
    delete::Vector{Vector{String}}
    load::Vector{Vector{String}}
    draw::Vector{Vector{String}}

    CommandQueue() = new(Vector{String}[], Vector{String}[], Vector{String}[])
end

isempty(queue::CommandQueue) = isempty(queue.delete) && isempty(queue.load) && isempty(queue.draw)

function empty!(queue::CommandQueue)
    empty!(queue.delete)
    empty!(queue.load)
    empty!(queue.draw)
end

immutable Visualizer
    lcm::LCM
    tree::LazyTree{VisData}
    queue::CommandQueue

    function Visualizer(lcm::LCM=LCM())
        vis = new(lcm, LazyTree{VisData}(), CommandQueue())
        function handle_msg(channel, msg)
            onresponse(vis, msg)
        end
        subscribe(lcm, "DRAKE_VIEWER2_RESPONSE", handle_msg, drakevis[:viewer2_comms_t])
        vis
    end
end

function queue_load!(vis::Visualizer, path::AbstractVector)
    queue_draw!(vis, path)
    push!(vis.queue.load, path)
end

function queue_load!(vis::Visualizer, path::AbstractVector, geom)
    vis.tree[path].data.geometry = geom
    queue_load!(vis, path)
end

function load!(vis::Visualizer, path::AbstractVector, geom)
    queue_load!(vis, path, geom)
    publish!(vis)
end

queue_draw!(vis::Visualizer, path::AbstractVector) = push!(vis.queue.draw, path)

function queue_draw!(vis::Visualizer, path::AbstractVector, tform)
    vis.tree[path].data.transform = tform
    queue_draw!(vis, path)
end

function draw!(vis::Visualizer, path::AbstractVector, tform)
    queue_draw!(vis, path, tform)
    publish!(vis)
end

function queue_delete!(vis::Visualizer, path::AbstractVector)
    delete!(vis.tree, path)
    push!(vis.queue.delete, path)
end

function delete!(vis::Visualizer, path::AbstractVector)
    queue_delete!(vis, path)
    publish!(vis)
end


function publish!(vis::Visualizer)
    if !isempty(vis.queue)
        data = serialize(vis, vis.queue)
        msg = to_lcm(data)
        publish(vis.lcm, "DRAKE_VIEWER2_REQUEST", msg)
        PyCall.pycall(vis.lcm.lcm_obj[:handle_timeout], PyCall.PyObject, 100)
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
        if !isnull(visdata.geometry)
            push!(data["load"], serialize(path, get(visdata.geometry)))
        end
    end
    for path in queue.draw
        visdata = vis.tree[path].data
        if isnull(visdata.transform)
            transform = IdentityTransformation()
        else
            transform = get(visdata.transform)
        end
        push!(data["draw"],
              Dict("path" => path,
                   "transform" => serialize(
                        compose(transform,
                                intrinsic_transform(visdata.geometry)))
              )
        )
    end
    data
end

serialize(color::Colorant) = (red(color),
                              green(color),
                              blue(color),
                              alpha(color))

function serialize(path::AbstractVector, geomdata::GeometryData)
    params = serialize(geomdata.geometry)
    params["color"] = serialize(geomdata.color)
    Dict("path" => path,
         "geometry" => params)
end

intrinsic_transform(geom::Nullable) = isnull(geom) ? IdentityTransformation() : intrinsic_transform(get(geom))
intrinsic_transform(geomdata::GeometryData) = intrinsic_transform(geomdata.geometry)
intrinsic_transform(geom::AbstractMesh) = IdentityTransformation()
intrinsic_transform(geom::AbstractGeometry) = Translation(center(geom)...)
intrinsic_transform(geom::PointCloud) = IdentityTransformation()

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
    if !isnull(g.colors)
        params["channels"]["rgb"] = serialize.(get(g.colors))
    end
    if !isnull(g.intensities)
        params["channels"]["intensity"] = serialize.(get(g.intensities))
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
