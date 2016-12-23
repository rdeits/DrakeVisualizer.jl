import Base: isempty

type GeometryData{GeometryType <: AbstractGeometry, TransformType <: Transformation}
    geometry::GeometryType
    transform::TransformType
    color::RGBA{Float64}
end

typealias Link Dict{String, GeometryData}

abstract Command
immutable Delete <: Command
end
command_name(::Delete) = "delete"
immutable Load <: Command
end
command_name(::Load) = "load"
immutable Draw <: Command
end
command_name(::Draw) = "draw"

immutable CommandQueue
    delete::Set{String}
    load::Set{String}
    draw::Set{String}

    CommandQueue() = new(Set{String}(), Set{String}(), Set{String}())
end

immutable Visualizer
    lcm::LCM
    links::OrderedDict{String, Link}
    transforms::OrderedDict{String, Transformation}
    queue::CommandQueue

    function Visualizer(lcm::LCM=LCM())
        vis = new(lcm,
                    OrderedDict{String, Link}(),
                    OrderedDict{String, Transformation}(),
                    CommandQueue())
        function handle_msg(channel, msg)
            onresponse(vis, JSON.parse(msg[:data]))
        end
        subscribe(lcm, "DRAKE_VIEWER2_RESPONSE", handle_msg, drakevis[:viewer2_comms_t])
        vis
    end
end

function queue_load!(vis::Visualizer, path::AbstractString)
    push!(vis.queue.load, path)
end

function queue_load!(vis::Visualizer, path::AbstractString, link)
    vis.links[path] = link
    push!(vis.queue.load, path)
end

function load!(vis::Visualizer, path::AbstractString, link::Link)
    queue_load!(vis, path, link)
    publish!(vis)
end

function queue_draw!(vis::Visualizer, path::String, tform)
    vis.transforms[path] = tform
    push!(vis.queue.draw, path)
end

function draw!(vis::Visualizer, path::String, tform)
    queue_draw!(vis, path, tform)
    publish!(vis)
end

function queue_delete!(vis::Visualizer, path::String)
    delete!(vis.links, path)
    delete!(vis.transforms, path)
    push!(vis.queue.delete, path)
end

isempty(queue::CommandQueue) = isempty(queue.delete) && isempty(queue.load) && isempty(queue.draw)

publish_order(vis::Visualizer) = ((Delete(), vis.queue.delete),
                                  (Draw(), vis.queue.draw),
                                  (Load(), vis.queue.load),
                                  (Draw(), vis.queue.draw))

function publish!(vis::Visualizer)
    for (cmd, queue) in publish_order(vis)
        publish!(vis, cmd, queue)
    end
end

function publish!(vis::Visualizer, cmd::Command, queue)
    if !isempty(queue)
        data = serialize(vis, cmd, queue)
        msg = to_lcm(data)
        publish(vis.lcm, "DRAKE_VIEWER2_REQUEST", msg)
        PyCall.pycall(vis.lcm.lcm_obj[:handle_timeout], PyCall.PyObject, 100)
    end
end

function onresponse(vis::Visualizer, msgdata)
    data = msgdata["data"]
    empty = []
    for path in get(data, "missing_paths", empty)
        queue_load!(vis, join(path, '/'))
    end
    for path in get(data, "drew_paths", empty)
        delete!(vis.queue.draw, join(path, '/'))
    end
    for path in get(data, "loaded_paths", empty)
        delete!(vis.queue.load, join(path, '/'))
    end
    for path in get(data, "deleted_paths", empty)
        delete!(vis.queue.delete, join(path, '/'))
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

function serialize(vis::Visualizer, cmd::Command, queue)
    timestamp = time_ns()
    data = Dict(
        "timestamp" => timestamp,
        "type" => command_name(cmd),
        "data" => command_data(vis, cmd, queue)
    )
end

function command_data(vis::Visualizer, cmd::Delete, queue)
    Dict("paths" => collect(queue))
end

function command_data(vis::Visualizer, cmd::Load, queue)
    links = [serialize(p, vis.links[p]) for p in queue]
    Dict("links" => links)
end

serialize(color::RGBA) = [color.r, color.g, color.b, color.alpha]

function serialize(path::String, link::Link)
    geometries = [serialize(name, geom) for (name, geom) in link]
    Dict("path" => split(path, '/'),
        "geometries" => geometries)
end

function serialize(name::AbstractString, geomdata::GeometryData)
    params = serialize(geomdata.geometry)
    params["color"] = serialize(geomdata.color)
    intrinsic_tform = intrinsic_transform(geomdata.geometry)
    Dict("name" => name,
         "pose" => serialize(compose(geomdata.transform, intrinsic_tform)),
         "parameters" => params)
end

intrinsic_transform(geom::AbstractMesh) = IdentityTransformation()
intrinsic_transform(geom::AbstractGeometry) = Translation(center(geom)...)

serialize(v::Vec) = convert(Vector, v)
serialize(v::Point) = convert(Vector, v)

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

function serialize{N, T, Offset}(face::Face{N, T, Offset})
    convert(Vector, convert(Face{N, T, -1}, face))
end

function command_data(vis::Visualizer, cmd::Draw, queue)
    commands = [serialize(p, vis.transforms[p]) for p in queue]
    Dict("commands" => commands)
end

function serialize(path::String, tform::Transformation)
    Dict("path" => split(path, '/'),
        "pose" => serialize(tform))
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
