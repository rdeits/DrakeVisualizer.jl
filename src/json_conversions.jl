import Base: isempty


type GeometryData{GeometryType <: AbstractGeometry, TransformType <: Transformation}
    geometry::GeometryType
    transform::TransformType
    color::RGBA{Float64}
    name::String
end

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

type PendingCommands
    delete::Set{String}
    load::Set{String}
    draw::Set{String}
end

immutable Visualizer
    links::OrderedDict{String, Link}
    transforms::OrderedDict{String, Transformation}
    prefix::String
    queue::PendingCommands
    lcm::LCM
end

function queue_load!(vis::Visualizer, path::String, link::Link)
    vis.links[path] = link
    push!(vis.queue.load, path)
end

function queue_draw!(vis::Visualizer, path::String, tform)
    vis.transforms[path] = tform
    push!(vis.queue.draw, path)
end

function queue_delete!(vis::Visualizer, path::String)
    delete!(vis.links, path)
    delete!(vis.transforms, path)
    push!(vis.queue.delete, path)
end

isempty(queue::PendingCommands) = isempty(queue.delete) && isempty(queue.load) && isempty(queue.draw)

function publish!(vis::Visualizer)
    publish!(vis, Delete(), vis.queue.delete)
    publish!(vis, Load(), vis.queue.load)
    publish!(vis, Draw(), vis.queue.draw)
end

function onresponse(vis::Visualizer, msgdata)
    empty = []
    for path in get(msgdata, "missing_paths", empty)
        queue_load!(vis, path)
    end
    for path in get(msgdata, "drew_paths", empty)
        delete!(vis.queue.draw, path)
    end
    for path in get(msgdata, "loaded_paths", empty)
        delete!(vis.queue.load, path)
    end
    for path in get(msgdata, "deleted_paths", empty)
        delete!(vis.queue.delete, path)
    end
    if !isempty(vis.queue)
        publish!(vis)
    end
end

function publish!(vis::Visualizer, cmd::Command, queue)
    data = serialize(vis, cmd, queue)
    msg = to_lcm(data)
    publish(vis.lcm, "DRAKE_VIEWER2_REQUEST", msg)
end

function serialize(vis::Visualizer, cmd::Command, queue)
    timestamp = time_ns()
    data = Dict(
        "timestamp" => timestamp,
        "type" => command_name(cmd),
        "data" => command_data(vis, cmd, queue)
    )
end

fullpath(vis::Visualizer, path::AbstractString) = join((vis.prefix, path), '/')

function command_data(vis::Visualizer, cmd::Delete, queue)
    Dict("paths" => [fullpath(vis, p) for p in queue])
end

function command_data(vis::Visualizer, cmd::Load, queue)
    links = [serialize(fullpath(vis, p), vis.links[p]) for p in queue]
    Dict("links" => links)
end

function serialize(path::String, link::Link)
    Dict("path" => path,
        "geometries" => serialize.(link))
end

function serialize(geomdata::GeometryData)
    params = serialize(geomdata.geometry)
    intrinsic_tform = intrinsic_transform(geomdata.geometry)
    Dict("name" => geomdata.name,
         "color" => serialize(geomdata.color),
         "pose" => serialize(compose(geomdata.transform, intrinsic_tform)),
         "parameters" => params)
end

intrinsic_transform(geom::AbstractGeometry) = Translation(center(geometry)...)

serialize(g::HyperRectangle) = Dict("type" => "box", "lengths" => widths(g))
serialize(g::HyperSphere) = Dict("type" => "sphere", "radius" => radius(g))
serialize(g::HyperEllipsoid) = Dict("type" => "ellipsoid", "radii" => radii(g))
serialize(g::HyperCylinder{3}) = Dict("type" => "cylinder",
                                      "length" => length(g),
                                      "radius" => radius(g))
serialize(g::HyperCube) = Dict("type" => "box", "lengths" => widths(g))


function command_data(vis::Visualizer, cmd::Draw, queue)
    commands = [serialize(fullpath(vis, p), vis.transforms[p]) for p in queue]
    Dict("commands" => commands)
end

function serialize(path::String, tform::Transformation)
    Dict("path" => path,
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
