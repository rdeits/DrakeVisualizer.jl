using .LazyTrees: LazyTree, data, children
import Base: getindex, convert

type GeometryData{G <: AbstractGeometry, C <: Colorant}
    geometry::G
    color::C
end

convert{G <: GeometryData}(::Type{G}, g::Union{AbstractGeometry, AbstractMesh}) = GeometryData(g, RGBA{Float64}(1, 0, 0, 0.5))

type VisData
    transform::Transformation
    geometries::Vector{GeometryData}
end

VisData() = VisData(IdentityTransformation(), GeometryData[])

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

typealias VisTree LazyTree{Symbol, VisData}

type CoreVisualizer
    lcm::LCM
    tree::VisTree
    queue::CommandQueue
    publish_immediately::Bool

    function CoreVisualizer(lcm::LCM=LCM())
        vis = new(lcm, VisTree(), CommandQueue(), true)
        function handle_msg(channel, msg)
            try
                onresponse(vis, msg)
            catch e
                warn("""
An error ocurred while handling the viewer response:
    error: $e
    response: $msg
""")
            end
        end
        subscribe(lcm, "DIRECTOR_TREE_VIEWER_RESPONSE", handle_msg, drakevis[:viewer2_comms_t])
        vis
    end
end

function load!(vis::CoreVisualizer, path::AbstractVector)
    push!(vis.queue.load, path)
    draw!(vis, path)
end

function load!(vis::CoreVisualizer, path::AbstractVector, geoms::AbstractVector)
    vis.tree[path].data.geometries = geoms
    load!(vis, path)
end

load!(vis::CoreVisualizer, path::AbstractVector, geom) = load!(vis, path, [geom])

load!(vis::CoreVisualizer, path::AbstractVector, geom::AbstractGeometry) =
    load!(vis, path, GeometryData(geom))

function draw!(vis::CoreVisualizer, path::AbstractVector)
    push!(vis.queue.draw, path)
    if vis.publish_immediately
        publish!(vis)
    end
end

function draw!(vis::CoreVisualizer, path::AbstractVector, tform)
    vis.tree[path].data.transform = tform
    draw!(vis, path)
end

function delete!(vis::CoreVisualizer, path::AbstractVector)
    delete!(vis.tree, path)
    push!(vis.queue.delete, path)
    if vis.publish_immediately
        publish!(vis)
    end
end

function publish!(vis::CoreVisualizer)
    if !isempty(vis.queue)
        data = serialize(vis, vis.queue)
        msg = to_lcm(data)
        publish(vis.lcm, "DIRECTOR_TREE_VIEWER_REQUEST", msg)

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

function onresponse(vis::CoreVisualizer, msg)
    data = JSON.parse(msg[:data])
    if data["status"] == 0
        empty!(vis.queue)
    elseif data["status"] == 1
        for path in LazyTrees.descendants(vis.tree)
            load!(vis, path)
        end
    else
        error("unhandled: $data")
    end
end

function to_lcm(data::Associative)
    msg = drakevis[:viewer2_comms_t]()
    msg[:utime] = data["utime"]
    msg[:format] = "treeviewer_json"
    msg[:format_version_major] = 1
    msg[:format_version_minor] = 0
    encoded = JSON.json(data)
    msg[:num_bytes] = length(encoded)
    msg[:data] = encoded
    msg
end

immutable Visualizer
    core::CoreVisualizer
    path::Vector{Symbol}

    Visualizer(lcm::LCM=LCM()) = new(CoreVisualizer(lcm), Symbol[])
    Visualizer(core::CoreVisualizer, path::AbstractVector) = new(core, path)
end

show(io::IO, vis::Visualizer) = print(io, "Visualizer with path prefix $(vis.path) using LCM $(vis.core.lcm)")

function load!(vis::Visualizer)
    load!(vis.core, vis.path)
    vis
end

function load!(vis::Visualizer, geom)
    load!(vis.core, vis.path, geom)
    vis
end

draw!(vis::Visualizer) = draw!(vis.core, vis.path)
draw!(vis::Visualizer, transform) = draw!(vis.core, vis.path, transform)
delete!(vis::Visualizer) = delete!(vis.core, vis.path)
publish!(vis::Visualizer) = publish!(vis.core)

getindex(vis::Visualizer, path::Symbol) = Visualizer(vis.core, vcat(vis.path, path))
getindex(vis::Visualizer, path::AbstractVector) = Visualizer(vis.core, vcat(vis.path, path))

function batch(func, vis::Visualizer)
    old_publish_flag = vis.core.publish_immediately
    try
        vis.core.publish_immediately = false
        func(vis)
        publish!(vis)
    finally
        vis.core.publish_immediately = old_publish_flag
    end
end

# Old-style visualizer interface
function Visualizer(geom::GeometryData)
    vis = Visualizer()
    load!(vis[:body1], geom)
    vis
end

Visualizer(geom::AbstractGeometry) = Visualizer(GeometryData(geom))

function Visualizer(geoms::AbstractVector)
    vis = Visualizer()
    batch(vis) do v
        for (i, geom) in enumerate(geoms)
            load!(v[Symbol("body$i")], geom)
        end
    end
    vis
end

draw(vis::Visualizer, transform::Transformation) = draw!(vis[:body1], transform)

function draw(vis::Visualizer, transforms::AbstractVector)
    batch(vis) do v
        for (i, tform) in enumerate(transforms)
            draw!(v[Symbol("body$i")], tform)
        end
    end
end
