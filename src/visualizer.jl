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

type Visualizer
    lcm::LCM
    tree::LazyTree{Symbol, VisData}
    queue::CommandQueue
    publish_immediately::Bool

    function Visualizer(lcm::LCM=LCM())
        vis = new(lcm, LazyTree{Symbol, VisData}(), CommandQueue(), true)
        function handle_msg(channel, msg)
            onresponse(vis, msg)
        end
        subscribe(lcm, "DRAKE_VIEWER2_RESPONSE", handle_msg, drakevis[:viewer2_comms_t])
        vis
    end
end

function batch(func, vis::Visualizer)
    old_publish_flag = vis.publish_immediately
    try
        vis.publish_immediately = false
        func(vis)
        publish!(vis)
    finally
        vis.publish_immediately = old_publish_flag
    end
end

function load!(vis::Visualizer, path::AbstractVector)
    push!(vis.queue.load, path)
    draw!(vis, path)
end

function load!(vis::Visualizer, path::AbstractVector, geom)
    vis.tree[path].data.geometries = [geom]
    load!(vis, path)
end

load!(vis::Visualizer, path::AbstractVector, geom::AbstractGeometry) =
    load!(vis, path, GeometryData(geom))

function draw!(vis::Visualizer, path::AbstractVector)
    push!(vis.queue.draw, path)
    if vis.publish_immediately
        publish!(vis)
    end
end

function draw!(vis::Visualizer, path::AbstractVector, tform)
    vis.tree[path].data.transform = tform
    draw!(vis, path)
end

function delete!(vis::Visualizer, path::AbstractVector)
    delete!(vis.tree, path)
    push!(vis.queue.delete, path)
    if vis.publish_immediately
        publish!(vis)
    end
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
        for path in LazyTrees.descendants(vis.tree)
            load!(vis, path)
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
