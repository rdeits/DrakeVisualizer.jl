using Base.Dates: Millisecond
import Base: getindex, convert
using .LazyTrees: LazyTree, data, children

mutable struct GeometryData{G <: Union{AbstractGeometry, AbstractMesh}, C <: Colorant, T <: Transformation}
    geometry::G
    color::C
    transform::T
end

GeometryData(g, c::Colorant) = GeometryData(g, c, IdentityTransformation())
GeometryData(g, t::Transformation) = GeometryData(g, RGBA(1, 0, 0, 0.5), t)

convert(::Type{G}, g::Union{AbstractGeometry, AbstractMesh}) where {G <: GeometryData} = GeometryData(g, RGBA(1, 0, 0, 0.5))

mutable struct VisData
    transform::Transformation
    geometries::Vector{GeometryData}
end

VisData() = VisData(IdentityTransformation(), GeometryData[])

const Path = Vector{Symbol}

struct CommandQueue
    delete::Set{Path}
    setgeometry::Set{Path}
    settransform::Set{Path}

    CommandQueue() = new(Set{Path}(), Set{Path}(), Set{Path}())
end

isempty(queue::CommandQueue) = isempty(queue.delete) && isempty(queue.setgeometry) && isempty(queue.settransform)

function empty!(queue::CommandQueue)
    empty!(queue.delete)
    empty!(queue.setgeometry)
    empty!(queue.settransform)
end

const VisTree = LazyTree{Symbol, VisData}

mutable struct CoreVisualizer
    window::Window
    tree::VisTree
    queue::CommandQueue
    publish_immediately::Bool

    function CoreVisualizer(window::Window)
        new(window, VisTree(), CommandQueue(), true)
    end
end

function setgeometry!(vis::CoreVisualizer, path::AbstractVector)
    push!(vis.queue.setgeometry, path)
    settransform!(vis, path)
end

function setgeometry!(vis::CoreVisualizer, path::AbstractVector, geoms::AbstractVector)
    vis.tree[path].data.geometries = geoms
    setgeometry!(vis, path)
end

function addgeometry!(vis::CoreVisualizer, path::AbstractVector, geoms::AbstractVector)
    append!(vis.tree[path].data.geometries, geoms)
    setgeometry!(vis, path)
end

setgeometry!(vis::CoreVisualizer, path::AbstractVector, geom) = setgeometry!(vis, path, [geom])
addgeometry!(vis::CoreVisualizer, path::AbstractVector, geom) = addgeometry!(vis, path, [geom])

setgeometry!(vis::CoreVisualizer, path::AbstractVector, geom::AbstractGeometry) =
    setgeometry!(vis, path, GeometryData(geom))
addgeometry!(vis::CoreVisualizer, path::AbstractVector, geom::AbstractGeometry) =
    addgeometry!(vis, path, GeometryData(geom))

function settransform!(vis::CoreVisualizer, path::AbstractVector)
    push!(vis.queue.settransform, path)
    if vis.publish_immediately
        publish!(vis)
    end
end

function settransform!(vis::CoreVisualizer, path::AbstractVector, tform)
    vis.tree[path].data.transform = tform
    settransform!(vis, path)
end

function delete!(vis::CoreVisualizer, path::AbstractVector)
    delete!(vis.tree, path)
    push!(vis.queue.delete, path)
    if vis.publish_immediately
        publish!(vis)
    end
end

const ZMQ_RECEIVE_TIMEOUT_S = 5

function publish!(core::CoreVisualizer)
    if !isempty(core.queue)
        data = serialize(core, core.queue)

        c = Channel{Bool}(1)
        @async begin
            ZMQ.send(core.window.socket, data)
            ZMQ.recv(core.window.socket)
            put!(c, true)
        end
        @async begin
            sleep(ZMQ_RECEIVE_TIMEOUT_S)
            put!(c, false)
        end
        success = take!(c)
        if !success
            close(core.window.socket)
            close(core.window.context)
            error("No response received from the visualizer. The ZMQ socket may no longer function, so it has been closed. You may want to close and re-launch the Visualizer.")
        end
        empty!(core.queue)
    end
end

function republish!(vis::CoreVisualizer)
    for path in LazyTrees.descendants(vis.tree)
        push!(vis.queue.setgeometry, path)
        push!(vis.queue.settransform, path)
    end
    push!(vis.queue.setgeometry, [])
    push!(vis.queue.settransform, [])
    publish!(vis)
end

struct Visualizer
    core::CoreVisualizer
    path::Vector{Symbol}

    function Visualizer(win=Window())
        new(CoreVisualizer(win), Symbol[])
    end
    Visualizer(core::CoreVisualizer, path::AbstractVector) = new(core, path)
end

show(io::IO, vis::Visualizer) = print(io, "Visualizer with path prefix $(vis.path) attached to $(vis.core.window)")

function setgeometry!(vis::Visualizer)
    setgeometry!(vis.core, vis.path)
    vis
end

function setgeometry!(vis::Visualizer, geom)
    setgeometry!(vis.core, vis.path, geom)
    vis
end

function addgeometry!(vis::Visualizer, geom)
    addgeometry!(vis.core, vis.path, geom)
    vis
end

settransform!(vis::Visualizer) = settransform!(vis.core, vis.path)
settransform!(vis::Visualizer, transform) = settransform!(vis.core, vis.path, transform)
delete!(vis::Visualizer) = delete!(vis.core, vis.path)
publish!(vis::Visualizer) = publish!(vis.core)

getindex(vis::Visualizer, path::Symbol) = Visualizer(vis.core, vcat(vis.path, path))
getindex(vis::Visualizer, path::AbstractVector) = Visualizer(vis.core, vcat(vis.path, path))

function batch(func, vis::Visualizer)
    old_publish_flag = vis.core.publish_immediately
    try
        vis.core.publish_immediately = false
        func(vis)
        if old_publish_flag
            publish!(vis)
        end
    finally
        vis.core.publish_immediately = old_publish_flag
    end
end

republish!(vis::Visualizer) = republish!(vis.core)

# Old-style visualizer interface
function Visualizer(geom::GeometryData, args...; kwargs...)
    vis = Visualizer(args...; kwargs...)[:anonymous]
    setgeometry!(vis, geom)
    vis
end

Visualizer(geom::Union{AbstractGeometry, AbstractMesh}, args...; kwargs...) = Visualizer(GeometryData(geom), args...; kwargs...)

Base.close(vis::Visualizer) = close(vis.core.window)
Base.open(vis::Visualizer) = open(vis.core.window)
reconnect(vis::Visualizer) = reconnect(vis.core.window)
