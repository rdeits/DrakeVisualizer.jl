module Trees

import Base: getindex, setindex!, show, delete!

using CoordinateTransformations: AffineMap
using StaticArrays: SMatrix, SVector


type LazyTree{T}
    data::T
    children::Dict{String, LazyTree{T}}
    parent::Nullable{LazyTree{T}}

    LazyTree() = new(T(), Dict{String, LazyTree{T}}(), Nullable{LazyTree{T}}())
    LazyTree(data::T) = new(data, Dict{String, LazyTree{T}}(), Nullable{LazyTree{T}}())
end

function show(io::IO, t::LazyTree)
    print(io, "LazyTree: $(t.data)")
    if length(t.children) > 0
        print(io, " with children: $(t.children)")
    end
end

children(t::LazyTree) = t.children
data(t::LazyTree) = get(t.data)
parent(t::LazyTree) = get(t.parent)

function getindex{T <: LazyTree}(t::T, childname::String)
    if haskey(children(t), childname)
        children(t)[childname]
    else
        child = T()
        t[childname] = child
        child
    end
end

function setindex!{T <: LazyTree}(t::T, child::T, childname::String)
    t.children[childname] = child
    child.parent = t
end

function getindex(t::LazyTree, path::Union{Tuple{Vararg{String}}, AbstractVector{String}})
    for childname in path
        t = t[childname]
    end
    t
end

getindex(t::LazyTree, path::Vararg{String}) = getindex(t, path)

function setindex!{T <: LazyTree, N}(t::T, child::T, path::Union{NTuple{N, String}, AbstractVector{String}})
    t[path[1]][path[2:end]] = child
end

setindex!{T <: LazyTree}(t::T, child::T, path::Tuple{String}) = t[path[1]] = child
setindex!{T <: LazyTree}(t::T, child::T, path::Vararg{String}) = t[path] = child

delete!(t::LazyTree, childname::String) = delete!(t.children, childname)
function delete!(t::LazyTree, path::Union{NTuple, AbstractVector})
    for p in path[1:end-1]
        t = t[p]
    end
    delete!(t, path[end])
end

function descendants(t::LazyTree, prefix=String[])
    result = Vector{Vector{String}}()
    for child in keys(children(t))
        childpath = vcat(prefix, child)
        push!(result, childpath)
        append!(result, descendants(children(t)[child], childpath))
    end
    result
end

end
