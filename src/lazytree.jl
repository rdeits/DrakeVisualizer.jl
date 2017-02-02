module LazyTrees

import Base: getindex, setindex!, show, delete!

using CoordinateTransformations: AffineMap
using StaticArrays: SMatrix, SVector


type LazyTree{K, T}
    data::T
    children::Dict{K, LazyTree{K, T}}
    parent::Nullable{LazyTree{K, T}}

    LazyTree() = new(T(), Dict{K, LazyTree{K, T}}(), Nullable{LazyTree{K, T}}())
    LazyTree(data::T) = new(data, Dict{K, LazyTree{K, T}}(), Nullable{LazyTree{K, T}}())
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

function getindex{T <: LazyTree}(t::T, childname)
    if haskey(children(t), childname)
        children(t)[childname]
    else
        child = T()
        t[childname] = child
        child
    end
end

function setindex!{T <: LazyTree}(t::T, child::T, childname)
    t.children[childname] = child
    child.parent = t
end

function getindex{K}(t::LazyTree{K}, path::Union{Tuple{Vararg{K}}, AbstractVector{K}})
    for childname in path
        t = t[childname]
    end
    t
end

getindex{K}(t::LazyTree{K}, path::Vararg{K}) = getindex(t, path)

function setindex!{T <: LazyTree, N, K}(t::T, child::T, path::Union{NTuple{N, K}, AbstractVector{K}})
    t[path[1]][path[2:end]] = child
end

setindex!{T <: LazyTree}(t::T, child::T, path::Tuple) = t[path[1]] = child
setindex!{T <: LazyTree}(t::T, child::T, path::Vararg) = t[path] = child

delete!{K}(t::LazyTree{K}, childname::K) = delete!(t.children, childname)
function delete!(t::LazyTree, path::Union{Tuple, AbstractVector})
    if length(path) == 0
        for child in keys(children(t))
            delete!(t, child)
        end
    else
        for p in path[1:end-1]
            t = t[p]
        end
        delete!(t, path[end])
    end
end

function descendants{K}(t::LazyTree{K}, prefix=K[])
    result = Vector{Vector{K}}()
    for (childname, child) in children(t)
        childpath = vcat(prefix, childname)
        push!(result, childpath)
        append!(result, descendants(child, childpath))
    end
    result
end

end
