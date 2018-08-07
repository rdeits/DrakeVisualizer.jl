module LazyTrees

import Base: getindex, setindex!, show, delete!, convert

using CoordinateTransformations: AffineMap
using StaticArrays: SMatrix, SVector


mutable struct LazyTree{K, T}
    data::T
    children::Dict{K, LazyTree{K, T}}

    LazyTree{K, T}() where {K, T} = new{K, T}(T(), Dict{K, LazyTree{K, T}}())
    LazyTree{K, T}(data::T) where {K, T} = new{K, T}(data, Dict{K, LazyTree{K, T}}())
end

function show(io::IO, t::LazyTree)
    print(io, "LazyTree: $(t.data)")
    if length(t.children) > 0
        print(io, " with children: $(t.children)")
    end
end

children(t::LazyTree) = t.children
data(t::LazyTree) = t.data

convert(::Type{LazyTree{K, T}}, x::T) where {K, T} = LazyTree{K, T}(x)
convert(::Type{<:LazyTree{K, T}}, x) where {K, T} = LazyTree{K, T}(convert(T, x))
convert(::Type{LT}, x::LT) where {LT<:LazyTree} = x

setindex!(t::LazyTree, child, childname) = t.children[childname] = child

function setindex!(t::LazyTree{K}, child, path::Union{<:AbstractVector{K}, <:(NTuple{N, K} where N)}) where {K}
    for i in 1:(length(path) - 1)
        t = t[path[i]]
    end
    t[path[end]] = child
end

setindex!(t::LazyTree, child, path...) = setindex!(t, child, path)


function getindex(t::T, childname) where T <: LazyTree
    if haskey(children(t), childname)
        children(t)[childname]
    else
        child = T()
        t[childname] = child
        child
    end
end

function getindex(t::LazyTree{K}, path::Union{<:AbstractVector{K}, <:(NTuple{N, K} where N)}) where {K}
    for childname in path
        t = t[childname]
    end
    t
end

getindex(t::LazyTree{K}, path...) where {K} = getindex(t, path)

delete!(t::LazyTree{K}, childname::K) where {K} = delete!(t.children, childname)
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

function descendants(t::LazyTree{K}, prefix=K[]) where K
    result = Vector{Vector{K}}()
    for (childname, child) in children(t)
        childpath = vcat(prefix, childname)
        push!(result, childpath)
        append!(result, descendants(child, childpath))
    end
    result
end

end
