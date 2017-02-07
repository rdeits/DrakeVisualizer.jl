module LazyTrees

import Base: getindex, setindex!, show, delete!

using CoordinateTransformations: AffineMap
using StaticArrays: SMatrix, SVector


type LazyTree{K, T}
    data::T
    children::Dict{K, LazyTree{K, T}}

    LazyTree() = new(T(), Dict{K, LazyTree{K, T}}())
    LazyTree(data::T) = new(data, Dict{K, LazyTree{K, T}}())
end

function show(io::IO, t::LazyTree)
    print(io, "LazyTree: $(t.data)")
    if length(t.children) > 0
        print(io, " with children: $(t.children)")
    end
end

children(t::LazyTree) = t.children
data(t::LazyTree) = t.data

function setindex!{T <: LazyTree}(t::T, child::T, childname)
    t.children[childname] = child
end
setindex!{K, T}(t::LazyTree{K, T}, childdata::T, childname::K) =
    setindex!(t, LazyTree{K, T}(childdata), childname)
setindex!{K, T}(t::LazyTree{K, T}, childdata, childname::K) =
    setindex!(t, convert(T, childdata), childname)

function setindex!{N, K}(t::LazyTree{K}, child, path::NTuple{N, K})
    for i in 1:(length(path) - 1)
        t = t[path[i]]
    end
    t[path[end]] = child
end

function setindex!{K}(t::LazyTree{K}, child, path::AbstractVector{K})
    for i in 1:(length(path) - 1)
        t = t[path[i]]
    end
    t[path[end]] = child
end

setindex!{K, T}(t::LazyTree{K, T}, child, path::Vararg{K}) = t[path] = child

function getindex{T <: LazyTree}(t::T, childname)
    if haskey(children(t), childname)
        children(t)[childname]
    else
        child = T()
        t[childname] = child
        child
    end
end

function getindex{K, N}(t::LazyTree{K}, path::NTuple{N, K})
    for childname in path
        t = t[childname]
    end
    t
end

function getindex{K}(t::LazyTree{K}, path::AbstractVector{K})
    for childname in path
        t = t[childname]
    end
    t
end

getindex{K}(t::LazyTree{K}, path::Vararg{K}) = getindex(t, path)

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
