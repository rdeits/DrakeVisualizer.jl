# GeometryTypes doesn't define an Ellipsoid type yet, so we'll make one ourselves!
mutable struct HyperEllipsoid{N, T} <: GeometryPrimitive{N, T}
    center::Point{N, T}
    radii::Vec{N, T}
end

origin(geometry::HyperEllipsoid{N, T}) where {N, T} = geometry.center
radii(geometry::HyperEllipsoid{N, T}) where {N, T} = geometry.radii
center(geometry::HyperEllipsoid) = origin(geometry)

center(geometry::HyperRectangle) = minimum(geometry) + 0.5 * widths(geometry)
center(geometry::HyperCube) = minimum(geometry) + 0.5 * widths(geometry)
center(geometry::HyperSphere) = origin(geometry)
center(geometry::Cylinder) = origin(geometry) + geometry.extremity / 2


struct PointCloud{T, Point <: StaticArray{Tuple{3}, T}} <: AbstractGeometry{3, T}
    points::Vector{Point}
    channels::Dict{Symbol, Any}
end

function PointCloud(points::AbstractVector{Point},
                    channels::Dict=Dict{Symbol, Any}()) where {T, Point <: StaticArray{Tuple{3}, T}}
    PointCloud{T, Point}(points, channels)
end

function PointCloud(points::AbstractVector{V},
                    channels::Dict=Dict{Symbol, Any}()) where {T, V <: AbstractVector{T}}
    PointCloud{T, Point{3, T}}(points, channels)
end

struct Triad <: AbstractGeometry{3, Float64}
    scale::Float64
    tube::Bool

    Triad(scale=1.0, tube=true) = new(scale, tube)
end

struct ArrowHead
    radius::Float64
    length::Float64

    ArrowHead(radius=0.05, length=radius) = new(radius, length)
end

struct PolyLine{T, Point <: StaticArray{Tuple{3}, T}} <: AbstractGeometry{3, T}
    points::Vector{Point}
    radius::Float64
    closed::Bool
    start_head::Union{ArrowHead, Nothing}
    end_head::Union{ArrowHead, Nothing}
end

PolyLine(points::AbstractVector{Point}, radius, closed, start_head, end_head) where {T, Point <: StaticArray{Tuple{3}, T}} =
    PolyLine{T, Point}(points, radius, closed, start_head, end_head)

PolyLine(points::AbstractVector{V}, args...) where {T, V <: AbstractVector{T}} =
    PolyLine(convert.(Point{3, Float64}, points), args...)

function PolyLine(points::AbstractVector{V};
    radius=0.0,
    closed=false,
    start_head=nothing,
    end_head=nothing) where {T, V <: AbstractVector{T}}
    PolyLine(points, radius, closed,
             start_head,
             end_head)
end

"""
A MeshFile is a wrapper around a mesh that indicates that the mesh should be
transmitted to the viewer by saving its data to a file and sending that file's
path to the viewer. This can avoid issues with very large meshes (which cannot
otherwise be sent over LCM), but will not work if the viewer is not running
locally.
"""
struct MeshFile <: AbstractGeometry{3, Float64}
    filename::String
end

function MeshFile(mesh::AbstractMesh)
    path = string(tempname(), ".ply")
    save(path, mesh)
    MeshFile(path)
end
