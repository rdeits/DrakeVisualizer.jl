# GeometryTypes doesn't define an Ellipsoid type yet, so we'll make one ourselves!
type HyperEllipsoid{N, T} <: GeometryPrimitive{N, T}
    center::Point{N, T}
    radii::Vec{N, T}
end

origin{N, T}(geometry::HyperEllipsoid{N, T}) = geometry.center
radii{N, T}(geometry::HyperEllipsoid{N, T}) = geometry.radii
center(geometry::HyperEllipsoid) = origin(geometry)

type HyperCylinder{N, T} <: GeometryPrimitive{N, T}
    length::T # along last axis
    radius::T
    # origin is at center
end

length(geometry::HyperCylinder) = geometry.length
radius(geometry::HyperCylinder) = geometry.radius
origin{N, T}(geometry::HyperCylinder{N, T}) = zeros(SVector{N, T})
center(g::HyperCylinder) = origin(g)

center(geometry::HyperRectangle) = minimum(geometry) + 0.5 * widths(geometry)
center(geometry::HyperCube) = minimum(geometry) + 0.5 * widths(geometry)
center(geometry::HyperSphere) = origin(geometry)


immutable PointCloud{T, Point <: StaticArray{Tuple{3}, T}} <: AbstractGeometry{3, T}
    points::Vector{Point}
    channels::Dict{Symbol, Any}
end

PointCloud(points::AbstractVector{Point}) where {T, Point <: StaticArray{Tuple{3}, T}} =
    PointCloud{T, Point}(points, Dict{Symbol, Any}())

immutable Triad <: AbstractGeometry{3, Float64}
    scale::Float64
    tube::Bool

    Triad(scale=1.0, tube=true) = new(scale, tube)
end

immutable ArrowHead
    radius::Float64
    length::Float64

    ArrowHead(radius=0.05, length=radius) = new(radius, length)
end

immutable PolyLine{T, Point <: StaticArray{Tuple{3}, T}} <: AbstractGeometry{3, T}
    points::Vector{Point}
    radius::Float64
    closed::Bool
    start_head::Nullable{ArrowHead}
    end_head::Nullable{ArrowHead}
end

PolyLine(points::AbstractVector{Point}, radius, closed, start_head, end_head) where {T, Point <: StaticArray{Tuple{3}, T}} =
    PolyLine{T, Point}(points, radius, closed, start_head, end_head)

PolyLine(points::AbstractVector{V}, args...) where {T, V <: AbstractVector{T}} = 
    PolyLine(convert.(Point{3, Float64}, points), args...)

function PolyLine(points::AbstractVector;
    radius=0.0,
    closed=false,
    start_head=nothing,
    end_head=nothing)
    PolyLine(points, radius, closed,
             start_head,
             end_head)
end
