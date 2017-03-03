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


immutable PointCloud{Point} <: AbstractGeometry
    points::Vector{Point}
    channels::Dict{Symbol, Any}
end

PointCloud{Point}(points::AbstractVector{Point}) =
    PointCloud{Point}(points, Dict{Symbol, Any}())

immutable Triad <: AbstractGeometry
    scale::Float64
    tube::Bool

    Triad(scale=1.0, tube=true) = new(scale, tube)
end

immutable ArrowHead
    radius::Float64
    length::Float64

    ArrowHead(radius=0.05, length=radius) = new(radius, length)
end

immutable PolyLine{Point} <: AbstractGeometry
    points::Vector{Point}
    radius::Float64
    closed::Bool
    start_head::Nullable{ArrowHead}
    end_head::Nullable{ArrowHead}
end

PolyLine{Point}(points::AbstractVector{Point}, radius, closed, start_head, end_head) =
    PolyLine{Point}(points, radius, closed, start_head, end_head)

function PolyLine(points::AbstractVector;
    radius=0.0,
    closed=false,
    start_head=nothing,
    end_head=nothing)
    PolyLine(points, radius, closed,
             start_head,
             end_head)
end
