# GeometryTypes doesn't define an Ellipsoid type yet, so we'll make one ourselves!
type HyperEllipsoid{N, T} <: GeometryPrimitive{N, T}
    center::Point{N, T}
    radii::Vec{N, T}
end

origin{N, T}(geometry::HyperEllipsoid{N, T}) = geometry.center
radii{N, T}(geometry::HyperEllipsoid{N, T}) = geometry.radii

type HyperCylinder{N, T} <: GeometryTypes.GeometryPrimitive{N, T}
    length::T # along last axis
    radius::T
    # origin is at center
end

length(geometry::HyperCylinder) = geometry.length
radius(geometry::HyperCylinder) = geometry.radius
center(geometry::HyperRectangle) = minimum(geometry) + 0.5 * widths(geometry)
center(geometry::HyperCube) = minimum(geometry) + 0.5 * widths(geometry)
center(geometry::HyperSphere) = origin(geometry)
center(geometry::HyperEllipsoid) = origin(geometry)
