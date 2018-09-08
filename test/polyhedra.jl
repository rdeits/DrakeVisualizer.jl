# A custom geometry type to test that we can render arbitrary primitives
# by decomposing them into simple meshes. This replaces the previous test
# which did the same thing using a Polyhedron from Polyhedra.jl. The Polyhedra
# test was removed because it required too many external dependencies just to
# verify a simple interface.
struct CustomGeometry <: GeometryPrimitive{3, Float64}
end

GeometryTypes.isdecomposable(::Type{<:Face}, ::CustomGeometry) = true
function GeometryTypes.decompose(::Type{F}, c::CustomGeometry) where {F <: Face}
    [convert(F, Face(1,2,3))]
end
GeometryTypes.isdecomposable(::Type{<:Point}, ::CustomGeometry) = true
function GeometryTypes.decompose(::Type{P}, c::CustomGeometry) where {P <: Point}
    convert(Vector{P}, [Point(0., 0, 0), Point(0., 1, 0), Point(0., 0, 1)])
end

@testset "Custom decomposable geometry" begin
    Visualizer(CustomGeometry())
end
