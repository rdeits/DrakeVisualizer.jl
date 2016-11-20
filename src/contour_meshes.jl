function contour_mesh(f::Function,
                      bounds::HyperRectangle,
                      isosurface_value::Number=0.0,
                      resolution::Real=maximum(widths(bounds))/20.0)
    # Extract an isosurface from a vector-input function in 3 dimensions and
    # return it as a mesh. This function mostly exists to work around bugs in
    # Meshing.jl: specifically its weird handling of non-zero isosurfaces
    # and its incorrect mesh point scaling:
    # https://github.com/JuliaGeometry/GeometryTypes.jl/issues/49
    window_width = widths(bounds)
    sdf = SignedDistanceField(x -> f(SVector{3, Float64}(x[1], x[2], x[3])) - isosurface_value,
                              bounds,
                              resolution)

    # We've already accounted for the isosurface value in the construction of
    # the SDF, so we set the iso value here to 0.
    mesh = HomogenousMesh(sdf, 0.0)

    # Rescale the mesh points and then construct a new mesh using the rescaled
    # points and the original faces.
    lb = minimum(bounds)
    rescaled_points = Point{3,Float64}[Vec(v-1) ./ (Vec(size(sdf))-1) .* window_width .+ lb for v in vertices(mesh)]
    HomogenousMesh(rescaled_points, mesh.faces)
end

contour_mesh(f::Function,
             lower_bound::Vec,
             upper_bound::Vec,
             isosurface_value::Number=0.0,
             resolution::Real=maximum(upper_bound - lower_bound)/20.0) = (
    contour_mesh(f,
                 HyperRectangle(lower_bound, upper_bound - lower_bound),
                 isosurface_value,
                 resolution))

contour_mesh(f::Function,
             lower_bound::AbstractVector,
             upper_bound::AbstractVector,
             isosurface_value::Number=0.0,
             resolution::Real=maximum(upper_bound - lower_bound)/20.0) = (
    contour_mesh(f,
                 Vec{3, Float64}(lower_bound[1], lower_bound[2], lower_bound[3]),
                 Vec{3, Float64}(upper_bound[1], upper_bound[2], upper_bound[3]),
                 isosurface_value,
                 resolution))
