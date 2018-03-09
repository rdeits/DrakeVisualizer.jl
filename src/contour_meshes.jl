function contour_mesh(f::Function,
                      bounds::HyperRectangle,
                      isosurface_value::Number=0.0,
                      resolution::Real=maximum(widths(bounds))/20.0)
    
    # Extract an isosurface from a vector-input function in 3 dimensions and
    # return it as a mesh.
    window_width = widths(bounds)
    sdf = SignedDistanceField(x -> f(SVector{3, Float64}(x[1], x[2], x[3])),
                              bounds,
                              resolution)
    mesh = HomogenousMesh(sdf, MarchingTetrahedra(isosurface_value))
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
