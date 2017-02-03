proc = DrakeVisualizer.new_window()

try
    @testset "open window" begin
        if is_apple() || is_linux()
            # @test DrakeVisualizer.any_open_windows() # doesn't pass when running headless
            DrakeVisualizer.any_open_windows()
        end
    end

    @testset "robot_load" begin
        f = x -> norm(x)^2
        bounds = HyperRectangle(Vec(0.,0,0), Vec(1.,1,1))
        geom = contour_mesh(f, minimum(bounds), maximum(bounds))
        vis = Visualizer(geom)
    end

    @testset "link_list_load" begin
        links = Vector{Vector{GeometryData}}()
        levels = [0.5; 1]
        for i = 1:2
            f = x -> norm(x)^2
            geom = contour_mesh(f, Vec(0.,0,0), Vec(1.,1,1), levels[i])
            push!(links, [geom])
        end
        vis = Visualizer(links)
    end

    @testset "geom_load" begin
        f = x -> norm(x)^2
        iso_level = 0.5
        geom = GeometryData(contour_mesh(f, [0.,0,0], [1.,1,1], iso_level))
        vis = Visualizer(geom)
    end


    @testset "robot_draw" begin
        links = Vector{Vector{GeometryData}}()
        link_lengths = [1.; 2; 3]
        for (i, l) in enumerate(link_lengths)
            geometry = HyperRectangle(Vec(0., -0.1, -0.1), Vec(l, 0.2, 0.2))
            geometry_data = GeometryData(geometry)
            push!(links, [geometry_data])
        end


        function link_origins(joint_angles)
            transforms = Array{Transformation}(length(link_lengths))
            transforms[1] = LinearMap(AngleAxis(joint_angles[1], 0, 0, 1.0))
            for i = 2:length(link_lengths)
                T = compose(Translation(link_lengths[i-1], 0, 0),
                            LinearMap(AngleAxis(joint_angles[i], 0, 0, 1.0))
                            )
                transforms[i] = compose(transforms[i-1], T)
            end
            transforms
        end

        model = Visualizer(links)

        for x in product([linspace(-pi, pi, 11) for i in 1:length(link_lengths)]...)
            origins = link_origins(reverse(x))
            draw(model, origins)
        end
    end

    @testset "ellipsoid" begin
        ellipsoid = DrakeVisualizer.HyperEllipsoid(Point(1.,0,0.1), Vec(0.3, 0.2, 0.1))
        Visualizer(ellipsoid)
    end

    @testset "cylinder" begin
        cylinder = DrakeVisualizer.HyperCylinder{3, Float64}(1.0, 0.5)
        Visualizer(cylinder)
    end

    @testset "destroy" begin
        vis = Visualizer(HyperCylinder{3, Float64}(1.0, 2.0))
        delete!(vis)
    end

    @testset "demo_notebook" begin
        jupyter = IJulia.jupyter
        demo_file = "../demo.ipynb"
        tmpfile = joinpath(dirname(@__FILE__), "demo.generated")
        run(`$jupyter nbconvert $demo_file --to script --output $tmpfile`)
        include("$tmpfile.jl")
    end
finally
    kill(proc)
end
