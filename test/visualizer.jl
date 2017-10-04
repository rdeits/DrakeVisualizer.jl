proc = DrakeVisualizer.new_window()

@testset "open window" begin
    if is_apple() || is_linux()
        # @test DrakeVisualizer.any_open_windows() # doesn't pass when running headless
        DrakeVisualizer.any_open_windows()
    end
end

@testset "robot_load" begin
    f = x -> norm(x)^2 - 1
    bounds = HyperRectangle(Vec(0.,0,0), Vec(1.,1,1))
    geom = contour_mesh(f, minimum(bounds), maximum(bounds))
    vis = Visualizer(geom)
end

@testset "geom_load" begin
    f = x -> norm(x)^2
    iso_level = 0.5
    geom = GeometryData(contour_mesh(f, [0.,0,0], [1.,1,1], iso_level))
    vis = Visualizer(geom)
end

@testset "robot_draw" begin
    vis = Visualizer()

    link_lengths = [1., 2, 3]
    for (i, l) in enumerate(link_lengths)
        geometry = HyperRectangle(Vec(0., -0.1, -0.1), Vec(l, 0.2, 0.2))
        setgeometry!(vis[Symbol("link_$i")], geometry)
    end

    function set_link_transforms(joint_angles)
        transforms = Array{Transformation}(length(link_lengths))
        transforms[1] = LinearMap(AngleAxis(joint_angles[1], 0, 0, 1.0))
        for i = 2:length(link_lengths)
            T = compose(Translation(link_lengths[i-1], 0, 0),
                        LinearMap(AngleAxis(joint_angles[i], 0, 0, 1.0))
                        )
            transforms[i] = compose(transforms[i-1], T)
        end
        batch(vis) do v
            for (i, tform) in enumerate(transforms)
                settransform!(vis[Symbol("link_$i")], tform)
            end
        end
    end

    for x in product([linspace(-pi, pi, 11) for i in 1:length(link_lengths)]...)
        set_link_transforms(reverse(x))
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

@testset "lines and arrows" begin
    vis = Visualizer()

    setgeometry!(vis[:arrow],
                 PolyLine([[0, 0, 0], [0, 0, 1], [1, 0, 1]], end_head=ArrowHead(0.05, 0.2), radius=0.02))

    setgeometry!(vis[:hairline], PolyLine([[0, 1, 0], [0, 1, 1]], radius=0))

    setgeometry!(vis[:double_arrow], PolyLine([[-1, 0, 0.5], [-0.5, 0, 0.5]],
                                              end_head=ArrowHead(),
                                              start_head=ArrowHead()))

    setgeometry!(vis[:basic_line], PolyLine([[0., 0, 0], [1., 0, 0]]))
end


@testset "destroy" begin
    vis = Visualizer(HyperCylinder{3, Float64}(1.0, 2.0))
    delete!(vis)
end

@testset "deprecations" begin
    vis = Visualizer()
    load!(vis, HyperCylinder{3, Float64}(1.0, 2.0))
    draw!(vis, IdentityTransformation())
end

@testset "addgeometry" begin
    vis = Visualizer()
    delete!(vis)
    setgeometry!(vis[:box1], HyperRectangle(Vec(0., 0, 0), Vec(1., 1, 1)))
    addgeometry!(vis[:box1], HyperSphere(Point(0., 0, 0), 1.0))
end

@testset "script" begin
    lcm = LCM()
    received_message = Ref(false)
    callback = function (channel, msg)
        received_message[] = true
        @test msg.format == "drake_visualizer_jl_test"
        @test msg.format_version_major == 0
        @test msg.format_version_minor == 1
    end
    subscribe(lcm, "DRAKE_VISUALIZER_JL_TEST", callback, DrakeVisualizer.Comms.CommsT)
    @async while true
        handle(lcm)
    end
    scriptproc = DrakeVisualizer.new_window(script="testscript.py")
    result = timedwait(() -> received_message[], 10.)
    kill(scriptproc)
    @test result == :ok
end
