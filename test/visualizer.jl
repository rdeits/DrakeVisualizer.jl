using ColorTypes: RGB
import GeometryTypes

proc = DrakeVisualizer.new_window()

@testset "open window" begin
    if Sys.isapple() || Sys.islinux()
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
        transforms = Array{Transformation}(undef, length(link_lengths))
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

    for x in product([range(-pi, stop=pi, length=11) for i in 1:length(link_lengths)]...)
        set_link_transforms(reverse(x))
    end
end

@testset "ellipsoid" begin
    ellipsoid = DrakeVisualizer.HyperEllipsoid(Point(1.,0,0.1), Vec(0.3, 0.2, 0.1))
    Visualizer(ellipsoid)
end

@testset "deprecated cylinder" begin
    cylinder = DrakeVisualizer.HyperCylinder(1.0, 0.5)
    Visualizer(cylinder)
end

@testset "cylinder" begin
    cylinder = Cylinder(Point(0., 0, 0), Point(0., 0, 1), 0.5)
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

@testset "pointcloud" begin
    vis = Visualizer()
    setgeometry!(vis[:pointcloud1],
                 PointCloud([[0., 0, 0], [1., 0, 0]]))
    setgeometry!(vis[:pointcloud2],
                 PointCloud([Point(0., 0.5, 0), Point(1., 0.5, 0)]))
    setgeometry!(vis[:pointcloud2_rgb],
                 PointCloud([Point(0., 1., 0), Point(1., 1., 0)],
                            Dict(:rgb=>[RGB(1., 1., 0.), RGB(1., 0., 1.)])))
end

const cat_mesh_path = joinpath(dirname(pathof(GeometryTypes)), "..", "test", "data", "cat.obj")

@testset "mesh files" begin
    vis = Visualizer()

    @testset "existing mesh" begin
        setgeometry!(vis[:mesh_file_existing], MeshFile(cat_mesh_path))
    end

    @testset "new mesh" begin
        f = x -> norm(x)^2
        iso_level = 0.5
        mesh = contour_mesh(f, [0.,0,0], [1.,1,1], iso_level)
        setgeometry!(vis[:mesh_file], MeshFile(mesh))
    end
end


@testset "destroy" begin
    vis = Visualizer(Cylinder(Point(0., 0, 0), Point(0., 0, 1), 2.0))
    delete!(vis)
end

@testset "addgeometry" begin
    vis = Visualizer()
    delete!(vis)
    setgeometry!(vis[:box1], HyperRectangle(Vec(0., 0, 0), Vec(1., 1, 1)))
    addgeometry!(vis[:box1], HyperSphere(Point(0., 0, 0), 1.0))
end

if haskey(ENV, "DISPLAY") && (get(ENV, "TRAVIS", "false") == "false" || ENV["TRAVIS_OS_NAME"] == "linux")
    @testset "script" begin
        expected_file = joinpath(@__DIR__, "test_script_success")
        isfile(expected_file) && rm(expected_file)
        @test !isfile(expected_file)
        scriptproc = DrakeVisualizer.new_window(script="testscript.py")
        result = timedwait(() -> isfile(expected_file), 10.)
        kill(scriptproc)
        result == :ok && rm(expected_file)
        @test result == :ok
    end
end
