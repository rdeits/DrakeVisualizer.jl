using DrakeVisualizer
using GeometryTypes
using AffineTransforms
using Meshing
using Base.Test
import Iterators: product

proc = DrakeVisualizer.launch()

function test_robot_load()
    sdf = SignedDistanceField(x -> norm(x)^2 - 1, HyperRectangle(Vec(0.,0,0), Vec(1.,1,1)));
    mesh = HomogenousMesh(sdf, 0.0)
    geom = GeometryData(mesh, tformeye(3))
    robot = convert(Robot, geom)
    vis = Visualizer(robot, 1)
end
test_robot_load()

function test_link_list_load()
    links = Link[]
    levels = [0.5; 1]
    for i = 1:2
        sdf = SignedDistanceField(x -> norm(x)^2 - levels[i], HyperRectangle(Vec(-1.,-1,-1), Vec(2.,2,2)));
        mesh = HomogenousMesh(sdf, 0.0)
        geom = GeometryData(mesh, tformeye(3))
        push!(links, Link(geom))
    end
    vis = Visualizer(links, 1)
end
test_link_list_load()

function test_geom_load()
    sdf = SignedDistanceField(x -> norm(x)^2 - 0.5, HyperRectangle(Vec(0.,0,0), Vec(1.,1,1)));
    mesh = HomogenousMesh(sdf, 0.0)
    geom = GeometryData(mesh, tformeye(3))
    vis = Visualizer(geom)
end
test_geom_load()


function test_robot_draw()
    links = Link[]
    link_lengths = [1.; 2; 3]
    for (i, l) in enumerate(link_lengths)
        geometry = HyperRectangle(Vec(0., -0.1, -0.1), Vec(l, 0.2, 0.2))
        geometry_data = GeometryData(geometry)
        push!(links, Link([geometry_data], "link$(i)"))
    end


    function link_origins(joint_angles)
        transforms = Array{AffineTransform}(length(link_lengths))
        transforms[1] = tformrotate([0;0;1], joint_angles[1])
        for i = 2:length(link_lengths)
            transforms[i] = transforms[i-1] * tformtranslate([link_lengths[i-1]; 0; 0]) * tformrotate([0.;0;1], joint_angles[i])
        end
        transforms
    end

    robot = Robot(links)
    model = Visualizer(robot)

    for x in product([linspace(-pi, pi, 11) for i in 1:length(link_lengths)]...)
        origins = link_origins(reverse(x))
        draw(model, origins)
    end
end
test_robot_draw()

let
    ellipsoid = DrakeVisualizer.HyperEllipsoid(Point(1.,0,0.1), Vec(0.3, 0.2, 0.1))
    Visualizer(ellipsoid)
end

let
    cylinder = DrakeVisualizer.HyperCylinder{3, Float64}(1.0, 0.5)
    Visualizer(cylinder)
end

let
    demo_file = "../demo.ipynb"
    run(`jupyter nbconvert --to notebook --execute $(demo_file) --output $(demo_file)`)
end

kill(proc)
