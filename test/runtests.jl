using DrakeVisualizer
using GeometryTypes
using AffineTransforms
using Meshing
using Base.Test

function test_robot_draw()
    sdf = SignedDistanceField(x -> norm(x)^2 - 1, HyperRectangle(Vec(0.,0,0), Vec(1.,1,1)));
    mesh = HomogenousMesh(sdf, 0.0)
    geom = GeometryData(mesh, tformeye(3))
    robot = convert(Robot, geom)
    vis = Visualizer()
    load(vis, robot, 1)
end
test_robot_draw()

function test_link_list_draw()
    links = Link[]
    levels = [0.5; 1]
    for i = 1:2
        sdf = SignedDistanceField(x -> norm(x)^2 - levels[i], HyperRectangle(Vec(-1.,-1,-1), Vec(2.,2,2)));
        mesh = HomogenousMesh(sdf, 0.0)
        geom = GeometryData(mesh, tformeye(3))
        push!(links, Link(geom))
    end
    vis = Visualizer()
    load(vis, links, 1)
end
test_link_list_draw()

function test_geom_draw()
    sdf = SignedDistanceField(x -> norm(x)^2 - 0.5, HyperRectangle(Vec(0.,0,0), Vec(1.,1,1)));
    mesh = HomogenousMesh(sdf, 0.0)
    geom = GeometryData(mesh, tformeye(3))
    vis = Visualizer()
    load(vis, geom, 1)
end
test_geom_draw()
