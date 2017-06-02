# using PyPlot, DifferentialEquations, NLsolve, ForwardDiff
using NLsolve, ForwardDiff

## Define the variables
l0 = 5.0
l1 = 3.0
l2 = 2.0
l3 = 1.0

ϕ0 = 0.0
Θ0 = 0.0
ψ0 = 0.0

ϕdot = 0.5
Θdot = 0.2
ψdot = 0.3

t0 = 0.0
tf = 50.0
δt = 0.01
t = t0:δt:tf

function f(t)
  ϕ = ϕ0 + ϕdot*(t-t0)
  Θ = Θ0 + Θdot*(t-t0)
  ψ = ψ0 + ψdot*(t-t0)

  return ϕ, Θ, ψ
end

ph = zeros(length(t), 1)
th = zeros(length(t), 1)
ps = zeros(length(t), 1)

for i in 1:length(t)
  ph[i], th[i], ps[i] = f(t[i])
end

## Plotting
# fig=figure(1)
# clf()
# plot(t, ph, t, th, t, ps, linewidth=2.)
# xlim(t0, tf)
# xlabel(L"t \; [sec]", fontsize=15)
# ylabel(L"$\phi, \theta, \psi \; [rad]$", fontsize=15)


## Visualize the output
using DrakeVisualizer
using CoordinateTransformations
import GeometryTypes: HyperRectangle, Vec, HomogenousMesh
import ColorTypes: RGBA

# Launch the viewer application if it isn't running already:
DrakeVisualizer.any_open_windows() || DrakeVisualizer.new_window()


# First, we'll create a simple geometric object
box1 = HyperRectangle(Vec(-l0/2,-l0/2,-0.25), Vec(l0,l0,0.5))
box2 = HyperRectangle(Vec(-l1/2,-l1/2,-0.2), Vec(l1,l1,0.4))
box3 = HyperRectangle(Vec(-l2/2,-l2/2,-0.2), Vec(l2,l2,0.4))
box4 = HyperRectangle(Vec(-l3/2,-l3/2,-0.2), Vec(l3,l3,0.4))

boxConn11 = HyperRectangle(Vec(-l1/30,l1/2,-l1/30), Vec(l1/15,(l0-l1)/2,l1/15))
boxConn12 = HyperRectangle(Vec(-l1/30,-l1/2,-l1/30), Vec(l1/15,-(l0-l1)/2,l1/15))
boxConn21 = HyperRectangle(Vec(l2/2,-l2/30,-l2/30), Vec((l1-l2)/2,l2/15,l2/15))
boxConn22 = HyperRectangle(Vec(-l2/2,-l2/30,-l2/30), Vec(-(l1-l2)/2,l2/15,l2/15))
boxConn31 = HyperRectangle(Vec(-l3/30,l3/2,-l3/30), Vec(l3/15,(l2-l3)/2,l3/15))
boxConn32 = HyperRectangle(Vec(-l3/30,-l3/2,-l3/30), Vec(l3/15,-(l2-l3)/2,l3/15))

blue_box = GeometryData(box1, RGBA(0., 0., 1, 0.1))
green_box = GeometryData(box2, RGBA(0., 1., 0, 0.6))
red_box = GeometryData(box3, RGBA(1., 0., 0, 0.3))
violet_box = GeometryData(box4, RGBA(0.58, 0., 0.83, 0.4))

conn_box11 = GeometryData(boxConn11, RGBA(0., 1., 0, 0.6))
conn_box12 = GeometryData(boxConn12, RGBA(0., 1., 0, 0.6))
conn_box21 = GeometryData(boxConn21, RGBA(1., 0., 0, 0.3))
conn_box22 = GeometryData(boxConn22, RGBA(1., 0., 0, 0.3))
conn_box31 = GeometryData(boxConn31, RGBA(0.58, 0., 0.83, 0.4))
conn_box32 = GeometryData(boxConn32, RGBA(0.58, 0., 0.83, 0.4))

# First we make an empty visualizer
vis = Visualizer()

# We can access a particular path within the visualizer with indexing notation:
vis[:group1]

# We load geometries using the same path notation:
blue_box_vis = setgeometry!(vis[:group1][:bluebox], blue_box)
green_box_vis = setgeometry!(vis[:group1][:greenbox], green_box)
red_box_vis = setgeometry!(vis[:group1][:redbox], red_box)
violet_box_vis = setgeometry!(vis[:group1][:violetbox], violet_box)
conn_box11_vis = setgeometry!(vis[:group1][:conn_box11], conn_box11)
conn_box12_vis = setgeometry!(vis[:group1][:conn_box12], conn_box12)
conn_box21_vis = setgeometry!(vis[:group1][:conn_box21], conn_box21)
conn_box22_vis = setgeometry!(vis[:group1][:conn_box22], conn_box22)
conn_box31_vis = setgeometry!(vis[:group1][:conn_box31], conn_box31)
conn_box32_vis = setgeometry!(vis[:group1][:conn_box32], conn_box32)

# trans = Translation(0.0, l0, 0.0)
# rot = LinearMap(RotZ(th[1]))
# composed = trans ∘ rot

# settransform!(blue_box_vis, Translation(0.0, l0, 0.))
# settransform!(blue_box_vis, LinearMap(AngleAxis(th[1],0,0,1)))
# settransform!(blue_box_vis, composed)


# for i in 1:convert(Int64,floor(length(t)/10))
for i in 1:length(t)
  settransform!(blue_box_vis, Translation(-l0/2*0, -l0/2*0, 0.0) ∘ LinearMap(RotX(π/2)))
  settransform!(green_box_vis, Translation(-l1/2*0, -l1/2*0, 0.0) ∘ LinearMap(RotZ(ps[i])*RotZ(π/2)*RotX(π/2)))
  settransform!(conn_box11_vis, Translation(-l1/2*0, -l1/2*0, 0.0) ∘ LinearMap(RotZ(ps[i])*RotZ(π/2)*RotX(π/2)))
  settransform!(conn_box12_vis, Translation(-l1/2*0, -l1/2*0, 0.0) ∘ LinearMap(RotZ(ps[i])*RotZ(π/2)*RotX(π/2)))
  settransform!(red_box_vis, Translation(-l2/2*0, -l2/2*0, 0.0) ∘ LinearMap(RotZ(ps[i])*RotY(th[i])*RotY(π/2)*RotZ(π/2)*RotX(π/2)))
  settransform!(conn_box21_vis, Translation(-l2/2*0, -l2/2*0, 0.0) ∘ LinearMap(RotZ(ps[i])*RotY(th[i])*RotY(π/2)*RotZ(π/2)*RotX(π/2)))
  settransform!(conn_box22_vis, Translation(-l2/2*0, -l2/2*0, 0.0) ∘ LinearMap(RotZ(ps[i])*RotY(th[i])*RotY(π/2)*RotZ(π/2)*RotX(π/2)))
  settransform!(violet_box_vis, Translation(-l3/2*0, -l3/2*0, 0.0) ∘ LinearMap(RotZ(ps[i])*RotY(th[i])*RotX(ph[i])*RotX(π/2)*RotY(π/2)*RotZ(π/2)*RotX(π/2)))
  settransform!(conn_box31_vis, Translation(-l3/2*0, -l3/2*0, 0.0) ∘ LinearMap(RotZ(ps[i])*RotY(th[i])*RotX(ph[i])*RotX(π/2)*RotY(π/2)*RotZ(π/2)*RotX(π/2)))
  settransform!(conn_box32_vis, Translation(-l3/2*0, -l3/2*0, 0.0) ∘ LinearMap(RotZ(ps[i])*RotY(th[i])*RotX(ph[i])*RotX(π/2)*RotY(π/2)*RotZ(π/2)*RotX(π/2)))
  sleep(0.001)
end
