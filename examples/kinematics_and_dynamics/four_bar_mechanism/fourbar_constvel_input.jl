# using PyPlot, DifferentialEquations, NLsolve, ForwardDiff
using NLsolve, ForwardDiff

## Define the variables
l0 = 1.10;
l1 = 0.50;
l2 = 1.20;
l3 = 0.75;

# Initial condition
Θ0 = π/2+0.1;
Θ = Θ0
q0 = [-π/2-0.1; π/2+0.1]   # (γ, ϕ)

# Constant input velocity
Θdot = 0.5;

fvec = zeros(2,1);
fjac = zeros(2,2);


## Forward Kinematics
function fforw!(q, fvec)
  γ = q[1]
  ϕ = q[2]
  # println(Θ)

  # fvec = zeros(2,1)
  fvec[1] = l1*cos(Θ) + l2*cos(Θ+γ) - l3*cos(ϕ) - l0
  fvec[2] = l1*sin(Θ) + l2*sin(Θ+γ) - l3*sin(ϕ)

  # return fvec
end


function gforw!(q, fjac)
  γ = q[1]
  ϕ = q[2]

  # fjac = zeros(4,4)

  fjac[1, 1] = -l2*sin(Θ+γ)
  fjac[1, 2] = l3*sin(ϕ)
  fjac[2, 1] = l2*cos(Θ+γ)
  fjac[2, 2] = -l3*cos(ϕ)

  # return fjac
end


# Solve position-level forward kinematics
sol = nlsolve(fforw!, gforw!, q0)
q0 = sol.zero

# Velocity-level forward kinematics
jointVel = Θdot*[l1*sin(Θ)+l2*sin(Θ+γ); -l1*cos(Θ)-l2*cos(Θ+γ)]
J = zeros(2,2)
gforw!(sol.zero, J)
xdot = J \ jointVel



## Solve for γ and ϕ for t ∈ [0., 50.]
t0 = 0.0;
tf = 50.0;
δt = 0.01;


function f(t)
  global Θ = Θ0 + Θdot*(t-t0)
  sol = nlsolve(fforw!, gforw!, q0)
  global q0 = sol.zero

  return sol.zero
end


t = t0:δt:tf
th = zeros(length(t), 1)
ph = zeros(length(t), 1)
gamma = zeros(length(t), 1)
for i in 1:length(t)
  th[i] = Θ0 + Θdot*(t[i]-t0)
  gamma[i] = f(t[i])[1]
  ph[i] = f(t[i])[2]
end
gammadot = [xdot[1]; diff(gamma)/δt]
phdot = [xdot[2]; diff(ph)/δt]


## Plotting
# fig=figure(1)
# clf()
# plot(t, th, t, gamma, t, ph, linewidth=2.)
# xlim(t0, tf)
# xlabel(L"t \; [sec]", fontsize=15)
# ylabel(L"$\theta, \gamma, \phi \; [rad]$", fontsize=15)

# figure(2)
# clf()
# plot(t, Θdot*ones(length(t),1), t, gammadot, t, phdot, linewidth=2.)
# xlim(t0, tf)
# xlabel(L"t \; [sec]", fontsize=15)
# ylabel(L"$\dot{\theta}, \dot{\gamma}, \dot{\phi} \; [\frac{rad}{s}]$", fontsize=15)




## Visualize the output
using DrakeVisualizer
using CoordinateTransformations
import GeometryTypes: HyperRectangle, Vec, HomogenousMesh
import ColorTypes: RGBA

# Launch the viewer application if it isn't running already:
DrakeVisualizer.any_open_windows() || DrakeVisualizer.new_window()

# First, we'll create a simple geometric object
box1 = HyperRectangle(Vec(0.,-0.05,0), Vec(l1,0.1,0.1))
box2 = HyperRectangle(Vec(0.,-0.05,0), Vec(l2,0.1,0.1))
box3 = HyperRectangle(Vec(0.,-0.05,0), Vec(l3,0.1,0.1))
# box4 = HyperRectangle(Vec(0.,-0.05,0), Vec(toollength,toolwidth,0.25))
# box5 = HyperRectangle(Vec(0.,-0.05,0), Vec(sliderlength,0.1,0.1))
blue_box = GeometryData(box1, RGBA(0., 0., 1, 0.5))
green_box = GeometryData(box2, RGBA(0., 1., 0, 0.5))
red_box = GeometryData(box3, RGBA(1., 0., 0, 0.5))
# tool = GeometryData(box4, RGBA(0.6, 0.4, 0.7, 0.4))
# slider = GeometryData(box5, RGBA(0.8, 0.6, 0.5, 0.9))

# First we make an empty visualizer
vis = Visualizer()

# We can access a particular path within the visualizer with indexing notation:
vis[:group1]

# We load geometries using the same path notation:
blue_box_vis = setgeometry!(vis[:group1][:bluebox], blue_box)
green_box_vis = setgeometry!(vis[:group1][:greenbox], green_box)
red_box_vis = setgeometry!(vis[:group1][:redbox], red_box)
# tool_vis = setgeometry!(vis[:group1][:tool], tool)
# slider_vis = setgeometry!(vis[:group1][:slider], slider)

# trans = Translation(0.0, l0, 0.0)
# rot = LinearMap(RotZ(th[1]))
# composed = trans ∘ rot

# settransform!(blue_box_vis, Translation(0.0, l0, 0.))
# settransform!(blue_box_vis, LinearMap(AngleAxis(th[1],0,0,1)))
# settransform!(blue_box_vis, composed)
for i in 1:length(t)
  settransform!(blue_box_vis, Translation(0.0, 0.0, 0.0) ∘ LinearMap(RotZ(th[i])))
  settransform!(green_box_vis, Translation(l1*cos(th[i]), l1*sin(th[i]), 0.0) ∘ LinearMap(RotZ(th[i]+gamma[i])))
  settransform!(red_box_vis, Translation(l0, 0.0, 0.0) ∘ LinearMap(RotZ(ph[i])))
  sleep(0.001)
end
