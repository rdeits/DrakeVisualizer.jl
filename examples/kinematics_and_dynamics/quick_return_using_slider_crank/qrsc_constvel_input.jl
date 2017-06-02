# Check https://me-mechanicalengineering.com/quick-return-mechanisms/
# for explanation of this mechanism

# using PyPlot, DifferentialEquations, NLsolve, ForwardDiff
using NLsolve, ForwardDiff

## Define the variables
l0 = 0.75;
l1 = 0.50;
l2 = 1.50;
l3 = 0.50;
y0 = 1.56;

# Initial condition
Θ0 = 1.5647736827656682
Θ = Θ0
q0 = [1.2; -1.2; l2; 0.40]   # (ϕ, γ, s, x)

# Constant input velocity
Θdot = 0.5;

fvec = zeros(4,1);
fjac = zeros(4,4);


## Forward Kinematics
function fforw!(q, fvec)
  ϕ = q[1]
  γ = q[2]
  s = q[3]
  x = q[4]
  # println(Θ)

  # fvec = zeros(4,1)
  fvec[1] = l1*cos(Θ) - s*cos(ϕ)
  fvec[2] = l1*sin(Θ) - s*sin(ϕ) + l0
  fvec[3] = l2*cos(ϕ) + l3*cos(ϕ+γ) - x
  fvec[4] = l2*sin(ϕ) + l3*sin(ϕ+γ) - y0

  # return fvec
end

function gforw!(q, fjac)
  ϕ = q[1]
  γ = q[2]
  s = q[3]
  x = q[4]

  # fjac = zeros(4,4)

  fjac[1, 1] = s*sin(ϕ)
  fjac[1, 2] = 0.0
  fjac[1, 3] = -cos(ϕ)
  fjac[1, 4] = 0.0
  fjac[2, 1] = -s*cos(ϕ)
  fjac[2, 2] = 0.0
  fjac[2, 3] = -sin(ϕ)
  fjac[2, 4] = 0.0
  fjac[3, 1] = -l2*sin(ϕ)-l3*sin(ϕ+γ)
  fjac[3, 2] = -l3*sin(ϕ+γ)
  fjac[3, 3] = 0.0
  fjac[3, 4] = -1.0
  fjac[4, 1] = l2*cos(ϕ) + l3*cos(ϕ+γ)
  fjac[4, 2] = l3*cos(ϕ+γ)
  fjac[4, 3] = 0.0
  fjac[4, 4] = 0.0

  # return fjac
end


# Solve position-level forward kinematics
sol = nlsolve(fforw!, gforw!, q0)
q0 = sol.zero

# Velocity-level forward kinematics
jointVel = l1*Θdot*[sin(Θ); -cos(Θ); 0.0; 0.0]
J = zeros(4,4)
gforw!(sol.zero, J)
xdot = J \ jointVel



## Solve the differential equation
t0 = 0.0;
tf = 50.0;

function f(t,x)
    global Θ = Θ0 + Θdot*(t-t0)
    sol = nlsolve(fforw!, gforw!, q0)

    # Velocity-level forward kinematics
    jointVel = l1*Θdot*[sin(Θ); -cos(Θ); 0.0; 0.0]
    gforw!(sol.zero, J)
    xdot = J \ jointVel

    dx = xdot[4]
    return dx
end

function ff(t)
  global Θ = Θ0 + Θdot*(t-t0)

  sol = nlsolve(fforw!, gforw!, q0)
  # sol = nlsolve(not_in_place(fforw), not_in_place(gforw), q0)
  # sol = nlsolve(not_in_place(fforw(,Θ)), q0)
  return sol.zero[4]
end


# Solve using Runge-Kutta
x0 = q0[4]
tspan = (t0,tf)
# prob = ODEProblem(clf(τ), x0, tspan)
prob = ODEProblem(f, x0, tspan)
solDE = DifferentialEquations.solve(prob, RK4(), dt = 0.01)
x = solDE.u
tt = solDE.t


q0 = [1.2; -1.2; l2; 0.40]
xd = zeros(length(tt),1)
xx = zeros(length(tt),1)
th = zeros(length(tt), 1)
ph = zeros(length(tt), 1)
gamma = zeros(length(tt), 1)
s = zeros(length(tt), 1)
for i in 1:length(tt)
  xd[i] = f(tt[i], x[i])
  xx[i] = ff(tt[i])
  th[i] = Θ0 + Θdot*(tt[i]-t0)
  sol = nlsolve(fforw!, gforw!, q0)
  ph[i] = sol.zero[1]
  gamma[i] = sol.zero[2]
  s[i] = sol.zero[3]
  q0 = [ph[i]; gamma[i]; s[i]; x[i]]
end

## Plotting
# fig=figure(1)
# clf()
# plot(tt, xd[:,1], linewidth=2.)
# xlim(t0, tf)
# xlabel(L"t \; [sec]", fontsize=15)
# ylabel(L"$\dot{x} \; [\frac{rad}{s}]$", fontsize=15)

# figure(2)
# clf()
# plot(tt, [0; diff(xx)]/0.01, linewidth=2.)
# xlim(t0, tf)
# xlabel(L"t \; [sec]", fontsize=15)
# ylabel(L"$\dot{x} \; [\frac{rad}{s}]$", fontsize=15)




## Visualize the output
using DrakeVisualizer
using CoordinateTransformations
import GeometryTypes: HyperRectangle, Vec, HomogenousMesh
import ColorTypes: RGBA

# Launch the viewer application if it isn't running already:
DrakeVisualizer.any_open_windows() || DrakeVisualizer.new_window()

toollength = 0.5
toolwidth = 0.25
sliderlength = 0.25

# First, we'll create a simple geometric object
box1 = HyperRectangle(Vec(0.,-0.05,0), Vec(l1,0.1,0.1))
box2 = HyperRectangle(Vec(0.,-0.05,0), Vec(l2,0.1,0.1))
box3 = HyperRectangle(Vec(0.,-0.05,0), Vec(l3,0.1,0.1))
box4 = HyperRectangle(Vec(0.,-0.05,0), Vec(toollength,toolwidth,0.25))
box5 = HyperRectangle(Vec(0.,-0.05,0), Vec(sliderlength,0.1,0.1))
blue_box = GeometryData(box1, RGBA(0., 0., 1, 0.5))
green_box = GeometryData(box2, RGBA(0., 1., 0, 0.5))
red_box = GeometryData(box3, RGBA(1., 0., 0, 0.5))
tool = GeometryData(box4, RGBA(0.6, 0.4, 0.7, 0.4))
slider = GeometryData(box5, RGBA(0.8, 0.6, 0.5, 0.9))

# First we make an empty visualizer
vis = Visualizer()

# We can access a particular path within the visualizer with indexing notation:
vis[:group1]

# We load geometries using the same path notation:
blue_box_vis = setgeometry!(vis[:group1][:bluebox], blue_box)
green_box_vis = setgeometry!(vis[:group1][:greenbox], green_box)
red_box_vis = setgeometry!(vis[:group1][:redbox], red_box)
tool_vis = setgeometry!(vis[:group1][:tool], tool)
slider_vis = setgeometry!(vis[:group1][:slider], slider)

# trans = Translation(0.0, l0, 0.0)
# rot = LinearMap(RotZ(th[1]))
# composed = trans ∘ rot

# settransform!(blue_box_vis, Translation(0.0, l0, 0.))
# settransform!(blue_box_vis, LinearMap(AngleAxis(th[1],0,0,1)))
# settransform!(blue_box_vis, composed)
for i in 1:length(tt)
  settransform!(blue_box_vis, Translation(0.0, l0, 0.0) ∘ LinearMap(RotZ(th[i])))
  settransform!(green_box_vis, Translation(0.0, 0.0, 0.0) ∘ LinearMap(RotZ(ph[i])))
  settransform!(red_box_vis, Translation(l2*cos(ph[i]), l2*sin(ph[i]), 0.0) ∘ LinearMap(RotZ(ph[i]+gamma[i])))
  settransform!(tool_vis, Translation(l2*cos(ph[i])+l3*cos(ph[i]+gamma[i])-toollength/2, l2*sin(ph[i])+l3*sin(ph[i]+gamma[i])-toolwidth/2, 0.0) ∘ LinearMap(RotZ(0.0)))
  settransform!(slider_vis, Translation((s[i]-sliderlength/2)*cos(ph[i]), (s[i]-sliderlength/2)*sin(ph[i]), 0.0) ∘ LinearMap(RotZ(ph[i])))
  sleep(0.001)
end
