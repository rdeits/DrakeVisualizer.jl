# using PyPlot, DifferentialEquations, NLsolve, ForwardDiff
using NLsolve, ForwardDiff

## Define the variables
l0 = 5.0
l1 = 1.0
l2 = 5.2
l3 = 2.5
l4 = 4.0

# Initial condition
Θ10 = π/3;
Θ1 = Θ10
q0 = [-π/6; 3*π/4; l0+1.0; l4]   # (Θ2, Θ3, s1, s2)

# Constant input velocity
Θ1dot = -0.4;

fvec = zeros(4,1);
fjac = zeros(4,4);


## Forward Kinematics
function fforw!(q, fvec)
  Θ2 = q[1]
  Θ3 = q[2]
  s1 = q[3]
  s2 = q[4]

  fvec[1] = l1*cos(Θ1) + l2*cos(Θ1+Θ2) - l3*cos(Θ3) - s1
  fvec[2] = l1*sin(Θ1) + l2*sin(Θ1+Θ2) - l3*sin(Θ3)
  fvec[3] = l0 - s1 - l4*cos(Θ3)
  fvec[4] = s2 - l4*sin(Θ3)
end


function gforw!(q, fjac)
  Θ2 = q[1]
  Θ3 = q[2]
  s1 = q[3]
  s2 = q[4]

  fjac[1, 1] = -l2*sin(Θ1+Θ2)
  fjac[1, 2] = l3*sin(Θ3)
  fjac[1, 3] = -1.
  fjac[1, 4] = 0.
  fjac[2, 1] = l2*cos(Θ1+Θ2)
  fjac[2, 2] = -l3*cos(Θ3)
  fjac[2, 3] = 0.
  fjac[2, 4] = 0.
  fjac[3, 1] = 0.
  fjac[3, 2] = l4*sin(Θ3)
  fjac[3, 3] = -1.
  fjac[3, 4] = 0.
  fjac[4, 1] = 0.
  fjac[4, 2] = -l4*cos(Θ3)
  fjac[4, 3] = 0.
  fjac[4, 4] = 1.
end

# Just for testing if the derivative is correct with the ForwardDiff package.
myflag = false
numberOfTestPoints = 100;
testPoints = rand(4, numberOfTestPoints)
y = zeros(4,1)
for j in 1:numberOfTestPoints
  gforw!(testPoints[:,j], fjac)
  if !(fjac ≈ ForwardDiff.jacobian((y,x)->fforw!(x,y), y, testPoints[:,j]))
    myflag = true
    break
  end
end
if myflag
  println("The Jacobian is erroneous.")
else
  println("The Jacobian is correct.")
end


# Solve position-level forward kinematics
sol = nlsolve(fforw!, gforw!, q0)
q0 = sol.zero
Θ2 = q0[1]
Θ3 = q0[2]
s1 = q0[3]
s2 = q0[4]

# Velocity-level forward kinematics
jointVel = Θ1dot*[l1*sin(Θ1)+l2*sin(Θ1+Θ2); -l1*cos(Θ1)-l2*cos(Θ1+Θ2); 0; 0]
J = zeros(4,4)
gforw!(sol.zero, J)
xdot = J \ jointVel



## Solve for γ and ϕ for t ∈ [0., 50.]
t0 = 0.0;
tf = 50.0;
δt = 0.01;


function f(t)
  global Θ1 = Θ10 + Θ1dot*(t-t0)
  sol = nlsolve(fforw!, gforw!, q0)
  global q0 = sol.zero

  return sol.zero
end


t = t0:δt:tf
th1 = zeros(length(t), 1)
th2 = zeros(length(t), 1)
th3 = zeros(length(t), 1)
s1 = zeros(length(t), 1)
s2 = zeros(length(t), 1)
for i in 1:length(t)
  th1[i] = Θ10 + Θ1dot*(t[i]-t0)
  th2[i] = f(t[i])[1]
  th3[i] = f(t[i])[2]
  s1[i] = f(t[i])[3]
  s2[i] = f(t[i])[4]
end
th2dot = [xdot[1]; diff(th2)/δt]
th3dot = [xdot[2]; diff(th3)/δt]
s1dot = [xdot[3]; diff(s1)/δt]
s2dot = [xdot[4]; diff(s2)/δt]


## Plotting
# fig=figure(1)
# clf()
# plot(t, th1, t, th2, t, th3, t, s1, t, s2, linewidth=2.)
# xlim(t0, tf)
# xlabel(L"t \; [sec]", fontsize=15)
# ylabel(L"$\theta_1, \theta_2, \theta_3, s_1, s_2 \; [rad]$", fontsize=15)

# figure(2)
# clf()
# plot(t, Θ1dot*ones(length(t),1), t, th2dot, t, th3dot, t, s1dot, t, s2dot, linewidth=2.)
# xlim(t0, tf)
# xlabel(L"t \; [sec]", fontsize=15)
# ylabel(L"$\dot{\theta}_1, \dot{\theta}_2, \dot{\theta}_3, \dot{s}_1, \dot{s}_2 \; [\frac{rad}{s}]$", fontsize=15)


## Visualize the output
using DrakeVisualizer
using CoordinateTransformations
import GeometryTypes: HyperRectangle, Vec, HomogenousMesh
import ColorTypes: RGBA

# Launch the viewer application if it isn't running already:
DrakeVisualizer.any_open_windows() || DrakeVisualizer.new_window()

toollength = 0.5
toolwidth = 0.25

# First, we'll create a simple geometric object
box1 = HyperRectangle(Vec(0.,-0.05,0), Vec(l1,0.1,0.1))
box2 = HyperRectangle(Vec(0.,-0.05,0), Vec(l2,0.1,0.1))
box3 = HyperRectangle(Vec(0.,-0.05,0), Vec(l4,0.1,0.1))
box4 = HyperRectangle(Vec(0.,-0.05,0), Vec(toollength,toolwidth,0.25))
box5 = HyperRectangle(Vec(0.,-0.05,0), Vec(toolwidth,toollength,0.25))

blue_box = GeometryData(box1, RGBA(0., 0., 1, 0.5))
green_box = GeometryData(box2, RGBA(0., 1., 0, 0.5))
red_box = GeometryData(box3, RGBA(1., 0., 0, 0.5))
toolHor = GeometryData(box4, RGBA(0.6, 0.4, 0.7, 0.4))
toolVer = GeometryData(box5, RGBA(0.8, 0.6, 0.5, 0.4))


# First we make an empty visualizer
vis = Visualizer()

# We can access a particular path within the visualizer with indexing notation:
vis[:group1]

# We load geometries using the same path notation:
blue_box_vis = setgeometry!(vis[:group1][:bluebox], blue_box)
green_box_vis = setgeometry!(vis[:group1][:greenbox], green_box)
red_box_vis = setgeometry!(vis[:group1][:redbox], red_box)
toolHor_vis = setgeometry!(vis[:group1][:toolhor], toolHor)
toolVer_vis = setgeometry!(vis[:group1][:toolver], toolVer)

# trans = Translation(0.0, l0, 0.0)
# rot = LinearMap(RotZ(th[1]))
# composed = trans ∘ rot

# settransform!(blue_box_vis, Translation(0.0, l0, 0.))
# settransform!(blue_box_vis, LinearMap(AngleAxis(th[1],0,0,1)))
# settransform!(blue_box_vis, composed)
for i in 1:length(t)
  settransform!(blue_box_vis, Translation(0.0, 0.0, 0.0) ∘ LinearMap(RotZ(th1[i])))
  settransform!(green_box_vis, Translation(l1*cos(th1[i]), l1*sin(th1[i]), 0.0) ∘ LinearMap(RotZ(th1[i]+th2[i])))
  settransform!(red_box_vis, Translation(s1[i], 0.0, 0.0) ∘ LinearMap(RotZ(th3[i])))
  settransform!(toolHor_vis, Translation(s1[i]-toollength/2, -toolwidth/2, 0.0) ∘ LinearMap(RotZ(0.0)))
  settransform!(toolVer_vis, Translation(l0-toolwidth/2, s2[i]-toollength/2, 0.0) ∘ LinearMap(RotZ(0.0)))
  sleep(0.001)
end
