# using PyPlot, DifferentialEquations, NLsolve, ForwardDiff
using NLsolve, ForwardDiff

## Define the variables
l0x = 1.10
l0y = 0.10
l1 = 0.50
l2 = 0.90 + 0.365
l3 = 0.75 + 0.165
l4 = 0.60

# Initial condition
Θ10 = π/2+0.2;
ϕ10 = π/2-0.2;
Θ1 = Θ10
ϕ1 = ϕ10
q0 = [-π/2-0.2; π/2-0.2]   # (Θ2, ϕ2)

# Constant input velocity
Θ1dot = -0.1;
ϕ1dot = 0.2;

fvec = zeros(2,1);
fjac = zeros(2,2);


## Forward Kinematics
function fforw!(q, fvec)
  Θ2 = q[1]
  ϕ2 = q[2]
  # println(Θ)

  # fvec = zeros(2,1)
  fvec[1] = l1*cos(Θ1) + l2*cos(Θ1+Θ2) - l3*cos(ϕ1+ϕ2) - l4*cos(ϕ1) - l0x
  fvec[2] = l1*sin(Θ1) + l2*sin(Θ1+Θ2) - l3*sin(ϕ1+ϕ2) - l4*sin(ϕ1) + l0y

  # return fvec
end


function gforw!(q, fjac)
  Θ2 = q[1]
  ϕ2 = q[2]

  # fjac = zeros(4,4)

  fjac[1, 1] = -l2*sin(Θ1+Θ2)
  fjac[1, 2] = l3*sin(ϕ1+ϕ2)
  fjac[2, 1] = l2*cos(Θ1+Θ2)
  fjac[2, 2] = -l3*cos(ϕ1+ϕ2)

  # return fjac
end

# Just for testing if the derivative is correct with the ForwardDiff package.
myflag = false
numberOfTestPoints = 100;
testPoints = rand(2, numberOfTestPoints)
y = zeros(2,1)
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
ϕ2 = q0[2]

# Velocity-level forward kinematics
jointVel = Θ1dot*[l1*sin(Θ1)+l2*sin(Θ1+Θ2); -l1*cos(Θ1)-l2*cos(Θ1+Θ2)] +
           ϕ1dot*[-l3*sin(ϕ1+ϕ2)-l4*sin(ϕ1); l3*cos(ϕ1+ϕ2)+l4*cos(ϕ1)]
J = zeros(2,2)
gforw!(sol.zero, J)
xdot = J \ jointVel



## Solve for γ and ϕ for t ∈ [0., 50.]
t0 = 0.0;
tf = 70.0;
δt = 0.01;


function f(t)
  global Θ1 = Θ10 + Θ1dot*(t-t0)
  global ϕ1 = ϕ10 + ϕ1dot*(t-t0)
  sol = nlsolve(fforw!, gforw!, q0)
  global q0 = sol.zero

  return sol.zero
end


t = t0:δt:tf
th1 = zeros(length(t), 1)
th2 = zeros(length(t), 1)
ph1 = zeros(length(t), 1)
ph2 = zeros(length(t), 1)
for i in 1:length(t)
  th1[i] = Θ10 + Θ1dot*(t[i]-t0)
  ph1[i] = ϕ10 + ϕ1dot*(t[i]-t0)
  th2[i] = f(t[i])[1]
  ph2[i] = f(t[i])[2]
end
th2dot = [xdot[1]; diff(th2)/δt]
ph2dot = [xdot[2]; diff(ph2)/δt]


## Plotting
# fig=figure(1)
# clf()
# plot(t, th1, t, th2, t, ph1, t, ph2, linewidth=2.)
# xlim(t0, tf)
# xlabel(L"t \; [sec]", fontsize=15)
# ylabel(L"$\theta_1, \theta_2, \phi_1, \phi_2 \; [rad]$", fontsize=15)

# figure(2)
# clf()
# plot(t, Θ1dot*ones(length(t),1), t, th2dot, t, ϕ1dot*ones(length(t),1), t, ph2dot, linewidth=2.)
# xlim(t0, tf)
# xlabel(L"t \; [sec]", fontsize=15)
# ylabel(L"$\dot{\theta}_1, \dot{\theta}_2, \dot{\phi}_1, \dot{\phi}_2 \; [\frac{rad}{s}]$", fontsize=15)




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
box4 = HyperRectangle(Vec(0.,-0.05,0), Vec(l4,0.1,0.1))

blue_box = GeometryData(box1, RGBA(0., 0., 1, 0.5))
green_box = GeometryData(box2, RGBA(0., 1., 0, 0.5))
red_box = GeometryData(box3, RGBA(1., 0., 0, 0.5))
violet_box = GeometryData(box4, RGBA(0.58, 0., 0.83, 0.5))


# First we make an empty visualizer
vis = Visualizer()

# We can access a particular path within the visualizer with indexing notation:
vis[:group1]

# We load geometries using the same path notation:
blue_box_vis = setgeometry!(vis[:group1][:bluebox], blue_box)
green_box_vis = setgeometry!(vis[:group1][:greenbox], green_box)
red_box_vis = setgeometry!(vis[:group1][:redbox], red_box)
violet_box_vis = setgeometry!(vis[:group1][:violetbox], violet_box)

# trans = Translation(0.0, l0, 0.0)
# rot = LinearMap(RotZ(th[1]))
# composed = trans ∘ rot

# settransform!(blue_box_vis, Translation(0.0, l0, 0.))
# settransform!(blue_box_vis, LinearMap(AngleAxis(th[1],0,0,1)))
# settransform!(blue_box_vis, composed)
for i in 1:length(t)
  settransform!(blue_box_vis, Translation(0.0, 0.0, 0.0) ∘ LinearMap(RotZ(th1[i])))
  settransform!(green_box_vis, Translation(l1*cos(th1[i]), l1*sin(th1[i]), 0.0) ∘ LinearMap(RotZ(th1[i]+th2[i])))
  settransform!(red_box_vis, Translation(l0x+l4*cos(ph1[i]), -l0y+l4*sin(ph1[i]), 0.0) ∘ LinearMap(RotZ(ph1[i]+ph2[i])))
  settransform!(violet_box_vis, Translation(l0x, -l0y, 0.0) ∘ LinearMap(RotZ(ph1[i])))
  sleep(0.001)
end
