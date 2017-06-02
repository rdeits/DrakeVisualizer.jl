using PyPlot, DifferentialEquations, NLsolve, ForwardDiff

## Define the variables
l0 = 0.75;
l1 = 0.50;
l2 = 1.50;
l3 = 0.50;
y0 = 1.56;

# For inverse kinematics
# x = 0.5;
# q0 = [1.0; 1.2; -1.2; l2] # (Θ, ϕ, γ, s)

# For forward kinematics
Θ = 1.5647736827656682
q0 = [1.2; -1.2; l2; 0.40]   # (ϕ, γ, s, x)

fvec = zeros(4,1);
fjac = zeros(4,4);

## Inverse kinematics
# function finv!(q, fvec)
#   Θ = q[1]
#   ϕ = q[2]
#   γ = q[3]
#   s = q[4]
#
#   fvec[1] = l1*cos(Θ) - s*cos(ϕ)
#   fvec[2] = l1*sin(Θ) - s*sin(ϕ) + l0
#   fvec[3] = l2*cos(ϕ) + l3*cos(ϕ+γ) - x
#   fvec[4] = l2*sin(ϕ) + l3*sin(ϕ+γ) - y0
# end
#
# function ginv!(q, fjac)
#   Θ = q[1]
#   ϕ = q[2]
#   γ = q[3]
#   s = q[4]
#
#   fjac[1, 1] = -l1*sin(Θ)
#   fjac[1, 2] = s*sin(ϕ)
#   fjac[1, 3] = 0.0
#   fjac[1, 4] = -cos(ϕ)
#   fjac[2, 1] = l1*cos(Θ)
#   fjac[2, 2] = -s*cos(ϕ)
#   fjac[2, 3] = 0.0
#   fjac[2, 4] = -sin(ϕ)
#   fjac[3, 1] = 0.0
#   fjac[3, 2] = -l2*sin(ϕ)-l3*sin(ϕ+γ)
#   fjac[3, 3] = -l3*sin(ϕ+γ)
#   fjac[3, 4] = 0.0
#   fjac[4, 1] = 0.0
#   fjac[4, 2] = l2*cos(ϕ) + l3*cos(ϕ+γ)
#   fjac[4, 3] = l3*cos(ϕ+γ)
#   fjac[4, 4] = 0.0
# end
#
# # Solve position-level inverse kinematics
# sol = nlsolve(finv!, ginv!, q0)
#
#
# # Velocity-level inverse kinematics
# xdot = -0.35
# endEffectorVel = [0.0; 0.0; xdot; 0.0]
# J = zeros(4,4)
# ginv!(sol.zero, J)
# qdot = J \ endEffectorVel
#
# # Acceleration level inverse kinematics
# xddot = 2.14;
# endEffectorAcc = [0.0; 0.0; xddot; 0.0]
# Coriolis_acc = zeros(4,1)
# Coriolis_acc[1] = -l1*qdot[1]^2*cos(sol.zero[1]) + 2*qdot[4]*qdot[2]*sin(sol.zero[2]) + sol.zero[4]*qdot[2]^2*cos(sol.zero[2])
# Coriolis_acc[2] = -l1*qdot[1]^2*sin(sol.zero[1]) - 2*qdot[4]*qdot[2]*cos(sol.zero[2]) + sol.zero[4]*qdot[2]^2*sin(sol.zero[2])
# Coriolis_acc[3] = -l2*qdot[2]^2*cos(sol.zero[2]) - l3*(qdot[2] + qdot[3])^2*cos(sol.zero[2] + sol.zero[3])
# Coriolis_acc[4] = -l2*qdot[2]^2*sin(sol.zero[2]) - l3*(qdot[2] + qdot[3])^2*sin(sol.zero[2] + sol.zero[3])
# qddot = J \ (endEffectorAcc - Coriolis_acc)


## Forward Kinematics
function fforw!(q, fvec)
  ϕ = q[1]
  γ = q[2]
  s = q[3]
  x = q[4]

  fvec[1] = l1*cos(Θ) - s*cos(ϕ)
  fvec[2] = l1*sin(Θ) - s*sin(ϕ) + l0
  fvec[3] = l2*cos(ϕ) + l3*cos(ϕ+γ) - x
  fvec[4] = l2*sin(ϕ) + l3*sin(ϕ+γ) - y0
end

function gforw!(q, fjac)
  ϕ = q[1]
  γ = q[2]
  s = q[3]
  x = q[4]

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
end

# Solve position-level forward kinematics
sol = nlsolve(fforw!, gforw!, q0)


# Velocity-level forward kinematics
Θdot = 0.583506
jointVel = l1*Θdot*[sin(Θ); -cos(Θ); 0.0; 0.0]
J = zeros(4,4)
gforw!(sol.zero, J)
xdot = J \ jointVel

# Acceleration level forward kinematics
Θddot = -3.58477;
jointAcc = l1*Θddot*[sin(Θ); -cos(Θ); 0.0; 0.0]
Coriolis_acc = zeros(4,1)
Coriolis_acc[1] = -l1*Θdot^2*cos(Θ) + 2*xdot[3]*xdot[1]*sin(sol.zero[1]) + sol.zero[3]*xdot[1]^2*cos(sol.zero[1])
Coriolis_acc[2] = -l1*Θdot^2*sin(Θ) - 2*xdot[3]*xdot[1]*cos(sol.zero[1]) + sol.zero[3]*xdot[1]^2*sin(sol.zero[1])
Coriolis_acc[3] = -l2*xdot[1]^2*cos(sol.zero[1]) - l3*(xdot[1] + xdot[2])^2*cos(sol.zero[1] + sol.zero[2])
Coriolis_acc[4] = -l2*xdot[1]^2*sin(sol.zero[1]) - l3*(xdot[1] + xdot[2])^2*sin(sol.zero[1] + sol.zero[2])
xddot = J \ (jointAcc - Coriolis_acc)


## Plotting

# fig=figure(1)
# clf()
# subplot(2,2,1)
# plot(tt, x[:,1], linewidth=2.)
# xlim(t0, tf)
# # ylim(-3.7, 0.02)
# xlabel(L"t \; [sec]", fontsize=15)
# ylabel(L"$\theta \; [rad]$", fontsize=15)
# # axis("tight")
# subplot(2,2,2)
# plot(tt, x[:,2], linewidth=2.)
# xlabel(L"t \; [sec]", fontsize=15)
# ylabel(L"\dot{\theta} \; [\frac{rad}{s}]", fontsize=15)
# xlim(t0, tf)
# # ylim(-0.78, 1.2)
# subplot(2,2,(3,4))
# plot(tt, torque, linewidth=2.)
# xlabel(L"t \; [sec]", fontsize=15)
# ylabel(L"\tau \; [Nm]", fontsize=15)
# xlim(t0, tf)
# # ylim(-0.77, 0.77)
# tight_layout()
#
# savefig("quick_return_slider_crank.eps", format="eps")

## Visualize the output

# using DrakeVisualizer
# using CoordinateTransformations
# import GeometryTypes: HyperRectangle, Vec, HomogenousMesh
# import ColorTypes: RGBA
#
# # Launch the viewer application if it isn't running already:
# DrakeVisualizer.any_open_windows() || DrakeVisualizer.new_window()
#
# # First, we'll create a simple geometric object
# box = HyperRectangle(Vec(0.,0,0), Vec(1.,1,-5))
# blue_box = GeometryData(box, RGBA(0., 0., 1, 0.5))
#
# # First we make an empty visualizer
# vis = Visualizer()
#
# # We can access a particular path within the visualizer with indexing notation:
# vis[:group1]
#
# # We load geometries using the same path notation:
# blue_box_vis = setgeometry!(vis[:group1][:bluebox], blue_box)
#
# for i in 1:length(tt)
#   settransform!(blue_box_vis, LinearMap(AngleAxis(x[i,1]-pi,0,1,0)))
#   sleep(0.01)
# end
