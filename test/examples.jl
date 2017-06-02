using NBInclude

include(joinpath(dirname(@__FILE__), "..", "examples", "kinematics_and_dynamics", "runtests.jl"))

@testset "notebook" begin
    nbinclude(joinpath(dirname(dirname(@__FILE__)), "examples", "demo.ipynb"))
end


