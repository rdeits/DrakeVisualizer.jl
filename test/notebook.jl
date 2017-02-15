using NBInclude

@testset "notebook" begin
    nbinclude(joinpath(dirname(dirname(@__FILE__)), "demo.ipynb"))
end
