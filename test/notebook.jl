using NBInclude

@testset "notebook" begin
    @nbinclude(joinpath(@__DIR__, "..", "demo.ipynb"))
end
