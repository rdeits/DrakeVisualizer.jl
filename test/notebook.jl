using NBInclude

@testset "notebook" begin
    @nbinclude(joinpath(@__DIR__, "..", "notebooks", "demo.ipynb"))
end
