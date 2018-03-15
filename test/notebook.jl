using NBInclude

import Interact
# Monkey-patch in a fix to make Interact work properly inside
# NBInclude.
Interact.update_view(args...) = nothing
@testset "notebook" begin
    nbinclude(joinpath(dirname(dirname(@__FILE__)), "demo.ipynb"))
end
