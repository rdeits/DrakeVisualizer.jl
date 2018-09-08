using DrakeVisualizer.LazyTrees: LazyTree, children, data, delete!, descendants

struct MaybeInt
    x::Union{Int, Nothing}
end

MaybeInt() = MaybeInt(nothing)
Base.convert(::Type{MaybeInt}, x::Integer) = MaybeInt(x)

@testset "LazyTrees" begin
    @testset "constructors" begin
        t = LazyTree{Symbol, Vector{Int}}()
        @test isempty(children(t))
        show(IOBuffer(), t)

        t2 = LazyTree{Symbol, Vector{Int}}(Int[])
        @test isempty(children(t))
        show(IOBuffer(), t2)
    end

    @testset "data" begin
        t = LazyTree{Symbol, String}("hello")
        @test data(t) == "hello"
        t[:child1] = "world"
        @test data(children(t)[:child1]) == "world"
    end

    @testset "getindex" begin
        t = LazyTree{String, MaybeInt}()
        @test t["foo"]["bar"]["baz"] === t["foo", "bar", "baz"]
        @test t["foo"]["bar"]["baz"] === t[("foo", "bar", "baz")]
        @test t["foo"]["bar"]["baz"] === t[["foo", "bar", "baz"]]
    end

    @testset "setindex" begin
        t = LazyTree{String, MaybeInt}()
        t["foo", "bar", "baz"] = 10
        t[["foo", "bar", "baz"]] = 10
        @test t["foo"]["bar"]["baz"] === t["foo", "bar", "baz"]
        @test t["foo"]["bar"]["baz"] === t[("foo", "bar", "baz")]
        @test t["foo"]["bar"]["baz"] === t[["foo", "bar", "baz"]]
    end

    @testset "delete" begin
        t = LazyTree{String, MaybeInt}()
        t["foo", "bar", "baz"] = 10
        delete!(t, ("foo", "bar"))
        @test t.children["foo"] === t["foo"]
        @test isempty(t["foo"].children)
    end

    @testset "descendants" begin
        t = LazyTree{String, MaybeInt}()
        t["foo", "bar", "baz"] = 10
        t["foo", "buzz"] = 20
        @test descendants(t) == [["foo"],
                                 ["foo", "bar"],
                                 ["foo", "bar", "baz"],
                                 ["foo", "buzz"]]
    end
end
