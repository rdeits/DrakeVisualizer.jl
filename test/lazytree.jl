using DrakeVisualizer.LazyTrees: LazyTree, children, data, delete!, descendants

@testset "LazyTrees" begin
    @testset "constructors" begin
        t = LazyTree{Symbol, Vector{Int}}()
        @test isnull(t.parent)
        @test isempty(children(t))
        show(IOBuffer(), t)

        t2 = LazyTree{Symbol, Vector{Int}}(Int[])
        @test isnull(t2.parent)
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
        t = LazyTree{String, Nullable{Int}}()
        @test t["foo"]["bar"]["baz"] === t["foo", "bar", "baz"]
        @test t["foo"]["bar"]["baz"] === t[("foo", "bar", "baz")]
        @test t["foo"]["bar"]["baz"] === t[["foo", "bar", "baz"]]
    end

    @testset "setindex" begin
        t = LazyTree{String, Nullable{Int}}()
        t["foo", "bar", "baz"] = Nullable{Int}(10)
        t[["foo", "bar", "baz"]] = Nullable{Int}(10)
        @test t["foo"]["bar"]["baz"] === t["foo", "bar", "baz"]
        @test t["foo"]["bar"]["baz"] === t[("foo", "bar", "baz")]
        @test t["foo"]["bar"]["baz"] === t[["foo", "bar", "baz"]]
    end

    @testset "delete" begin
        t = LazyTree{String, Nullable{Int}}()
        t["foo", "bar", "baz"] = Nullable{Int}(10)
        delete!(t, ("foo", "bar"))
        @test t.children["foo"] === t["foo"]
        @test isempty(t["foo"].children)
    end

    @testset "descendants" begin
        t = LazyTree{String, Nullable{Int}}()
        t["foo", "bar", "baz"] = Nullable{Int}(10)
        t["foo", "buzz"] = 20
        @test descendants(t) == [["foo"],
                                 ["foo", "bar"],
                                 ["foo", "bar", "baz"],
                                 ["foo", "buzz"]]
    end
end
