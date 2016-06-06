using BinDeps

@BinDeps.setup

function cflags_validator(pkg_name)
    return (name, handle) -> begin
        try
            run(`pkg-config --cflags $(pkg_name)`)
            return true
        catch ErrorException
            return false
        end
    end
end

@linux? (
    begin
        deps = [
            python_dev = library_dependency("python_dev", aliases=["libpython2.7.so", "libpython3.2.so", "libpython3.3.so", "libpython3.4.so", "libpython3.5.so", "libpython3.6.so", "libpython3.7.so"], validate=cflags_validator("python"))
            vtk5 = library_dependency("vtk5", aliases=["libvtkCommon.5.8", "libvtkCommon.5.10"])
            python_vtk = library_dependency("python_vtk", aliases=["libvtkPythonCore.so"], depends=[vtk, python_dev])
            director = library_dependency("director", aliases=["libddApp"], depends=[vtk5, vtk_python, python_dev])
        ]
        provides(AptGet, Dict("libvtk5-qt4-dev" => vtk, "python-dev" => python_dev, "python-vtk" => python_vtk))
    end
    : begin
        deps = [
        vtk5 = library_dependency("vtk5", aliases=["libvtkCommon.5.8", "libvtkCommon.5.10"])
        director = library_dependency("director", aliases=["libddApp"], depends=[vtk5])
        ]
        @osx_only begin
            if Pkg.installed("Homebrew") === nothing
                error("Homebrew package not installed, please run Pkg.add(\"Homebrew\")")
            end
            using Homebrew
            provides(SimpleBuild,
                (@build_steps begin
                    () -> Homebrew.add("staticfloat/juliadeps/hdf5")
                    () -> Homebrew.brew(`install robotlocomotion/director/vtk5 --with-qt`)
                end), vtk5, os=:Darwin)
        end
    end
)


prefix = joinpath(BinDeps.depsdir(director), "usr")

director_sha = "19fa14e5f5b42984cb677508f3fc14dbe1f38d9b"
director_dirname = "director-$(director_sha)"
provides(Sources,
    URI("https://github.com/rdeits/director/archive/$(director_sha).zip"),
    director,
    unpacked_dir=director_dirname)

director_build_dir = joinpath(BinDeps.depsdir(director), "builds", "director")
director_src_dir = joinpath(BinDeps.depsdir(director), "src", director_dirname, "distro", "superbuild")

provides(BuildProcess, Dict(
    (@build_steps begin
        GetSources(director)
        CreateDirectory(director_build_dir)
        @build_steps begin
            ChangeDirectory(director_build_dir)
            @build_steps begin
                `cmake -DUSE_EXTERNAL_INSTALL=true -DCMAKE_INSTALL_PREFIX="$prefix" -DUSE_BOT_CORE_LCMTYPES=true -DUSE_LCM=true -DUSE_STANDALONE_LCMGL=true $director_src_dir`
                `make`
            end
        end
    end) => director), onload="""
    using PyCall
    sys = pyimport("sys")
    unshift!(PyVector(sys["path"]), joinpath("$(prefix)", "lib", "python" * string(sys[:version_info][1]) * "." * string(sys[:version_info][2]), "dist-packages"))

    ENV["PATH"] = ENV["PATH"] * ":" * "$(joinpath(prefix, "bin"))"
    """
    )


@BinDeps.install Dict(:director => :director)
