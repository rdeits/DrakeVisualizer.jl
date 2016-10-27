using BinDeps

@BinDeps.setup

basedir = joinpath(Pkg.dir("DrakeVisualizer"), "deps")
director_version = v"0.1.0"

@static if is_linux()
    deps = [
        python = library_dependency("python", aliases=["libpython2.7.so", "libpython3.2.so", "libpython3.3.so", "libpython3.4.so", "libpython3.5.so", "libpython3.6.so", "libpython3.7.so"])
        python_vtk = library_dependency("vtkCommon", aliases=["libvtkCommon.so", "libvtkCommon.so.5.8", "libvtkCommon.so.5.10"], depends=[python],
            validate=(name, handle) -> begin
                isfile(replace(name, r"vtkCommon", "vtkCommonPythonD", 1))
            end)
        director = library_dependency("ddApp", aliases=["libddApp"], depends=[python_vtk, python])
    ]
    provides(SimpleBuild,
        () -> run(`sudo apt-get install libvtk5-qt4-dev python-vtk`),
        python_vtk)
    provides(AptGet, Dict("python2.7" => python))
    provides(BuildProcess, (@build_steps begin
        FileDownloader("http://people.csail.mit.edu/patmarion/software/director/releases/director-$(director_version)-linux.tar.gz",
                       joinpath(basedir, "downloads", "director.tar.gz"))
        CreateDirectory(joinpath(basedir, "usr"))
        (`tar xzf $(joinpath(basedir, "downloads", "director.tar.gz")) --directory=usr --strip-components=1`)
    end), director)
end

@BinDeps.install Dict(:ddApp => :libddApp)
