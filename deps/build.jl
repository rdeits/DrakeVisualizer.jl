using BinDeps
using CMakeWrapper

@BinDeps.setup

basedir = dirname(@__FILE__)
director_version = "0.1.0-35-g26aa674"
director_sha = "1f69904c03b083e95035a2cd3070d59a942c9618"

@static if is_linux()
    deps = [
        python = library_dependency("python", aliases=["libpython2.7.so", "libpython3.2.so", "libpython3.3.so", "libpython3.4.so", "libpython3.5.so", "libpython3.6.so", "libpython3.7.so"])
        python_vtk = library_dependency("vtkCommon", aliases=["libvtkCommon.so", "libvtkCommon.so.5.8", "libvtkCommon.so.5.10"], depends=[python],
            validate=(name, handle) -> begin
                isfile(replace(name, r"vtkCommon", "vtkCommonPythonD", 1))
            end)
        director = library_dependency("ddApp", aliases=["libddApp"], depends=[python_vtk, python])
    ]

    linux_distributor = strip(readstring(`lsb_release -i -s`))
    linux_version = try
        VersionNumber(strip(readstring(`lsb_release -r -s`)))
    catch e
        if isa(e, ArgumentError)
            v"0.0.0"
        else
            rethrow(e)
        end
    end


    if linux_distributor == "Ubuntu" || linux_distributor == "Debian"
        # The vtkPython libraries all have undeclared dependencies on libpython2.7,
        # so they cannot be dlopen()ed without missing symbol errors. As a result,
        # we can't use the regular library_dependency mechanism to look for vtk5
        # and python-vtk. Instead, we combined both dependencies into "python_vtk"
        # and make one build rule to apt-get install all the vtk-related packages.
        provides(SimpleBuild,
            () -> run(`sudo apt-get install libvtk5-qt4-dev python-vtk`),
            python_vtk)
    end

    force_source_build = lowercase(get(ENV, "DIRECTOR_BUILD_FROM_SOURCE", "")) in ["true", "1"]

    director_binary = nothing
    if !force_source_build
        if linux_distributor == "Ubuntu"
            if linux_version >= v"16.04"
                director_binary = "ubuntu-16.04"
            elseif linux_version >= v"14.04"
                director_binary = "ubuntu-14.04"
            end
        elseif linux_distributor == "Debian"
            if linux_version >= v"8.7"
                director_binary = "ubuntu-14.04"
            end
        end
    end

    if director_binary !== nothing
        provides(AptGet, Dict("python2.7" => python))
        provides(BuildProcess, (@build_steps begin
            FileDownloader("http://people.csail.mit.edu/patmarion/software/director/releases/director-$(director_version)-$(director_binary).tar.gz",
                           joinpath(basedir, "downloads", "director.tar.gz"))
            CreateDirectory(joinpath(basedir, "usr"))
            (`tar xzf $(joinpath(basedir, "downloads", "director.tar.gz")) --directory=usr --strip-components=1`)
        end), director)
    else
        provides(Sources,
                 URI("https://github.com/RobotLocomotion/director/archive/$(director_sha).zip"),
                 director,
                 unpacked_dir="director-$(director_sha)")
        provides(CMakeProcess(srcdir=joinpath(basedir, "src",
                                              "director-$(director_sha)", "distro", "superbuild"),
                              cmake_args=["-DUSE_LCM=ON",
                                          "-DUSE_EXTERNAL_INSTALL:BOOL=ON"],
                              targetname=""),
                 director)
    end


elseif is_apple()
    deps = [
        director = library_dependency("ddApp", aliases=["libddApp"])
    ]
    provides(BuildProcess, (@build_steps begin
        FileDownloader("http://people.csail.mit.edu/patmarion/software/director/releases/director-$(director_version)-mac.tar.gz",
                       joinpath(basedir, "downloads", "director.tar.gz"))
        CreateDirectory(joinpath(basedir, "usr"))
        (`tar xzf $(joinpath(basedir, "downloads", "director.tar.gz")) --directory=usr --strip-components=1`)
    end), director)

end

@BinDeps.install Dict(:ddApp => :libddApp)
