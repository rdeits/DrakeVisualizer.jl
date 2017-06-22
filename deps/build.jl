using BinDeps
using CMakeWrapper

@BinDeps.setup

function cflags_validator(pkg_names...)
    return (name, handle) -> begin
        for pkg_name in pkg_names
            try
                run(`pkg-config --cflags $(pkg_name)`)
                return true
            catch ErrorException
            end
        end
        false
    end
end

basedir = dirname(@__FILE__)
director_version = "0.1.0-130-g4109097" 
director_sha = "4109097ab03fe2728bf6ac9a9be1be952e449153"

@static if is_linux()
    deps = [
        python = library_dependency("python", aliases=["libpython2.7.so",], validate=cflags_validator("python", "python2"))
        qt4 = library_dependency("QtCore", aliases=["libQtCore.so", "libQtCore.so.4.8"], depends=[python])
        qt4_opengl = library_dependency("QtOpenGL", aliases=["libQtOpenGL.so", "libQtOpenGL.so.4.8"], depends=[qt4])
        director = library_dependency("ddApp", aliases=["libddApp"], depends=[python, qt4])
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

    provides(AptGet, Dict("libqt4-dev"=>qt4, "libqt4-opengl-dev"=>qt4_opengl, "python-dev"=>python))

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
    # Use the libvtkDRCFilters library instead of libddApp
    # to work around weird segfault when dlclose()-ing libddApp
    deps = [
        director = library_dependency("vtkDRCFilters", aliases=["libvtkDRCFilters.dylib"])
    ]
    provides(BuildProcess, (@build_steps begin
        FileDownloader("http://people.csail.mit.edu/patmarion/software/director/releases/director-$(director_version)-mac.tar.gz",
                       joinpath(basedir, "downloads", "director.tar.gz"))
        CreateDirectory(joinpath(basedir, "usr"))
        (`tar xzf $(joinpath(basedir, "downloads", "director.tar.gz")) --directory=usr --strip-components=1`)
    end), director)

end

@BinDeps.install Dict(:ddApp => :libddApp)
