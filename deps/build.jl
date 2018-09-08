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

"""
Director's libraries are unversioned, so there's no possible way
to know if a system-installed version of Director is compatible
with this interface. Instead, we have to restrict ourselves to
only versions of Director which are built locally.
"""
function is_local_build(name, handle)
    startswith(relpath(name, @__DIR__), "usr/")
end


basedir = dirname(@__FILE__)
director_version = "0.1.0-234-g74cea84"
director_sha = "02c2ef65f8d1d9f3de1d56d129351cd43846d70b"

@static if Sys.islinux()
    python = library_dependency("python", aliases=["libpython2.7.so",], validate=cflags_validator("python", "python2"))
    qt4 = library_dependency("QtCore", aliases=["libQtCore.so", "libQtCore.so.4.8"], depends=[python])
    qt4_opengl = library_dependency("QtOpenGL", aliases=["libQtOpenGL.so", "libQtOpenGL.so.4.8"], depends=[qt4])
    director = library_dependency("ddApp", aliases=["libddApp"], depends=[python, qt4, qt4_opengl], validate=is_local_build)

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
            FileDownloader("https://dl.bintray.com/patmarion/director/director-$(director_version)-$(director_binary).tar.gz",
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


elseif Sys.isapple()
    # Use the libvtkDRCFilters library instead of libddApp
    # to work around weird segfault when dlclose()-ing libddApp
    director = library_dependency("vtkDRCFilters", aliases=["libvtkDRCFilters.dylib"], validate=is_local_build)
    provides(BuildProcess, (@build_steps begin
        FileDownloader("https://dl.bintray.com/patmarion/director/director-$(director_version)-mac.tar.gz",
                       joinpath(basedir, "downloads", "director.tar.gz"))
        CreateDirectory(joinpath(basedir, "usr"))
        (`tar xzf $(joinpath(basedir, "downloads", "director.tar.gz")) --directory=usr --strip-components=1`)
    end), director)

end

@BinDeps.install Dict(:ddApp => :libddApp)
