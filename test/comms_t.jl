@testset "comms_t" begin
    utime = 123456
    format = "format_name_here"
    format_version_major = 1
    format_version_minor = 32
    data = rand(UInt8, 1000)
    msg = DrakeVisualizer.Comms.CommsT(
        utime,
        format,
        format_version_major,
        format_version_minor,
        data)
    encoded = DrakeVisualizer.Comms.encode(msg)
    decoded = DrakeVisualizer.Comms.decode(encoded, DrakeVisualizer.Comms.CommsT)

    for field in fieldnames(typeof(msg))
        @test getfield(msg, field) == getfield(decoded, field)
    end
end
