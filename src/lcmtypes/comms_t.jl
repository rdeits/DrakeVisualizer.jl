module Comms

import LCMCore: encode, decode

struct CommsT
    utime::Int64
    format::String
    format_version_major::Int32
    format_version_minor::Int32
    data::Vector{UInt8}
end

fingerprint(::Type{CommsT}) = UInt8[211, 104, 224, 63, 51, 197, 104, 190]

function encode(comms::CommsT)
    io = IOBuffer()
    write(io, fingerprint(CommsT))
    write(io, hton(comms.utime))
    write(io, hton(UInt32(length(comms.format) + 1)))
    write(io, comms.format)
    write(io, UInt8(0))
    write(io, hton(comms.format_version_major))
    write(io, hton(comms.format_version_minor))
    write(io, hton(Int32(length(comms.data))))
    write(io, hton.(comms.data))
    io.data
end

struct FingerprintException <: Exception
    message::String
end

function decode(data::Vector{UInt8}, ::Type{CommsT})
    io = IOBuffer(data)
    msg_fingerprint = read(io, 8)
    if msg_fingerprint != fingerprint(CommsT)
        throw(FingerprintException("LCM message fingerprint did not match. This means that you are trying to decode the wrong message type, or a different version of the message type."))
    end
    utime = Int64(ntoh(read(io, Int64)))
    formatlen = UInt32(ntoh(read(io, UInt32)))
    format = String(read(io, formatlen - 1))
    read(io, 1) # strip off null
    ver_maj = Int32(ntoh(read(io, Int32)))
    ver_min = Int32(ntoh(read(io, Int32)))
    num_bytes = Int32(ntoh(read(io, Int32)))
    data = String(read(io, UInt8, num_bytes))
    CommsT(utime, format, ver_maj, ver_min, data)
end

end
