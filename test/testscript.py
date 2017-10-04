from director import lcmUtils
from director.utime import getUtime
import robotlocomotion as lcmrl

channel = 'DRAKE_VISUALIZER_JL_TEST'
msg = lcmrl.viewer2_comms_t()
msg.utime = getUtime()
msg.format = 'drake_visualizer_jl_test'
msg.format_version_major = 0
msg.format_version_minor = 1
msg.data = bytearray()
msg.num_bytes = len(msg.data)
lcmUtils.publish(channel, msg)
