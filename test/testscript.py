import os

fname = "test_script_success"
with open(fname, 'a'):
    os.utime(fname, None)