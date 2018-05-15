import scipy.io.wavfile
import numpy as np
import sys


if __name__ == '__main__':
    np.set_printoptions(threshold=np.nan)
    
    fname = sys.argv[1]
    name = fname.split('.')[0]
    
    rate, rdata = scipy.io.wavfile.read(fname)
    
    # print([d-128 for d in rdata])
    print(rdata)
    
    sys.exit()
    
    zero = lambda sample: np.int8(sample-128)
    zdata = np.array([zero(val) for val in rdata])
    
    posmax = 0
    negmax = 0
    
    for val in zdata:
        if val > posmax:
            posmax = val
        elif val < negmax:
            negmax = val
    
    if posmax < abs(negmax):
        uprat = -128.0/negmax
    else:
        uprat = 127.0/posmax
    
    # testing the following line
    uprat = 60.0
    
    
    scale = lambda sample: np.int64(sample*uprat)
    sdata = np.array([scale(val) for val in zdata])
    
    unzero = lambda sample: np.uint8(sample+128)
    wdata = np.array([unzero(val) for val in sdata])
    
    # testing the following two lines
    np.clip(sdata, -128, 127, wdata)
    wdata = np.array([np.int8(d+128) for d in wdata])
    
    wfile = name + '_SCALED' + '.WAV'
    
    scipy.io.wavfile.write(wfile, rate, wdata)