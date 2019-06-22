import sys
import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    file_path = sys.argv[1]
    temp = np.loadtxt(file_path, dtype=np.str, delimiter=",")
    print 'load file:',file_path
    label = temp[0,:].astype(np.str)
    data = temp[1:,:].astype(np.float)
    timestamp = (data[:,0]-data[0,0])/1000000000
    print 'data length:',len(data)
    print timestamp
    for i in range(len(label)): 
        if i!= 0:
            print '[',i,']: ',label[i]
    print 'plot the curve'
    print 'example:[1,2]'
    wtp = [1,2,3,14,15,16]
    for i in range(len(wtp)):
        plt.plot(timestamp, data[:,wtp[i]], linewidth = 2.5, linestyle='-',marker='.', label=label[wtp[i]])
    plt.xlabel('t')
    plt.legend()
    plt.show()
