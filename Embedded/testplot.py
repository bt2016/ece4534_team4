import matplotlib.pyplot as plt
import numpy as np


def startplot():
    plt.ion()
    plt.close()
    t = np.arange(0.0,2.0,0.01)
    s = np.sin(2*np.pi*t)
    plt.plot(t,s,'o')

    plt.xlabel('time (s)')
    plt.ylabel('voltage (mV)')
    plt.title('About as simple as it gets, folks')
    plt.grid(True)
    plt.show()
