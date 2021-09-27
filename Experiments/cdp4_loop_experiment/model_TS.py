'''Winnerless competition model for Visual Target Selection
using Generalized Lotka-Volterra Model as given by Rabinovich et al (2008)
[doi:10.1371/journal.pcbi.1000072]

Code author: Vaishnavi Narayanan - vaishnavi.narayanan@maastrichtuniversity.nl

Description
----------
Simulate N pools representing each visual location that compete for the fixation
of the viewer

parameters
----------
I       : Total input to the pools
I_ext   : External input to the pools
W       : weight matrix describing connections between pools
sig     : noise level

properties
----------
R  : instantaneous firing activity

functions
---------
update_weights  : recalculate weights based on updated parameter 'J'
read_saliency_map: read saliency map input and return the normalized and resized map
simulate(time)  : simulate model for "time" seconds
update          : perform numerical integration for single time step
reset           : reset the model
plot_R          : plot firing rate

'''

import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from pylab import *
from scipy.special import kl_div

class TS:
    def __init__(self, params):
        self.params = params
        # self.dt = self.params['dt']
        self.dt = 1e-3
        self.N = self.params['N']
        self.n = int(np.sqrt(self.N))
        self.I_ext = self.params['I_ext']
        self.sig = self.params['sigma'] * np.sqrt(self.dt/ self.params['tau'])

        X_coord, Y_coord = np.meshgrid(np.arange(self.n), np.arange(self.n))
        X_coord = X_coord.reshape(self.N,1)
        Y_coord = Y_coord.reshape(self.N,1)
        self.distance = np.hypot(X_coord - X_coord.T, Y_coord - Y_coord.T)
        self.update_weights()

        self.I = np.zeros(self.N)
        self.R = np.random.uniform(0.01,0.1,(self.N))

    def update_weights(self):
        Jw, Jb, k1 = self.params['J']
        W = np.eye(self.N) * Jw + (np.eye(self.N)==0) * Jb
        W_dist = W * np.exp(- k1 * self.distance)
        self.W = W_dist

    def read_saliency_map(self, filename):
        img = cv2.imread(filename, cv2.IMREAD_UNCHANGED)
        self.resized = cv2.resize(img, (self.n,self.n), interpolation = cv2.INTER_AREA)
        input = self.resized.ravel()/np.mean(self.resized)
        I_stim = input/max(input)
        return I_stim

    def read_saliency_NRP(self, map):
        self.resized = cv2.resize(map, (self.n,self.n), interpolation = cv2.INTER_AREA)
        input = self.resized.ravel()/np.mean(self.resized)
        I_stim = input/max(input)
        return I_stim

    def update(self):
        self.I += self.dt * (-self.I + \
                        self.params['mu'] * self.I_ext * (1. - self.R / self.params['freq']))
        self.R += self.dt * (self.R * (self.I - np.dot(self.W, self.R) \
                                       + self.sig * np.random.randn(self.N)))

    def simulate(self,time):
        R = np.random.uniform(0.01,0.1, (self.N, time))
        for t in range(time):
            self.update()
            R[:, t] = self.R
        return R

    def reset(self):
        self.I_ext = np.zeros(self.N)
        self.I = np.zeros(self.N)
        self.R = np.random.uniform(0.01,0.1,self.N)
        self.sig = self.params['sigma'] * np.sqrt(self.dt/ self.params['tau'])
        self.update_weights()

    def plot_R(self, R):
        fig0 = plt.figure(0)
        plt.plot(R)
        plt.ylim([0, 2*self.params['mu']])
        plt.title('Activity over time; ' + '\n $J_{w}=$' + str(self.params['J'][0]) \
                    + '; $J_{b}=$' + str(self.params['J'][1])\
                    + '; $k_{1}=$' + str(self.params['J'][2]))
        plt.show()

    def animate_R(self, R, sampling_rate, stimulus_img):
        fig = plt.figure()
        ax = fig.add_subplot(111)
        plt.title('Firing pattern animated over time')
        # im = ax.imshow(np.reshape(R[:,0], (self.n, self.n)))
        im = ax.imshow(cv2.resize(stimulus_img, (self.n,self.n), interpolation = cv2.INTER_AREA))
        eye_pos = np.unravel_index(R[:,0], (self.n, self.n))
        # blur = np.exp(-(winning_target - 0)**2 / (2*5))

        redDot, = plt.plot(eye_pos[0], eye_pos[0], '.r')

        tight_layout()

        def update_img(m):
            # img = R[:,m*sampling_rate]/np.max(R)
            eye_pos = np.unravel_index(R[:,m*sampling_rate], (self.n, self.n))
            # tmp = np.reshape(img, (self.n, self.n))
            redDot.set_data(eye_pos[0], eye_pos[1])
            return redDot

        ani = animation.FuncAnimation(fig, update_img, frames=int((R.shape[1])/sampling_rate)
                                        , interval=20, repeat=False, blit=False)
        return ani
