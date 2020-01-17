import numpy as np
import math

def sigmoid(x):
    return 1/(1+np.exp(-x))

#mu = 0.0
#sig = 1.0
#def sigmoid2(x):
#    return 1./(math.sqrt(2.*math.pi)*sig)*np.exp(-np.power((x - mu)/sig, 2.)/2)

class CTRNN():

    def __init__(self, size):
        self.Size = size                        # number of neurons in the network
        self.Voltage = np.zeros(size)           # neuron activation vector
        self.TimeConstant = np.ones(size)       # time-constant vector
        self.invTimeConstant = np.ones(size)       # time-constant vector
        self.Bias = np.zeros(size)              # bias vector
        self.Weight = np.zeros((size,size))     # weight matrix
        self.Output = np.zeros(size)            # neuron output vector
        self.Input = np.zeros(size)             # neuron output vector
  
    def setParameters(self,genotype):
        k = 0
        for i in range(self.Size):
            for j in range(self.Size):
                self.Weight[i][j] = genotype[k]*15
                k += 1
        for i in range(self.Size):
            self.Bias[i] = genotype[k]*15
            k += 1
        for i in range(self.Size):
            self.TimeConstant[i] =  np.exp(1.5*genotype[k] - 1.5) #maps to # [0.000045, 1]
            self.invTimeConstant[i] = 1.0/self.TimeConstant[i]
        
    def randomizeParameters(self):
        self.Weight = np.random.uniform(-10.,10.,size=(self.Size,self.Size))
        self.Bias = np.random.uniform(-10.,10.,size=(self.Size))
        self.TimeConstant = np.exp(np.random.uniform(-6.,-2.,size=(self.Size)))
        self.invTimeConstant = 1.0/self.TimeConstant

#     def step(self,dt):
#         netinput = self.Input + np.dot(self.Weight.T, self.Output)
    def step(self,dt, Input=None):
        netinput = Input + np.dot(self.Weight.T, self.Output)
        self.Voltage += (dt * self.invTimeConstant)*(-self.Voltage+netinput)
        self.Output = sigmoid(self.Voltage+self.Bias)

    def reset(self):
        self.Voltage = -self.Bias
        self.Output = np.zeros(self.Size)
##    def stepForLoops(self,dt):
##        for i in range(self.Size):
##            netinput = self.Input[i]
##            for j in range(self.Size):
##                netinput += self.Weight[j][i]*self.Output[j]
##            dydt = (1/self.TimeConstant[i])*(-self.Voltage[i]+netinput)
##            self.Voltage[i] += dt * dydt          
##        self.Output = sigmoid(self.Voltage+self.Bias)
