import random
import numpy as np
import matplotlib.pyplot as plt

class EA():
    def __init__(self, fitnessFunction, popsize, genesize, recombProb, mutatProb, genotype_encoding):
        self.fitnessFunction = fitnessFunction
        self.popsize = popsize
        self.genesize = genesize
        self.recombProb = recombProb
        self.mutatProb = mutatProb
        self.avgHistory = []
        self.bestHistory = []
        self.genotype_encoding = genotype_encoding
        if genotype_encoding == "R":
            self.pop = np.random.rand(popsize,genesize)*2 - 1           # Real-value encoding
        else:
            self.pop = np.random.randint(2,size=(popsize,genesize))     # Binary encoding
        self.popfit = np.zeros(popsize) #@@@@@@@@@@@@@@@

    def showFitness(self):
        plt.plot(self.bestHistory)
        plt.plot(self.avgHistory)
        plt.xlabel("Generations")
        plt.ylabel("Fitness")
        plt.title("Best and average fitness")
        plt.show()
        
    def fitStats(self):
        bestfit = 0
        bestind = -1
        avgfit = 0
        k=0
        for ind in self.pop:
            fit = self.fitnessFunction(ind)
            self.popfit[k] = fit  #@@@@@@@@@@@@@@@
            avgfit += fit
            if (fit > bestfit):
                bestfit = fit
                bestind = ind
            k+=1
        return avgfit/self.popsize, bestfit, bestind

    def reportStats(self):
        af, bf, bi = self.fitStats()
        self.avgHistory.append(af)
        self.bestHistory.append(bf)

    def pick2(self):
        a = random.randint(0,self.popsize-1)
        b = random.randint(0,self.popsize-1)
        while (a==b):   # Make sure they are two different individuals
            b = random.randint(0,self.popsize-1)
        return a,b

    def compare(self,a,b):
#         if (self.fitnessFunction(self.pop[a]) > self.fitnessFunction(self.pop[b])):
        if (self.popfit[a] > self.popfit[b]):
            winner = a
            loser = b
        else:
            winner = b
            loser = a
        return winner, loser

    def recombine(self,winner,loser):
        for l in range(self.genesize):
            if (random.random() < self.recombProb):
                self.pop[loser][l] = self.pop[winner][l]

    def mutateB(self,loser):
        for l in range(self.genesize):
            if (random.random() < self.mutatProb):
                if self.pop[loser][l] == 1:
                    self.pop[loser][l] = 0
                else:
                    self.pop[loser][l] = 1
                
    def mutateR(self,loser):
        for l in range(self.genesize):
            self.pop[loser][l] += random.gauss(0.0,self.mutatProb)
            if self.pop[loser][l] > 1.0:
                self.pop[loser][l] = 1.0
            if self.pop[loser][l] < -1.0:
                self.pop[loser][l] = -1.0
            
    def tournament(self):
        # Step 1: Pick 2 individuals
        a,b = self.pick2()
        # Step 2: Compare their fitness
        w,l = self.compare(a,b)
        # Step 3: Transfect loser with winner
        self.recombine(w,l)
        # Step 4: Mutate loser and Make sure new organism stays within bounds
        #self.mutateB(l)
        self.mutateR(l)

    def generation(self):
        self.reportStats() #Also updates self.popfit
        for i in range(self.popsize):
            self.tournament()
