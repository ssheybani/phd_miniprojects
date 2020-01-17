# Adapted from: https://github.com/elplatt/elp_nkmodel
import random, numpy

class NK(object):
    
    def __init__(self, N, K, dep='ordered', vtab='identical'):
        self.N = N
        self.K = K
        self.loci = range(N)
        self.depmod = dep
        self.vtabmod = vtab # The value table can either be identical for all loci or it can be different.
        self.values = [{} for n in self.loci]    
        self.__init_values__()
        if self.depmod == 'random':
        # The k loci are chosen randomly
            self.dependence = [ [n] + random.sample(set(self.loci) - set([n]), K) for n in self.loci]
        elif self.depmod == 'ordered':
            tmploci = [n for n in self.loci]; tmploci = tmploci+tmploci;
            self.dependence = [ list(tmploci[n:n+self.K+1]) for n in self.loci]
        else:
            raise ValueError("dep should be either 'random' or 'ordered'.")
        # Reverse lookup for dependence
        self.depends_on = [set() for n in self.loci]
        for n in self.loci:
            for d in self.dependence[n]:
                self.depends_on[d].add(n)
    
    def get_value(self, state):
        total_value = 0.0
        for n in self.loci:
            label = tuple([state[i] for i in self.dependence[n]])
            try:
                total_value += self.values[n][label]
            except KeyError:
                v = random.random()
                self.values[n][label] = v
                total_value += v
        total_value /= float(self.N)
        return total_value
    
    def __create_labels__(gene_size):
        # This function generates all possible labels for a gene of size gene_size.
        # example: gene_size=2, alphabet={0,1}; output=[(0,0), (0,1),(1,0),(1,1)]
        def reproduce(cell):
            """ 
            A recursive function which takes the following global variables as implicit input and outputs, respectively:
            "gene_size" and "labels"
            """
            if len(cell)==gene_size:
                labels.append(cell)
                return
            else:
                reproduce(cell+(0,))
                reproduce(cell+(1,))

        labels = []
        reproduce(()) # an empty cell is passed to the function.
        return labels
    
    def __init_values__(self):
        gene_size = self.K+1
        A=2; gene_var = A**(gene_size)
        labels = NK.__create_labels__(gene_size)
        
        if self.vtabmod == 'identical':
            gene_f = numpy.random.uniform(0,1, gene_var)
            row_values = dict(zip(labels, gene_f))
            for n in self.loci:
                self.values[n] = row_values
        else:
            for n in self.loci:
                gene_f = numpy.random.uniform(0,1, gene_var)
                zipbObj = zip(labels, gene_f)
                self.values[n] = dict(zipbObj)