import numpy as np


def rms(*args):
    # args is an arbitrary number of vectors such as error (across time), force1, force2.
    v2sum =0*args[0]
    n_v = 0; # n_v keeps the number of vectors passed.
    for vec in args:
        v2sum += vec**2
        n_v +=1
    return np.sqrt(np.mean(v2sum/n_v))


class Trajectory():
    # A traj_spec is the specification of a trajectory.
    # A trajectory is a combination of several sinusoids, each described by the
    # three factors: amplitude, frequency (Hz) and phase. 
    # 
    # traj = \Sigma_k A_k cos(2\pi f_k t +\phi_k)
    
    
    def __init__(self, tstep, duration):
        self.tstep = tstep
        self.duration = duration            
            
#     def _check_v0(self, traj_spec, exclude=None):
#         # exclude is a list of indices that should be ignored in summation
#         sum_ac =0.
#         for i, snsd in enumerate(traj_spec):
#             if exclude is not None and i in exclude:
#                 continue
#             sum_ac +=(-snsd[1])*snsd[0]*np.sin(snsd[2]) 
#         return sum_ac
    
    def generate_traj_spec(self, amps, max_amp, traj_max_f, amp_std=0.03):
        # Generate a traj_spec randomly
        # amps is a vector of numbers within [0,1], which "roughly" determines 
            # the relative amplitude of the sinosid components.
        
        amps = [0]+amps
        frq_bins, fstep = np.linspace(0, traj_max_f, len(amps), retstep=True)

        traj_spec = []
        for i in range(1, len(amps)):
            frq = fstep*np.random.rand() +frq_bins[i-1]
            amp = max_amp* (amps[i]+amp_std*(np.random.rand()-0.5))#(1-i/(3*n_snsd))* np.random.rand()
            phi = 2*np.pi*np.random.rand()
            traj_spec.append([amp, frq, phi])
        
        return traj_spec
        
    def _traj_derivative(self, traj_spec):
        # Calculate the derivative of a traj_spec. Return the result in the form of another traj_spec.
                
        traj_spec_d=[]
        for snsd in traj_spec:
            traj_spec_d.append([2*np.pi*snsd[1]*snsd[0], snsd[1], snsd[2]+np.pi/2])
        return traj_spec_d
    
        
    
    def get_max_force(self, traj_spec, egg_mass, egg_fric):
        # Calculate how much force is required to follow the above trajectory
#         for tr_sp in traj_specs:
        sum_f =0; #sum_amp =0; 
        for snsd in traj_spec:
            sum_f += abs(snsd[0]*((2*np.pi*snsd[1])**2))*egg_mass + abs(snsd[0]*(2*np.pi*snsd[1]))*egg_fric 
#             sum_amp += snsd[0]
#         fc_req_max = sum_acc/sum_amp
#         fc_req_max = max(acc_max)#*egg_mass
        return sum_f
    


    def generate(self, traj_spec):#, normalize=True):
        # traj_specs is a list of traj_spec. 
        # Each traj_spec is a list of sinosoid descriptors. 
        # Each descriptor comprises 3 scalars: amplitude, frequency, phase.
        # Hence, traj_specs is a 3 dimensional list.
        
        n = np.arange(0, self.duration, self.tstep)
        x =np.zeros_like(n)
        
    # Check traj_spec
        for snsd in traj_spec:
            if len(snsd) !=3:
                raise ValueError 
        
        # Generate the trajectory
        sum_amp =0
        for i in range(len(traj_spec)):
            x += traj_spec[i][0]* np.cos(2*np.pi*n *traj_spec[i][1] +traj_spec[i][2])
            sum_amp += traj_spec[i][0]

#         # Apply the correct scaling
#         if normalize is True:
#             traj = (max_amp/sum_amp) *x   
#         else:
#             traj = max_amp* x 
        
        return n, x

    
    def ideal_force_profile(self, r_spec, egg, egg_bnd):
                
        # Check traj_spec
        for snsd in r_spec:
            if len(snsd) !=3:
                raise ValueError
                
        r_d_spec = self._traj_derivative(r_spec)
        n, r_d = self.generate(traj_spec=r_d_spec)
        r_dd_spec = self._traj_derivative(r_d_spec)
        _, r_dd = self.generate(traj_spec=r_dd_spec)
        
        f_follow = egg_bnd[0]*np.ones_like(n)
        f_lead = f_follow +abs(egg.mass*r_dd +egg.fric*r_d)
        # f_net = egg.mass*r_dd + egg.fric*r_d
        # f_opt = egg_bnd[0]*np.ones_like(n)
        #
        # f1 = (f_net>=0)*f_net + f_opt
        # f2 = -1*(f_net<=0)*f_net + f_opt
        return f_lead, f_follow
    
    
    
    def ideal_force_profile2(self, r_spec, egg, egg_bnd):
                
        # Check traj_spec
        for snsd in r_spec:
            if len(snsd) !=3:
                raise ValueError
                
        r_d_spec = self._traj_derivative(r_spec)
        n, r_d = self.generate(traj_spec=r_d_spec)
        r_dd_spec = self._traj_derivative(r_d_spec)
        _, r_dd = self.generate(traj_spec=r_dd_spec)
        
#         f_follow = egg_bnd[0]*np.ones_like(n)
#         f_lead = f_follow +egg.mass*r_dd +egg.fric*r_d
        f_net = egg.mass*r_dd + egg.fric*r_d
        f_opt = egg_bnd[0]*np.ones_like(n)
        #
        f1 = (f_net>=0)*f_net + f_opt
        f2 = -1*(f_net<=0)*f_net + f_opt
        return f1, f2
    
    
    def get_rms(self, r_spec, egg, egg_bnd):
        f_l, f_f = self.ideal_force_profile(r_spec, egg, egg_bnd)
        f_rms = rms(f_l, f_f)
        return f_rms
    
    
    
    def generate_random(self, n_traj, rel_amps, max_amp, traj_max_f, egg, egg_bnd, n_deriv=0, ret_specs=True):
        traj_specs = []; trajs =[]
        for i in range(n_traj):
            
            traj_spec = self.generate_traj_spec(rel_amps, max_amp, traj_max_f, amp_std=0.03)
            # determine the scaling factor of (the amplitude of) the traj_spec so that they require same energy
            sc_fac = self.get_rms(traj_spec, egg, egg_bnd)
            for i in range(len(traj_spec)):
                traj_spec[i][0] = traj_spec[i][0]/sc_fac
            
            traj_specs.append(traj_spec) 
            time1, traj = self.generate(traj_spec)
            
            traj_dx_spec = traj_spec
            traj = np.expand_dims(traj, axis=0)
            for i in range(n_deriv):
                traj_dx_spec = self._traj_derivative(traj_dx_spec)
                _, traj_dx = self.generate(traj_dx_spec)
                traj = np.concatenate((traj, traj_dx[np.newaxis,:]), axis=0)
            
            trajs.append(traj)
        
        if ret_specs is True:
            return time1, trajs, traj_specs
        else:
            return time1, trajs
        
