# **** Time-related Variables **** 
tstep = 0.025 
duration = 20. #25


ref_slice0 = 100; ref_slice1 = 100; # The visualized slice of the reference
#The following variables should be adjusted together. They all affect the "difficulty" of the game.
# egg bounds, 
# traj_max_amp, traj_max_f 
# f_bound, fdot_bound


# Force and egg dynamics
fdot_bound = 400. #0.4 #0.8
f_bound = 100. #3.

egg_mass = 0.5
egg_fric = 2*egg_mass #b as in f = bv


# Loss-related constants
egg_ub = 1.; egg_lb = 0.1 
traj_max_amp = 0.05
egg_single_step_penalty = 1. # The egg_bnd (normal force bounds) is set in the bottom, after calculating the forces.
brkloss_coef = 1


# **** Parameter vector organization ****
# n_features = 3; 
n_rsamples = 3;
n_fsamples = 2;
n_regimes = 2;
n_agents = 2;


# n_switch = n_agents*n_regimes #number of controllers
role_labels = ['Role 0(S): Force Stabilization', 'Role 1(T): Tracking']
# ctr_labels = ['A1Sw0', 'A1Sw1', 'A2Sw0', 'A2Sw1']
ftr_names = ['r', 'r\'', 'r\"', 'e', 'e\'', 'e\"', 'f', 'f\'', 'c']
# ftr_labels = n_switch*ftr_names


n_w = len(ftr_names) #int(n_samples*n_features+1) #+1 is the constant