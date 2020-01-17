# Configuration of the visualization
# The shapes are RViz basic shapes

from visualization_msgs.msg import Marker

# egg parameters
egg_dims = [0.05, 0.035, 0.035]
egg_color = [0., 0., 0.]

ref_size = 0.02
ref_color = [1., 1., 0.]
# See ref_slice0, ref_slice1 in config.py

viz_scale = 10.
nf_scale = viz_scale/3
#

# The indicator for normal force and its bounds
fmeter_pos = [3., -2.] # In RViz screen map coordinates
fmeter_w = 0.02
fmeter_c = [0., 1., 0.]

fmeter_base_c = [1., 0., 0.]

nf_size = ref_size
nf_c = [0.67, 0.84, .9]

nftext_size = 0.02
