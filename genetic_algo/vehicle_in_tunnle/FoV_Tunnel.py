import numpy as np
from numpy import array as narr

class Lineseg():
    def __init__(self, pos12, obj_tag=None):
        #self.vtype = vtype #0: line-segment start; 1: line-segment end
        # pos12: 2x2 np array, where: column1=x, xolumn2=y
        self.pos12 = narr(pos12)
        self.pos12p = np.zeros((2,2)); self.update_polar_sort([0,0],0,0)
        self.tag = obj_tag
        
         #rad, angle
        
    def update_polar_sort(self, camera_pos, camera_phi, retina_s_ang):
        # This function should be called everytime the camera moves or rotates.
        pos12tmp = self.pos12 - narr([camera_pos, camera_pos]) #pos12tmp keeps the relative cartesian coordinates of the line_segment
        self.pos12p[:,0] = np.sqrt(pos12tmp[:,0]**2 + pos12tmp[:,1]**2)
        tmp_ang = np.arctan2(pos12tmp[:,1], pos12tmp[:,0]) - camera_phi
        for i in range(len(tmp_ang)):
            if tmp_ang[i] < -np.pi/2:
                tmp_ang[i] += 2*np.pi # We need the angles to be between -90-start_ang and 270-start_ang
        self.pos12p[:,1] = tmp_ang - retina_s_ang
        if self.pos12p[0,1]<self.pos12p[0,0]: #Put the closer vertex on the first row.
            self.pos12p = np.flip(self.pos12p, axis=0)
        return
        
    def __repr__(self):
        return self.__str__()
    def __str__(self):
        return 'Tag='+str(self.tag)+ ' Pos(rho,phi) = '+str(self.pos12p)
    
    
    
class FoV():
    def __init__(self, retina_len, retina_range, view_range, oversample_r=10, verbose=True):
        self.retina_len = retina_len
        self.oversample_r = oversample_r
        self.verbose = verbose
        self.retinah_len = retina_len*oversample_r #The length of the oversampled retina
        self.retina_range = retina_range
        self.retina_s_ang = np.deg2rad(retina_range[0]) # retina start angle: The right-most angle that the agent sees
        self.retina_e_ang = np.deg2rad(retina_range[1]) # retina end angle: The left_most angle the the agent sees
        self.angle_orig = self.retina_s_ang

        self.max_angle_prec = (self.retina_e_ang - self.retina_s_ang)/self.retinah_len

        self.retina = narr([-1]*retina_len)
        self.retinah = narr([-1]*self.retinah_len) # An intermediate variable that holds a higher resolution version of the retina.
        
        self.obj_list = []
        self.lineseg_list = []
        self.view_range = view_range
        
    
    def add_env_obj(self, obj_list):
        # obj_list is a list of lines, each specified by their vertices and tag. 
            # Example:
            # l1 = [[4,1], [0,4], 11] # [vertex1 position, vertex2 position, tag]
            # obj_list = [l1]
        self.obj_list+=obj_list
        # Create vertices from obj_list. 
        # Note that the polar coordinates of the vertices need to be updated to remain correct.
        self._create_lineseg_list()
    
    def remove_env_obj(self, n_obj):
        # In order to save memory, delete the n oldest line segments!
        del self.obj_list[0:n_obj]
        self._create_lineseg_list()
        
    def update_retina(self, camera_pos, camera_phi):
        ### # Create vertices from obj_list, Update the polar coordinates of the vertices and sort them by angle, and assign vtypes.
        self._update_polar(camera_pos, camera_phi)
        self._sort_segs()
        self.retinah.fill(0)
        self._fill_retinah()
        self._downsample_retinah()
        return self.retina
    
    def get_retina(self):
        #probably unnecessary, having a update_retina() method.
        return self.retina
    
    def get_cell_angles(self):
        angle_prec = self.max_angle_prec* self.oversample_r
        cell_angles = [self.angle_orig+i*angle_prec for i in range(self.retina_len)]
        cell_angles.reverse()
        return cell_angles
    
    def _create_lineseg_list(self):
        if self.verbose:
            print(self.obj_list)
        self.lineseg_list = [Lineseg(narr([item[0], item[1]]),obj_tag=item[2]) 
                             for item in self.obj_list]
    
    def _sort_segs(self):
        self.lineseg_list.sort(key=lambda lineseg: lineseg.pos12p[0,0]) #pos12p[0,0] should contain
        #the distance from the closest node of the line segment.
        #print('lineseg_list=',self.lineseg_list)
        
    def _update_polar(self, camera_pos, camera_phi):
        self.angle_orig = self.retina_s_ang +camera_phi
        #map(lambda lineseg: lineseg.update_polar_sort(camera_pos, angle_orig), self.lineseg_list)
        
        # Equivalent to this code:
        for lineseg in self.lineseg_list:
            lineseg.update_polar_sort(camera_pos, camera_phi, self.retina_s_ang)
#             lineseg.update_polar_sort(camera_pos, self.angle_orig)
        
    
    def _fill_retinah(self):
        curr_r = 0; i=0;
#         print("Running _fill_retinah")
#         print("self.lineseg_list = ", self.lineseg_list)
        for i in range(len(self.lineseg_list)):
            curr_r = self.lineseg_list[i].pos12p[0,0]
#             print("curr_r = ",curr_r)
            
            if not np.all(self.retinah) and curr_r <self.view_range:
#                 print("Entered if not np.all...")
                # Iterate through the line segments and 
                # Fill the corresponding retina cells with their tags.
                c_lineseg = self.lineseg_list[i]
                starting_cell = int(c_lineseg.pos12p[0,1] / self.max_angle_prec)
#                 ending_cell = int(c_lineseg.pos12p[1,1] / self.angle_prec)+1
                ending_cell = int(c_lineseg.pos12p[1,1] / self.max_angle_prec)
                
                if starting_cell>ending_cell: # Sort the starting_cell and ending cell.
                    starting_cell, ending_cell = ending_cell, starting_cell #swap variables
                if starting_cell<0: # We don't want negative indices for retina! (corresponds to the objects outside of view)
                    if ending_cell>0:
                        if ending_cell<self.retinah_len/2: #The object stretches into the right half of the visual field
                            starting_cell=0
                        elif ending_cell>self.retinah_len/2 and ending_cell<self.retinah_len:
                            #The object stretches into the left half of the visual field
                            starting_cell=ending_cell
                            ending_cell = self.retinah_len
                    else:
                        continue
                self.retinah[starting_cell:ending_cell+1] = c_lineseg.tag
#                 print('retinah[ ',starting_cell, ':',ending_cell,'=',self.retinah[starting_cell:ending_cell+1])
#                 print('c_lineseg=',c_lineseg,' starting_cell=', starting_cell,' ending_cell=', ending_cell)
            else:
                #print('closing loop with: i=',i,' curr_r=',curr_r)
                break
            
        self.retinah = np.flip(self.retinah, axis=0)

    def _downsample_retinah(self):
        self.retina = self.retinah.reshape(-1, self.oversample_r).mean(axis=1) # reshape retinah into a 2D array where the mean of each column will make one cell in retina.