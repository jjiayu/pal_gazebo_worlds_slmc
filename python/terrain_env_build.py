#Scripts Building the environment model for optimization as well as the gazebo world file for simulation

from ast import Constant
import os

from multicontact_learning_local_objectives.python.terrain_create import *
import multicontact_learning_local_objectives.python.visualization as viz
from multicontact_learning_local_objectives.python.terrain_create.geometry_utils import *
import pickle
import sys

terrain_file_name = sys.argv[1]

# Check if we have the world file genrated or not
world_file_path = "/home/jiayu/catkin_ws/src/pal_gazebo_worlds_slmc/worlds/testing_terrain.world"
terrain_model_file_path = "/home/jiayu/catkin_ws/src/sl1m_fixed_combinatorials/planning_code/" + terrain_file_name
# terrain_model_logfile_path = "/home/jiayu/Desktop/MLP_DataSet/Env_Models/uneven_terrain_generate_log.txt"

# Stop the program if we have uneven terrain world file already existed
if os.path.isfile(world_file_path):
    raise Exception("Uneven Terrain World Description File Already Exists")

# #Logging command line output
# logging = True
# if logging == True:
#     stdoutOrigin=sys.stdout; sys.stdout = open(terrain_model_logfile_path, "w")

# Load terrain model
with open(terrain_model_file_path, 'rb') as f: #nodes_3.pickle terrain_and_contact_seq_.p
    terrain_and_contact_seq = pickle.load(f)

terrain_model = terrain_and_contact_seq["TerrainModel"]

# print(terrain_model)

# print(terrain_and_contact_seq.keys())

# Make a world file (with ground floor) if we dont have one
with open(world_file_path, 'x') as f:
    f.write('<?xml version="1.0" ?>\n')
    f.write('<sdf version="1.4">\n')
    f.write('  <world name="default">\n')
    f.write('    <physics type="ode">\n')
    f.write('      <gravity>0 0 -9.81</gravity>\n')
    f.write('      <ode>\n')
    f.write('        <solver>\n')
    f.write('          <type>quick</type>\n')
    f.write('          <iters>50</iters>\n')
    f.write('          <sor>1.4</sor>\n')
    f.write('        </solver>\n')
    f.write('        <constraints>\n')
    f.write('          <cfm>0.0</cfm>\n')
    f.write('          <erp>0.2</erp>\n')
    f.write('          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>\n')
    f.write('          <contact_surface_layer>0.0</contact_surface_layer>\n')
    f.write('        </constraints>\n')
    f.write('      </ode>\n')
    f.write('      <real_time_update_rate>1000</real_time_update_rate>\n')
    f.write('      <max_step_size>0.001</max_step_size>\n')
    f.write('    </physics>\n')
    f.write('\n')
    f.write('    <!-- A global light source -->\n')
    f.write('    <include>\n')
    f.write('      <uri>model://sun</uri>\n')
    f.write('    </include>\n')
    f.write('\n')
    f.write('    <!-- A ground plane -->\n')
    f.write('    <include>\n')
    f.write('      <uri>model://ground_plane</uri>\n')
    f.write('    </include>\n')
    f.write('\n')

    # # Write the front bar
    # f.write('    <!-- Place the front bar -->\n')
    # f.write('    <include>\n')
    # f.write('      <name>font_bar</name>\n')
    # f.write('      <static>'+ str(1) +'</static>\n')
    # init_surf_border_x = terrain_model["AllPatchesVertices"][0][0][0]-0.01
    # init_surf_border_y = terrain_model["AllPatchesVertices"][0][2][1]
    # f.write('      <uri>model://front_bar</uri>\n')
    # f.write('      <pose> ' + str(init_surf_border_x) + " " + str(init_surf_border_y) + " " + str(-doormat_height) + ' 0.0 0.0 ' + str(0.0) + '</pose>\n')
    # f.write('    </include>\n')
    # f.write('\n')

# # construct the gazebo world file, append terrain models
with open(world_file_path, 'a') as f:
    for surf_idx in range(len(terrain_model)):
        #temp_type = terrain_model["AllPatchesVertices"][surf_idx]
        temp_surf = terrain_model[surf_idx]
        print(temp_surf)
        # temp_center_x, temp_center_y, temp_center_z = getCenter(temp_surf)
        #get min z of the patch
        cur_patch_min_z = np.min([temp_surf[0,2], temp_surf[1,2], temp_surf[2,2], temp_surf[3,2]])
        # temp_type = getSurfaceType(temp_surf)
        # temp_surf_inclination = abs(getTerrainRotationAngle(temp_surf))#terrain_model["ContactSurfsInclinationsDegrees"][surf_idx]
        size_x = max(temp_surf[0:,0])-min(temp_surf[0:,0])
        size_y = max(temp_surf[0:,1])-min(temp_surf[0:,1])
        temp_center_x = min(temp_surf[0:,0]) + size_x/2.0
        temp_center_y = min(temp_surf[0:,1]) + size_y/2.0
        size_z = 0.01
        # print(temp_type)
        f.write('    <!-- Place a Block -->\n')
        f.write('      <model name = "block_'+ str(surf_idx)+'"' + '>\n')
        f.write('        <pose> ' + str(temp_center_x) + " " + str(temp_center_y) + " " + str(-size_z/2+0.005) + ' 0.0 0.0 0.0' + '</pose>\n')
        f.write('        <static>'+ str(1) +'</static>\n')
        f.write('        <link name="box_'+str(surf_idx)+'_body">\n')
        f.write('          <inertial>\n')
        f.write('          <mass>1.0</mass>\n')
        f.write('          <inertia>\n')
        f.write('            <ixx>1.0</ixx>\n')
        f.write('            <ixy>0.0</ixy>\n')
        f.write('            <ixz>0.0</ixz>\n')
        f.write('            <iyy>1.0</iyy>\n')
        f.write('            <iyy>1.0</iyy>\n')
        f.write('            <iyz>0.0</iyz>\n')
        f.write('            <izz>1.0</izz>\n')
        f.write('          </inertia>\n')
        f.write('        </inertial>\n')
        f.write('        <collision name="collision">\n')
        f.write('          <geometry>\n')
        f.write('            <box>\n')
        f.write('              <size>' + str(size_x) + ' ' + str(size_y) + ' ' + str(size_z) + ' ' + '</size>\n')
        f.write('            </box>\n')
        f.write('          </geometry>\n')
        f.write('        </collision>\n')
        f.write('        <visual name="visual">\n')
        f.write('          <geometry>\n')
        f.write('            <box>\n')
        f.write('              <size>' + str(size_x) + ' ' + str(size_y) + ' ' + str(size_z) + ' ' + '</size>\n')
        f.write('            </box>\n')
        f.write('          </geometry>\n')
        f.write('        </visual>\n')
        f.write('      </link>\n')
        f.write('    </model>\n')
        f.write('\n')

#Closing the world file
with open(world_file_path, 'a') as f:
    f.write('  </world>\n')
    f.write('</sdf>\n')
