import cv2
import json
from os import listdir
from os.path import exists
import numpy as np
import pyrealsense2 as rs
def load_intrinsics(filename):
    depth_scale = 1.0
    intrin = rs.intrinsics()

    if exists(filename):
        with open(filename) as infile:
            data = json.load(infile)
        depth_scale = data['depth_scale']
        intrin.width = data['width']
        intrin.height = data['height']
        intrin.ppx = data['ppx']
        intrin.ppy = data['ppy']
        intrin.fx = data['fx']
        intrin.fy = data['fy']
        if data['model'] == '%s' % (rs.distortion.brown_conrady,):
            intrin.model == rs.distortion.brown_conrady
        intrin.coeffs = data['coeffs']
        
    return intrin, depth_scale

def depth_coords(intrinsics):
    return np.array([(w, h) for h in range(intrinsics.height) for w in range(intrinsics.width)]).T

def deproject_pixel_to_point(intrinsic, d_coords, depth, depth_scale):
    width = intrinsic.width
    height = intrinsic.height
    _x, _y = d_coords[0], d_coords[1]
	
    ######
# =============================================================================
    for x2d in _x:
        for y2d in _y:
            x3d,y3d,z3d = Get3D(x2d,y2d,depth[y2d,x2d])
        print(x3d.shape)
            #print(point3d)
# =============================================================================
    ######
   
  

def Get3D(x,y,d):
    camera_factor = 1
    camera_cx = 325.5
    camera_cy = 253.5
    camera_fx = 518.0
    camera_fy = 519
    
    z3d = d / camera_factor
    x3d = (x - camera_cx) * (z3d / camera_fx)
    y3d = (y - camera_cy) * (z3d / camera_fy)
    
    return x3d,y3d,z3d
# =============================================================================
# def label(xyz, ):
# =============================================================================
    
'''for filename in listdir(File.color_folder()):
    depth_file = File.img_filename(File.depth_folder(), timestamp)'''
    

depth_file = r"1540995306138.png"

depth = cv2.imread(depth_file, cv2.IMREAD_UNCHANGED)
depth_intrinsics, depth_scale = load_intrinsics(r"intrinsics.json")
d_coords = depth_coords(depth_intrinsics)

xyz = deproject_pixel_to_point(depth_intrinsics, d_coords, depth, depth_scale)


np.savetxt('test4.csv', xyz, delimiter = ',')