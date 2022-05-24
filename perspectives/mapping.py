import numpy as np
import cv2
import glob

#Directories
loaddir="/home/sijt/ISBEP/Arm_system/agrobot_ws/src/detection/scripts/camera_data/"
savedir = "/home/sijt/ISBEP/Arm_system/perspectives/"

#Camera Parameters
cam_mtx=np.load(loaddir+'cam_mtx.npy')
dist=np.load(loaddir+'dist.npy')
newcam_mtx=np.load(loaddir+'newcam_mtx.npy')
roi=np.load(loaddir+'roi.npy')

#load in intrinsic matrix
#make world points, object points
#make rvec from solvePnP
#make R matrix with rodrigues
#Image coordinate to normalized camera frame
#calculate s 

cx = cam_mtx[0,2]
cy = cam_mtx[1,2]
fx = cam_mtx[0,0]
fy = cam_mtx[1,1]

#XYZ at world position of cx,cy
X_center=10.9
Y_center=10.7
Z_center=43.4
worldPoints=np.array([[X_center,Y_center,Z_center],
                       [5.5,3.9,46.8],
                       [14.2,3.9,47.0],
                       [22.8,3.9,47.4],
                       [5.5,10.6,44.2],
                       [14.2,10.6,43.8],
                       [22.8,10.6,44.8],
                       [5.5,17.3,43],
                       [14.2,17.3,42.5],
                       [22.8,17.3,44.4]], dtype=np.float32)

imagePoints=np.array([[cx,cy],
                       [502,185],
                       [700,197],
                       [894,208],
                       [491,331],
                       [695,342],
                       [896,353],
                       [478,487],
                       [691,497],
                       [900,508]], dtype=np.float32)

retva, rvec, tvec = cv2.solvePnP(worldPoints, imagePoints,cam_mtx, dist)
R, jacobian = cv2.Rodrigues(rvec)
Rt=np.column_stack((R,tvec))
P = cam_mtx.dot(Rt)


def calcXYZ(u,v):
    xc = (u-cx)/fx #x'
    yc = (v-cy)/fy #y'
    xy1 = np.array([[xc,yc,1]], dtype=np.float32) 
    xy1 = xy1.T
    s = calcScaling()
    xyz = s*xy1
    R_T = R.T
    XYZ = R_T.dot(xyz) - R_T.dot(tvec)
    return XYZ.T

def calcXYZalt(u,v): #based on the FDXLabs method
    s = calcScaling()
    A_inv = np.linalg.inv(cam_mtx)
    R_inv = np.linalg.inv(R)
    uv1 = np.array([[u,v,1]], dtype=np.float32)
    uv1 = uv1.T #transpose array
    suv1 = s*uv1
    XYZ_int = A_inv.dot(suv1)
    XYZ_int2 = XYZ_int - tvec
    XYZ = R_inv.dot(XYZ_int2)
    return XYZ.T

def calcScaling():
    s = np.empty([np.size(worldPoints,0),1])

    for i in range(0,np.size(worldPoints,0)):
        #print("-----------", i, "------------")
        XYZ1 = np.array([[worldPoints[i,0], worldPoints[i,1], worldPoints[i,2],1 ]], dtype=np.float32) #(X,Y,Z,1)^T
        XYZ1 = XYZ1.T
        #print("XYZ1: ", XYZ1, sep = '\n')
        suv1 = P.dot(XYZ1)
        #print("suv1: ", suv1,sep = '\n')
        s[i,0] = suv1[2,0] #(u,v,1)T.s = (su,sv,s)T -> s = [2,0]
        uv1 = suv1/s[i,0]
        #print("s: ",s,sep = '\n')
    s_mean = np.mean(s)
    #print("Mean s: ",s_mean)
    return s_mean

coord = np.empty((0,3),dtype=np.float32)
coord_alt = np.empty((0,3),dtype=np.float32)

for i in range(0,np.size(imagePoints,0)):    
    coord = np.append(coord, calcXYZ(imagePoints[i,0], imagePoints[i,1]), axis=0)
    coord_alt = np.append(coord_alt, calcXYZalt(imagePoints[i,0], imagePoints[i,1]), axis=0)
    #coord_alt[:,i] = calcXYZalt(imagePoints[i,0], imagePoints[i,1])
#print("ABC method:", coord , sep = '\n')
#print("FDX method:", coord_alt , sep = '\n')
error = abs((coord - worldPoints)/worldPoints) *100
print("Error [%]", error)
error_alt = abs((coord_alt - worldPoints)/1) *1
print("---------------------------")
print("Error [-]", error_alt)
print("Mean error error [%]", np.mean(error, axis=0))
print("Mean error error [-]", np.mean(error_alt, axis=0))
niks = 230

