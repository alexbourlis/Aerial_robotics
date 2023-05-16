import numpy as np
from numpy.linalg import norm

v_drone = np.array([1,2])
v_goal = np.array([10,4])


def dist(point):

    v_drone = np.array([1,2]).reshape((2,1))
    v_point = np.array(point)
    v_goal = np.array([10,4]).reshape((2,1))                                                    
    dir_point = v_point-v_drone.T
    vec_path = v_goal-v_drone
    v_dist = dir_point-(dir_point.dot(vec_path)/vec_path.T.dot(vec_path))*vec_path.T
    return v_dist

def vector_orientation(u,v):
	angle = np.zeros((v.shape[0],1))
	u = np.array(u).reshape((1,2))
	u_prime = np.array([[-u[0][1],u[0][0]]])
	v = np.array(v)
	det_uv = v.dot(u_prime.T)
	scal_prod = v.dot(u.T)

	mask = (scal_prod < 0) | (det_uv == 0)
	angle[mask] = np.pi
	angle[~mask] = np.sign(det_uv[~mask])*np.arccos(scal_prod[~mask]/norm(u)/norm(v,axis=1).reshape((v.shape[0],1))[~mask])

	return angle
	#return np.squeeze(angle).tolist()

s = np.array([[2,2],[3,3],[4,4]])

u = (v_goal-v_drone).reshape((1,2))
v = s-v_drone

i = vector_orientation(u,v)
print(i)

#print(u)
#print(v)
#u_prime = np.array([[-u[0][1],u[0][0]]])
#print(u_prime)
#
#print(u)
#det_uv = v.dot(u_prime.T)
#print(det_uv)
#scal_prod = v.dot(u.T)
#print(scal_prod)
#mask = (scal_prod < 0) | (det_uv == 0)
#print(mask)
#print(norm(v,axis=1).reshape((v.shape[0],1))[~mask])
#print(det_uv[~mask])
#print(scal_prod[~mask])
#print(norm(u))
#print(np.arccos(scal_prod[~mask]/norm(u)/norm(v,axis=1).reshape((v.shape[0],1))[~mask]))
#print(np.sign(det_uv[~mask]))
#print(np.sign(det_uv[~mask])*np.arccos(scal_prod[~mask]/norm(u)/norm(v,axis=1).reshape((v.shape[0],1))[~mask]))
#angle = np.zeros((v.shape[0],1))
#angle[~mask] = np.sign(det_uv[~mask])*np.arccos(scal_prod[~mask]/norm(u)/norm(v,axis=1).reshape((v.shape[0],1))[~mask])
#print(angle)

