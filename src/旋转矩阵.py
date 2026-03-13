
import numpy as np
def rotation_matrix_x(theta_degrees):
    theta = np.radians(theta_degrees)  
    R = np.array([
        [1, 0, 0],
        [0, np.cos(theta), -np.sin(theta)],
        [0, np.sin(theta), np.cos(theta)]
    ])
    return R

def rotation_matrix_y(theta_degrees):
    theta = np.radians(theta_degrees)  
    R = np.array([
        [np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])
    return R

def rotation_matrix_z(theta_degrees):
    theta = np.radians(theta_degrees)  
    R = np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]
    ])
    return R

Rxy=rotation_matrix_x(45)@rotation_matrix_y(45)
print(f'Rxy={Rxy}')
Ryx=rotation_matrix_y(45)@rotation_matrix_x(45)
print(f'Ryx={Ryx}')

p=np.array([1, 2, 3])
pr=Rxy@p
print(f'pr={pr}')

