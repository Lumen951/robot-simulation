
import numpy as np
def rotation_matrix_x(theta_degrees):
    theta = np.radians(theta_degrees)  
    R = np.array([
        [1, 0, 0, 0],
        [0, np.cos(theta), -np.sin(theta), 0],
        [0, np.sin(theta), np.cos(theta), 0],
        [0, 0, 0, 1]
    ])
    return R

def rotation_matrix_y(theta_degrees):
    theta = np.radians(theta_degrees)  
    R = np.array([
        [np.cos(theta), 0, np.sin(theta), 0],
        [0, 1, 0, 0],
        [-np.sin(theta), 0, np.cos(theta), 0],
        [0, 0, 0, 1]
    ])
    return R

def rotation_matrix_z(theta_degrees):
    theta = np.radians(theta_degrees)  
    R = np.array([
        [np.cos(theta), -np.sin(theta), 0, 0],
        [np.sin(theta), np.cos(theta), 0, 0],
        [0, 0, 1, 0]
        [0, 0, 0, 1]
    ])
    return R

Hxy=rotation_matrix_x(45)@rotation_matrix_y(45)
print(f'Hxy={Hxy}')
Hyx=rotation_matrix_y(45)@rotation_matrix_x(45)
print(f'Hyx={Hyx}')

p=np.array([1, 2, 3, 1])
pr=Hxy@p
print(f'pr={pr[0:3]}')

# print(Rxy[0:2,0:2])