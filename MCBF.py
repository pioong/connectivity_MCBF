import numpy as np
from scipy.optimize import minimize
import cvxpy as cp

class CBF_SDP():
    def __init__(self, comm_range, DRONE_COUNT, DRONE_SIZE, maintain_connectivity = True, avoid_collision = True):
        self.communication_range = comm_range
        self.DRONE_COUNT = DRONE_COUNT
        self.DRONE_SIZE = DRONE_SIZE
        self.epsilon = 0.1
        self.alpha1 = 3.0  # Connectivity
        self.alpha2 = 1.0 # Collision
        self.u = cp.Variable(2*DRONE_COUNT)
        self.Lgh2 = cp.Parameter((int((DRONE_COUNT-1)*DRONE_COUNT/2), 2*DRONE_COUNT))
        self.Lgh2.value = np.zeros((int((DRONE_COUNT-1)*DRONE_COUNT/2), 2*DRONE_COUNT))
        self.h2 = cp.Parameter(int((DRONE_COUNT-1)*DRONE_COUNT/2))
        self.h2.value = np.zeros(int((DRONE_COUNT-1)*DRONE_COUNT/2))
        self.u_des = cp.Parameter(2*DRONE_COUNT)
        self.L = cp.Parameter((DRONE_COUNT, DRONE_COUNT))
        self.L.value = np.zeros((DRONE_COUNT,DRONE_COUNT))
        self.LgH = cp.Parameter((DRONE_COUNT,DRONE_COUNT,2*DRONE_COUNT))
        self.LgH.value = np.zeros((DRONE_COUNT,DRONE_COUNT,2*DRONE_COUNT))
        self.slack = cp.Variable()
        self.lambdas = cp.Parameter(DRONE_COUNT)
        self.indicator = cp.Parameter((2,2*DRONE_COUNT))
        self.indicator_times_u_des = cp.Parameter((2,))
        obj = cp.Minimize(cp.sum_squares(self.u-self.u_des)+self.slack**2)
        constraints = []
        constraints += [self.indicator @ self.u == self.indicator_times_u_des]
        if avoid_collision:
            constraints += [self.Lgh2 @ self.u + self.alpha2 * self.h2 >=0]
        
        LguH = cp.Parameter((self.DRONE_COUNT, self.DRONE_COUNT))
        LguH.value = np.zeros((self.DRONE_COUNT, self.DRONE_COUNT))
        for i in range(2*DRONE_COUNT):
            LguH += self.LgH[:, :, i] * self.u[i]

        if maintain_connectivity:
            constraints += [LguH + self.alpha1 * (self.L - self.epsilon * cp.Constant(np.eye(self.DRONE_COUNT))
                                           + cp.Constant(np.ones((self.DRONE_COUNT,self.DRONE_COUNT))) )>> 0 ]
        self.prob = cp.Problem(obj,constraints)

    def compute_desired_velocities(self, positions, destination_vector, kp=0.01, v_max=0.8):
        positions = np.array(positions, dtype=float)
        destination_vector = np.array(destination_vector, dtype=float)
        return kp*(destination_vector - positions)

    def create_Laplacian_matrix(self, positions):
        positions = np.array(positions)  # Ensure positions is a NumPy array
        position_difference = positions[:, np.newaxis, :] - positions[np.newaxis, :, :]
        distances_sq = np.sum(position_difference ** 2, axis=2)

        A = np.exp(((1 - distances_sq/self.communication_range**2)**2) ) - 1 # Compute adjacency values
        A[distances_sq > self.communication_range ** 2] = 0
        np.fill_diagonal(A, 0)
        D = np.diag(np.sum(A, axis=1))
        L = D - A

        dL_dx = [np.zeros((self.DRONE_COUNT,self.DRONE_COUNT)) for _ in range(2*self.DRONE_COUNT)]

        for dimension in range(2):
            dA_dx_preprocess = (A+1)*2*(1 - distances_sq/self.communication_range**2)/ self.communication_range**2 *2*position_difference[:,:,dimension]
            dA_dx_preprocess[distances_sq > self.communication_range ** 2] = 0
            np.fill_diagonal(dA_dx_preprocess, 0)
            for i in range(self.DRONE_COUNT):
                dA_dxi = np.zeros((self.DRONE_COUNT,self.DRONE_COUNT))
                dA_dxi[i,:] = -dA_dx_preprocess[i,:]
                dA_dxi[:,i] = dA_dx_preprocess[:,i]
                dD_dxi = np.diag(np.sum(dA_dxi, axis=1))
                dL_dx[dimension+2*i] = dD_dxi - dA_dxi
        return L, dL_dx

        
    def update_prioritized_task_constraint(self, u_des, manual_drone_index):
        self.priority_index = manual_drone_index
        self.indicator.value = np.zeros((2,2*self.DRONE_COUNT))
        self.indicator.value[0,2*self.priority_index] = 1
        self.indicator.value[1,2*self.priority_index+1] = 1
        self.indicator_times_u_des.value = self.indicator.value @ self.u_des.value
        return
    

    def update_connectivity_CBF(self, positions):
        self.L.value, dL_dx = self.create_Laplacian_matrix(positions)

        eigenvalues, _ = np.linalg.eigh(self.L.value) #eigh returns smallest to largest eigval
        self.lambdas.value = eigenvalues
        for i in range(2*self.DRONE_COUNT):
            self.LgH.value[:, :, i] = dL_dx[i]
        return
    
    def update_collision_CBF(self, positions):
        positions = np.array(positions)
        x_diff = positions[:, np.newaxis, :] - positions[np.newaxis, :, :]  # Shape: (N, N, 2)
        pair = 0
        for i in range(self.DRONE_COUNT-1):
            for j in range(i+1,self.DRONE_COUNT):
                self.Lgh2.value[pair,2*i] = x_diff[i,j,0]
                self.Lgh2.value[pair,2*i+1] = x_diff[i,j,1]
                self.Lgh2.value[pair,2*j] = x_diff[j,i,0]
                self.Lgh2.value[pair,2*j+1] = x_diff[j,i,1]
                dist = np.linalg.norm(x_diff[i,j,:])
                self.h2.value[pair] = dist**2-(2.5*self.DRONE_SIZE)**2
                pair = pair + 1
        return

    def solve_sdp(self, positions, u_des, manual_drone_index):

        self.u_des.value = u_des.flatten() 

        self.update_prioritized_task_constraint(u_des, manual_drone_index)
        self.update_connectivity_CBF(positions)
        self.update_collision_CBF(positions)

        try:
            self.prob.solve(solver='CLARABEL',warm_start=True,canon_backend=cp.SCIPY_CANON_BACKEND)
        except:
            print("Warning: Insufficient Solver Progress. Reducing Tolerances...")
            try:
                self.prob.solve(solver='CLARABEL',warm_start=False,canon_backend=cp.SCIPY_CANON_BACKEND,
                            equilibrate_enable=True,static_regularization_enable=True,
                            tol_gap_abs = 1e-5, tol_gap_rel = 1e-5, tol_feas = 1e-5)
                print("Solved. Returning to Default Tolerances...")
            except:
                print("Warning: Still Insufficient Solver Progress. Using Last Solution...")

        if self.u.value is None or self.prob.status == 'infeasible':
            print('Error solving SDP. Returning u_des instead.')
            return u_des

        result = self.u.value 
        return result.reshape(self.DRONE_COUNT, 2) 

    