import cvxpy as cp
import numpy as np
import time

class VSCS:

    def __init__(self):
        self.A = []
        self.B = []
        self.L = []
        self.lambda2_L = 0
        self.lambda_max_L = 0
        self.lambda_max_L2 = 0

    def init_vscs_model(self):
        # start_time = time.time()
        self.init_system_dynamic_matrix()
        self.init_curr_L()
        self.calc_lambda_L()
        # self.validate_parameters()
        # end_time = time.time()
        # elapsed_time = end_time - start_time
        # print(f"Function executed in {elapsed_time:.4f} seconds")

    def init_system_dynamic_matrix(self):
        self.A = np.array([
            [0, -1],
            [0, -1]
            ])

        self.B = np.array([
            [0],
            [1]
            ])

    def init_curr_L(self):
        # self.L = np.array([
        #     [  1, -1,  0,  0],
        #     [ -1,  3, -1, -1],
        #     [  0, -1,  1,  0],
        #     [  0, -1,  0,  1]
        #     ])

        n = 3
        self.L =  (n - 1) * np.eye(n) - np.ones((n, n))

    def calc_lambda_L(self):
        if self.L is []:
            raise ValueError("No Laplacian Matrix")
        
        L = self.L
        
        eigvals_L = np.linalg.eigvals(L)

        # 排除接近0的特征值
        nonzero_eigs = eigvals_L[np.abs(eigvals_L) > 1e-6]          

        self.lambda2_L = np.min(nonzero_eigs)                     
        self.lambda_max_L = np.max(eigvals_L)
        self.lambda_max_L2 = np.max(np.linalg.eigvals(L @ L)) 
        
        print("lambda2(L_t) =", self.lambda2_L)
        print("lambda_max(L_t) =", self.lambda_max_L)
        print("lambda_max(L_t^2) =", self.lambda_max_L2)

    def validate_parameters(self):
        # Check if the matrix parameters are empty using numpy's size attribute
        if np.size(self.A) == 0:
            raise ValueError("Parameter A is an empty matrix!")
        if np.size(self.B) == 0:
            raise ValueError("Parameter B is an empty matrix!")
        if np.size(self.L) == 0:
            raise ValueError("Parameter L is an empty matrix!")
        
        # Check if the scalar parameters are 0
        if self.lambda2_L == 0:
            raise ValueError("Parameter lambda2_L cannot be 0!")
        if self.lambda_max_L == 0:
            raise ValueError("Parameter lambda_max_L cannot be 0!")
        if self.lambda_max_L2 == 0:
            raise ValueError("Parameter lambda_max_L2 cannot be 0!")
        
        # If all validations pass, return True
        return True

    def sol_lmi_with_global_sector(self):

        A = self.A
        B = self.B
        
        Q = cp.Variable((2, 2), symmetric=True)

        alpha = cp.Variable()
        delta = cp.Variable()
        epsilon = cp.Variable()

        LMI1 = Q @ A.T + A @ Q - epsilon * (B @ B.T)
        LMT2 = epsilon - 2*alpha*self.lambda2_L + delta*self.lambda_max_L2 - alpha**2/delta 
        # TODO: 如何解决变量delta在分母上的问题

        constraints = []
        constraints.append(LMI1 <= -1e-3 * np.eye(2))
        constraints.append(LMT2 <= -1e-6)

        objective = cp.Minimize(0)
        prob = cp.Problem(objective, constraints)
        result = prob.solve(solver=cp.SCS, verbose=True)

        print("Status:", prob.status)
        if prob.status in ["optimal", "optimal_inaccurate", "feasible"]:
            alpha = alpha.value
            Q_val = Q.value
            P_val = np.linalg.inv(Q_val)
            K_val = alpha * (B.T @ P_val)
            print("K =\n", K_val)
        else:
            print("No feasible solution found")
        
    def sol_lmi_with_local_sector(self, zeta=0.1):
        A = self.A
        B = self.B
        lambda2 = self.lambda2_L
        lambda_max_Lt2 = self.lambda_max_L2

        Q = cp.Variable((2, 2), symmetric=True)
        R = cp.Variable((1, 2))

        alpha = cp.Variable()
        delta = cp.Variable()
        epsilon = cp.Variable()

        LMT1 = 2*alpha*lambda2 - delta*lambda_max_Lt2 -epsilon
        LMI2 = 2*zeta*delta - 1

        X11 = Q @ A.T + A @ Q - epsilon * (B @ B.T)
        X12 = R.T
        X13 = np.zeros((2,1))

        X21 = R
        X22 = np.array([[-2/zeta]])
        X23 = -np.eye(1)

        X31 = np.zeros((1,2))
        X32 = -np.eye(1)
        # X33 = np.array([[-delta * zeta**2]])
        X33 = cp.reshape(-delta * zeta**2, (1, 1), order='C')

        X = cp.bmat([
            [X11, X12, X13],
            [X21, X22, X23],
            [X31, X32, X33]
        ])
        
        objective = cp.Minimize(0)

        constraints = []
        constraints.append(Q >> 0)
        constraints.append(alpha >= 1e-6)
        constraints.append(delta >= 1e-6)
        constraints.append(epsilon >= 1e-6)
        constraints.append(LMT1 >= 1e-6)
        constraints.append(LMI2 >= 1e-6)
        constraints.append(X <= -1e-6 * np.eye(4))

        prob = cp.Problem(objective, constraints)
        result = prob.solve(solver=cp.SCS, verbose=True)

        # print("Status:", prob.status)
        if prob.status in ["optimal", "optimal_inaccurate", "feasible"]:
            alpha = alpha.value
            Q_val = Q.value
            P_val = np.linalg.inv(Q_val)
            K_val = alpha * (B.T @ P_val)
            print("K =\n", K_val)

            R_val = R.value
            M_val = R_val @ P_val
            print(M_val)
        else:
            print("No feasible solution found")

    def sol_lmi_with_local_sector_ref(self, zeta=0.1):
        A = self.A
        B = self.B
        lambda2 = self.lambda2_L
        lambda_max_Lt2 = self.lambda_max_L2

        Q = cp.Variable((2, 2), symmetric=True)
        F = cp.Variable((2, 2), symmetric=True)
        R = cp.Variable((1, 2))
        R_ref = np.array([[1, 1]])

        alpha = cp.Variable()
        delta = cp.Variable()
        epsilon = cp.Variable()

        LMT1 = 2*alpha*lambda2 - delta*lambda_max_Lt2 -epsilon

        X11 = Q @ A.T + A @ Q - epsilon * (B @ B.T) + F
        X12 = R.T
        X13 = np.zeros((2,1))

        X21 = R
        X22 = np.array([[-2/zeta]])
        X23 = -np.eye(1)

        X31 = np.zeros((1,2))
        X32 = -np.eye(1)
        X33 = cp.reshape(-delta * zeta**2, (1, 1), order='C')

        X = cp.bmat([
            [X11, X12, X13],
            [X21, X22, X23],
            [X31, X32, X33]
        ])
        
        objective = cp.Minimize(0)
        # objective = cp.Minimize(cp.norm(R - R_ref, 'fro'))

        constraints = []
        constraints.append(Q >> 0)
        constraints.append(F >> 0)
        # constraints.append(cp.norm(R, 'fro') >= epsilon)
        # constraints.append(cp.abs(R) <= 10)
        constraints.append(alpha >= 1e-9)
        constraints.append(delta >= 1e-9)
        constraints.append(epsilon >= 1e-9)
        constraints.append(LMT1 >= 1e-9)
        constraints.append(X <= -1e-9 * np.eye(4))

        prob = cp.Problem(objective, constraints)
        result = prob.solve(solver=cp.SCS, verbose=True)

        print("Status:", prob.status)
        if prob.status in ["optimal", "optimal_inaccurate", "feasible"]:
            Q_val = Q.value
            P_val = np.linalg.inv(Q_val)
            alpha = alpha.value
            K_val = alpha * (B.T @ P_val)
            print("K =\n", K_val)

            R_val = R.value
            M_val = R_val @ P_val
            print(M_val)
        else:
            print("No feasible solution found")
            
    def sol_disturbance_lmi_with_local_sector_ref(self, zeta=1):
        A = self.A
        B = self.B
        lambda2 = self.lambda2_L
        lambda_max_L = self.lambda_max_L
        lambda_max_Lt2 = self.lambda_max_L2

        Q = cp.Variable((2, 2), symmetric=True)
        F = cp.Variable((2, 2), symmetric=True)
        R = cp.Variable((1, 2))

        alpha = cp.Variable()
        delta = cp.Variable()
        epsilon = cp.Variable()
        gamma = cp.Variable()

        LMT1 = 2 * alpha * lambda2 - delta * lambda_max_Lt2 - epsilon

        X11 = Q @ A.T + A @ Q - epsilon * (B @ B.T) + F
        X12 = zeta * R.T
        X13 = lambda_max_L * B
        X14 = Q
        X15 = np.zeros((2, 1))

        X21 = zeta * R
        X22 = np.array([[-2/zeta]])
        X23 = np.zeros((1, 1))
        X24 = np.zeros((1, 2))
        X25 = -np.eye(1)

        X31 = lambda_max_L * B.T
        X32 = np.zeros((1, 1))
        X33 = cp.reshape(-gamma, (1, 1), order='C')
        X34 = np.zeros((1, 2))
        X35 = np.zeros((1, 1))

        X41 = Q
        X42 = np.zeros((2, 1))
        X43 = np.zeros((2, 1))
        X44 = -gamma * np.eye(2)
        X45 = np.zeros((2,1))

        X51 = np.zeros((1, 2))
        X52 = -np.eye(1)
        X53 = np.zeros((1, 1))
        X54 = np.zeros((1, 2))
        X55 = cp.reshape(-delta, (1, 1), order='C')

        X = cp.bmat([
            [X11, X12, X13, X14, X15],
            [X21, X22, X23, X24, X25],
            [X31, X32, X33, X34, X35],
            [X41, X42, X43, X44, X45],
            [X51, X52, X53, X54, X55]
        ])

        objective = cp.Minimize(0)
        # objective = cp.Maximize(cp.norm(R - R_ref, 'fro'))

        constraints = []
        constraints.append(Q >> 0)
        constraints.append(F >> 0)
        constraints.append(alpha >= 1e-9)
        constraints.append(delta >= 1e-9)
        constraints.append(epsilon >= 1e-9)
        constraints.append(LMT1 >= 1e-9)
        constraints.append(X <= -1e-9 * np.eye(7))

        prob = cp.Problem(objective, constraints)
        result = prob.solve(solver=cp.SCS, verbose=True)

        print("Status:", prob.status)
        if prob.status in ["optimal", "optimal_inaccurate", "feasible"]:
            Q_val = Q.value
            P_val = np.linalg.inv(Q_val)
            alpha = alpha.value
            K_val = alpha * (B.T @ P_val)
            print("K =\n", K_val)

            R_val = R.value
            M_val = R_val @ P_val
            print(M_val)
        else:
            print("No feasible solution found")

    def generate_H(self, h: dict, n_t=3):
        H = np.zeros((n_t, n_t))

        for (i, j), value in h.items():
            if i < j:
                H[i, j] = -value
                H[j, i] = value
                
        for i in range(n_t):
            s = 0.0            
            for j in range(n_t):
                if i == j:
                    continue
                if i < j:
                    s += h.get((i, j), 0)
                else:
                    s += -h.get((j, i), 0)
            H[i, i] = s

        return H

    def calc_sigma_max(self, H):
        U, s, Vt = np.linalg.svd(H)
        norm_H = s[0]
        # print(norm_H)
        return norm_H

    def sol_lmi_position_based(self, A, B, H, zeta=0.1):
        A = A
        B = B
        sigma_max = self.calc_sigma_max(H)
        sigma_max_squared = sigma_max**2

        Q = cp.Variable((2, 2), symmetric=True)
        F = cp.Variable((2, 2), symmetric=True)
        R = cp.Variable((1, 2))

        delta = cp.Variable()

        X11 = Q @ A.T + A @ Q + delta* sigma_max_squared * (B @ B.T) + F
        X12 = zeta * R.T
        X13 = np.zeros((2,1))

        X21 = zeta * R
        X22 = np.array([[-2 * zeta]])
        X23 = -np.eye(1)

        X31 = np.zeros((1,2))
        X32 = -np.eye(1)
        X33 = cp.reshape(-delta, (1, 1), order='C')

        X = cp.bmat([
            [X11, X12, X13],
            [X21, X22, X23],
            [X31, X32, X33]
        ])
        
        objective = cp.Maximize(cp.trace(X))
        constraints = []
        constraints.append(Q >> 0)
        constraints.append(F >> 0)
        constraints.append(delta >= 1e-9)
        constraints.append(X <= -1e-9 * np.eye(4))

        prob = cp.Problem(objective, constraints)
        result = prob.solve(solver=cp.SCS, verbose=False)

        print("Status:", prob.status)
        if prob.status in ["optimal", "optimal_inaccurate", "feasible"]:
            Q_val = Q.value
            P_val = np.linalg.inv(Q_val)
            # print("P =\n", P_val)
            # print(B.T)
            K_val = - B.T @ P_val
            # print("K =\n", K_val)
        else:
            print("No feasible solution found")
        
        return K_val


if __name__ == "__main__":
    vcsc = VSCS()
    vcsc.init_vscs_model()
    vcsc.sol_lmi_position_based()