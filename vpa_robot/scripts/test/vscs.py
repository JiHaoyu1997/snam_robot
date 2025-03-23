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
        self.init_system_dynamic_matrix()

    def init_system_dynamic_matrix(self):
        self.A = np.array([
            [0, -1],
            [0,  0]
            ])

        self.B = np.array([
            [0],
            [1]
            ])
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
        
        # print("lambda2(L_t) =", self.lambda2_L)
        # print("lambda_max(L_t) =", self.lambda_max_L)
        # print("lambda_max(L_t^2) =", self.lambda_max_L2)

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

    def sol_lmi(self, L, zeta=0.1):
        self.L = L
        self.calc_lambda_L()

        A = self.A
        B = self.B
        
        lambda2 = self.lambda2_L
        lambda_max_Lt2 = self.lambda_max_L2

        Q = cp.Variable((2, 2), symmetric=True)
        H = cp.Variable((2, 2), symmetric=True)
        M = cp.Variable((1, 2))

        alpha = cp.Variable()
        delta = cp.Variable()
        epsilon = cp.Variable()

        LMT1 = 2*alpha*lambda2 - delta*lambda_max_Lt2 - epsilon

        X11 = Q @ A.T + A @ Q - epsilon * (B @ B.T) + H
        X12 = zeta * M.T
        X13 = np.zeros((2,1))

        X21 = zeta * M
        X22 = np.array([[-2*zeta]])
        X23 = -np.eye(1)

        X31 = np.zeros((1,2))
        X32 = -np.eye(1)
        X33 = cp.reshape(-delta, (1, 1), order='C')

        X = cp.bmat([
            [X11, X12, X13],
            [X21, X22, X23],
            [X31, X32, X33]
        ])
        
        objective = cp.Minimize(0)
        # objective = cp.Maximize(cp.trace(X))

        constraints = []
        constraints.append(Q >> 0)
        constraints.append(H >> 0)
        constraints.append(delta >= 1e-9)
        constraints.append(epsilon >= 1e-9)
        constraints.append(LMT1 >= 1e-9)
        constraints.append(X <= -1e-9 * np.eye(4))

        prob = cp.Problem(objective, constraints)
        result = prob.solve(solver=cp.SCS, verbose=False)

        print("Status:", prob.status)
        if prob.status in ["optimal", "optimal_inaccurate", "feasible"]:
            Q_val = Q.value
            P_val = np.linalg.inv(Q_val)
            alpha = alpha.value
            print(alpha)
            # print(np.shape(B.T), np.shape(P_val))
            K_val = alpha * (B.T @ P_val)
            # print(np.shape(K_val))
            print("K =\n", K_val)
            return K_val

            M_val = M.value
            M_val = M_val @ P_val
            # print(M_val)
        else:
            print("No feasible solution found")


if __name__ == "__main__":
    vcsc = VSCS()
    vcsc.sol_lmi()