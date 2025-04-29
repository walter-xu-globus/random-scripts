import control
import numpy as np
import scipy.linalg
import cvxpy as cp


class LinearMPC:

    def __init__(self, A, B, Q, R, horizon):
        self.dx = A.shape[0]
        self.du = B.shape[1]
        assert A.shape == (self.dx, self.dx)
        assert B.shape == (self.dx, self.du)
        self.H = horizon
        self.A = A
        self.B = B
        self.Q = Q
        self.R = R

    def compute_SM(self):
        """
        Computes the S and M matrices as defined in the ipython notebook

        All the variables you need should be class member variables already

        Returns:
            S: np.array of shape (horizon * dx, horizon * du) S matrix
            M: np.array of shape (horizon * dx, dx) M matrix

        """
        S, M = None, None

        # --- Your code here
        M = self.A
        temp = self.B
        for i in range(1,self.H):
            M = np.vstack((M, np.linalg.matrix_power(self.A, i+1)))
            temp = np.vstack((temp, np.linalg.matrix_power(self.A, i)@self.B))

        S = temp
        for i in range(1,self.H):
            col = np.vstack((np.zeros((i*self.dx,self.du)),temp[0:-i*self.dx,:]))
            S = np.hstack((S,col))
        
        # ---
        return S, M

    def compute_Qbar_and_Rbar(self):
        Q_repeat = [self.Q] * self.H
        R_repeat = [self.R] * self.H
        return scipy.linalg.block_diag(*Q_repeat), scipy.linalg.block_diag(*R_repeat)

    def compute_finite_horizon_lqr_gain(self):
        """
            Compute the controller gain G0 for the finite-horizon LQR

        Returns:
            G0: np.array of shape (du, dx)

        """
        S, M = self.compute_SM()
        Qbar, Rbar = self.compute_Qbar_and_Rbar()

        G0 = None

        # --- Your code here
        G = -np.linalg.inv(S.T@Qbar@S + Rbar) @ S.T @ Qbar @ M
        G0 = G[0,None]


        # ---

        return G0

    def compute_lqr_gain(self):
        """
            Compute controller gain G for infinite-horizon LQR
        Returns:
            Ginf: np.array of shape (du, dx)

        """
        Ginf = None
        theta_T_theta, _, _ = control.dare(self.A, self.B, self.Q, self.R)

        # --- Your code here
        Ginf = -np.linalg.inv(self.R + self.B.T@theta_T_theta@self.B) @ self.B.T @ theta_T_theta @ self.A


        # ---
        return Ginf

    def lqr_box_constraints_qp_shooting(self, x0, u_min, u_max):
        """
            Solves the finite-horizon box-constrained LQR problem using Quadratic Programing with shooting

        Args:
            x0: np.array of shape (dx,) containing current state
            u_min: np.array of shape (du,), minimum control value
            u_max: np.array of shape (du,), maximum control value

        Returns:
            U: np.array of shape (horizon, du) containing sequence of optimal controls

        """

        S, M = self.compute_SM()
        Qbar, Rbar = self.compute_Qbar_and_Rbar()
        U = None
        # --- Your code here
        x = cp.Variable((self.dx, self.H+1))
        u = cp.Variable((self.H, self.du))
        x0 = x0.reshape(self.dx, 1)

        cost = 0
        constr = []
        for t in range(self.H):
            cost += cp.quad_form(S@u + M@x0, Qbar) + cp.quad_form(u, Rbar)
            constr += [x[:,t+1] == self.A@x[:,t] + self.B@u[t], 
                       u[t] <= u_max,
                       u[t] >= u_min]
        constr += [x[:, 0] == x0[:,0]]
        prob = cp.Problem(cp.Minimize(cost), constr)
        prob.solve()

        U = u.value
        U = U.reshape(self.H, self.du)


        # ---

        return U

    def lqr_box_constraints_qp_collocation(self, x0, u_min, u_max):
        """
            Solves the finite-horizon box-constrained LQR problem using Quadratic Programing
            with collocation

        Args:
            x0: np.array of shape (dx,) containing current state
            u_min: np.array of shape (du,), minimum control value
            u_max: np.array of shape (du,), maximum control value

        Returns:
            U: np.array of shape (horizon, du) containing sequence of optimal controls
            X: np.array of shape (horizon, dx) containing sequence of optimal states

        """

        X, U = None, None

        # --- Your code here
        X = cp.Variable((self.H, self.dx))
        U = cp.Variable((self.H, self.du))

        constraints = [self.A@x0 + self.B@U[0] == X[0], U[self.H-1] <= u_max, U[self.H-1] >= u_min]
        for i in range(self.H-1):
            constraints += [
                self.A @ X[i] + self.B @ U[i+1] == X[i+1],
                U[i] <= u_max,
                U[i] >= u_min]
        cost = 0
        for t in range(self.H):
            cost += cp.quad_form(X[t], self.Q) + cp.quad_form(U[t], self.R)
        
        prob = cp.Problem(cp.Minimize(cost), constraints)
        prob.solve()
        U = U.value
        X = X.value


        # ---

        return U, X
