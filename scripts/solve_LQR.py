import numpy as np
from scipy.linalg import solve_continuous_are


J = np.diag([0.08, 0.08, 0.12])
J_inv = np.linalg.inv(J)
alpha = (0.011 + 0.027) / 2 # From paper

A = np.vstack((
        np.hstack((np.zeros((3,3)), J_inv)),
        np.hstack((np.zeros((3,3)), -np.eye(3)/alpha))
))
B = np.vstack((np.zeros((3,3)), np.eye(3) / alpha))
Q = np.diag([1, 1, 1, 1, 1, 1.0])
R = np.diag([40, 40, 400.0])

P = solve_continuous_are(A, B, Q, R)
print("p")
print(P)
K = np.linalg.inv(R) @ B.T @ P
print("k")
print(K)
