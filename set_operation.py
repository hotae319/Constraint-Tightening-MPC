# Set Operation 
import numpy as np
import cvxpy as cp 
import scipy as sp
class polytope:
    """Polytope : Ax <= b"""
    def __init__(self, A, b):
        self.A = A
        self.b = b
    def inequality(self, x):
        check = self.A@x - self.b 
        if chech <= 0:
            status = True
        else:
            status = False
        return status
    def generate(self):
        return 1

def solve_lp(c, A, b):
    # max cx s.t. Ax <= b
    x = cp.Variable((len(c),))
    cost = c@x
    constr = [A@x<=b]
    problem = cp.Problem(cp.Maximize(cost), constr)
    problem.solve()
    return problem.value, x.value

def pontryagin_diff(P, Q):
    # arg : P, Q (polytopes)
    n = P.A.shape[0]
    H = np.zeros((n,))
    for i in range(n):
        # solve LP to get H := max p^T x
        obj, x = solve_lp(P.A[i], Q.A, Q.b) 
        H[i] = obj
    #print(H)
    Diff = polytope(P.A, P.b-H)
    return Diff

def multiplication(P, L):
    # P : polytope, L : matrix
    # assume L is pseudo invertibl
    if L.shape[0] == L.shape[1]:
        C = P.A@np.linalg.pinv(L)
        d = P.b
    else:
        C1 = P.A@np.linalg.pinv(L)
        d1 = P.b
        C2_temp = sp.linalg.null_space(L.T).T
        d2_temp = np.zeros((C2_temp.shape[0],1))
        C2 = np.concatenate((C2_temp,-C2_temp),axis = 0)
        d2 = np.concatenate((d2_temp,d2_temp),axis=0)
        d2 = np.squeeze(d2)
        C = np.concatenate((C1,C2),axis = 0)
        d = np.concatenate((d1,d2),axis = 0)
    Mul = polytope(C, d)
    return Mul

if __name__ == "__main__":
    A = np.array([[1, 0],[-1,0],[0,1],[0,-1]])
    b = np.array([6,-6,6,-6])
    #print(A.shape, b.shape)
    C = np.array([[1, 0],[-1,0],[0,1],[0,-1]])
    d = np.array([2,-2,2,-2])
    P = polytope(A,b)
    Q = polytope(C,d)
    diff = pontryagin_diff(P,Q)
    print(diff)
    print(diff.A)
    L = np.array([[1,2],[3,4]])
    mul = multiplication(P, L)
    print(mul.A, mul.b)

