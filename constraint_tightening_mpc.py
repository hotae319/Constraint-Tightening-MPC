'''
© 2020 Hotae Lee <hotae.lee@berkeley.edu>
'''

# Import packages.
import cvxpy as cp
import numpy as np
import matplotlib.pyplot as plt
from set_operation import polytope
from set_operation import pontryagin_diff, multiplication, polytope_intersection
from agent import Agent

class TighteningMPC:
    def __init__(self, Target, Constr, N, agent):
        # polytopes
        self.target_init = Target
        self.constr_init = Constr
        self.N = N
        self.A = agent.A # class
        self.B = agent.B
        self.C = agent.C
        self.D = agent.D
        self.E = agent.E
        self.F = agent.F
        self.W = agent.W
        self.x0 = agent.x
        self.K = []
    def state_update(self, x_new):
        self.x0 = x_new
    def recursion_l(self):
        L_cur = np.identity((self.A.shape[0]))
        L = [L_cur]
        for i in range(self.N):  
            #print("L cur {}, {}\n".format(L_cur, i))          
            L_cur = np.round((self.A+self.B@self.K[i])@L_cur,3)
            L.append(L_cur)
        return L
    def recursion_target(self,L):     
        target_cur = self.target_init
        target = [target_cur]
        for i in range(self.N):
            #print("(E+FK)L, K {}, \n, {}".format(np.round((self.E + self.F @ self.K[i])@L[i],2),self.K[i]))
            W_temp = multiplication(self.W,(self.E + self.F @ self.K[i])@L[i])
            print("wtemp\n")
            print(np.round(W_temp.A,2),np.round(W_temp.b,2))
            target_cur = pontryagin_diff(target_cur,W_temp)
            print("target_cur")
            print(target_cur.A,target_cur.b)
            target.append(target_cur)
        return target
    def recursion_y(self,L):
        constr_cur = self.constr_init
        constr = [constr_cur]
        for i in range(self.N):
            # print("w, (c+dk)l\n")
            # print(self.W.A, (self.C + self.D @ self.K[i])@L[i])
            # print("(C+DK)L, K {}, \n, {}".format(np.round((self.C + self.D @ self.K[i])@L[i],2),self.K[i]))
            W_temp = multiplication(self.W,np.round((self.C + self.D @ self.K[i])@L[i],2))

            # print("wtemp\n")
            # print(W_temp.A,W_temp.b)
            # print("constr")
            # print(constr_cur.A,constr_cur.b)
            constr_cur = pontryagin_diff(constr_cur,W_temp)
            #print("wtemp {},{}".format(W_temp.A,W_temp.b))
            # print("constrcut,i {},{},{} \n".format(constr_cur.A,constr_cur.b,i))
            constr.append(constr_cur)
            # print("\n")
        return constr
    def nilpotent_lqr(self, Q, R, M):
        # Q,R : LQR Gain
        # M : The policy drives the nominal sys. to the origin in M steps
        K_seq = []        
        P = np.zeros((Q.shape))
        S = np.identity(self.B.T.shape[1]) 
        #print(A,B)
        for j in range(M-1,-1,-1):            
            H1 = np.concatenate((R+self.B.T@P@self.B, self.B.T@S.T),axis=1)
            H2 = np.concatenate((S@self.B,np.zeros(Q.shape)),axis=1)
            H = np.concatenate((H1,H2),axis = 0)
            H_pseudo = np.linalg.pinv(H)
            # compute K
            #K = -np.concatenate((np.identity(self.B.shape[1]),np.zeros((B.T.shape))),axis=1) @ H_pseudo @ np.concatenate((B.T@P,np.concatenate((np.ones(B.shape),S@B), axis = 1)), axis=0) @ A
            K = -np.concatenate((np.identity(self.B.shape[1]),np.zeros((self.B.T.shape))),axis=1) @ H_pseudo @ np.concatenate((self.B.T@P,S), axis=0) @ self.A
            
            K_seq.append(K)
            # compute S
            S = S@(self.A+self.B@K)
            # compute P            
            P = Q+K.T@R@K+(self.A+self.B@K)@P@(self.A+self.B@K)
        # 0,1,...M-1
        K_seq.reverse()
        K_p = np.zeros((self.B.shape[1],self.A.shape[1]))
        for j in range(M, self.N):
            K_seq.append(K_p)
        return K_seq
    def candidate_control(self, ustar, K, L, w):
        uhat = ustar +K@L@w
    def nominal_lqr(self):
        M = 0
        return M
    def solve_mpc(self, target, constr, XF):
        t = 17
        n = self.A.shape[1]
        m = self.B.shape[1]
        p = self.E.shape[0]
        q = self.C.shape[0]
        x = cp.Variable((n,self.N+2))
        u = cp.Variable((m,self.N+1))
        y = cp.Variable((q,self.N+1))        
        z = cp.Variable((p,self.N+1))
        s = cp.Variable((self.N+1,))
        c = np.ones((self.N+1,))
        cost = c.T@s
        # distance constraint
        constr_mpc = [s>=0]
        for j in range(self.N+1):            
            constr_mpc += [target[j].A@z[:,j] <= target[j].b+t*s[j]]
        # evolution constraint
        constr_mpc += [x[:,0] == self.x0]
        for j in range(self.N+1):
            constr_mpc += [x[:,j+1] == self.A@x[:,j]+self.B@u[:,j]]
            constr_mpc += [y[:,j] == self.C@x[:,j]+self.D@u[:,j]]
            constr_mpc += [z[:,j] == self.E@x[:,j]+self.F@u[:,j]]
        # constr constraint
        for j in range(self.N+1):
            constr_mpc += [constr[j].A@y[:,j]<=constr[j].b]
        # terminal set constraint
        constr_mpc += [XF.A @ x[:,-1] <= XF.b]
        problem_mpc = cp.Problem(cp.Minimize(cost), constr_mpc)
        problem_mpc.solve(verbose=False)
        #print(s.value)
        return u.value

if __name__ == "__main__":
    # disturbance set
    Aw = np.array([[1,0],[-1,0],[0,1],[0,-1]])
    bw = np.array([0.1,0.1,0.1,0.1])
    W = polytope(Aw, bw)
    # Target set
    At = np.array([[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]])
    bt = np.array([1,1,1,1,1,1])
    target_set = polytope(At,bt)
    # Output Constr set
    Ac = np.array([[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]])
    bc = np.array([1,1,1,1,1,1])
    constr_set = polytope(Ac,bc)
    # general constr
    Ax = np.array([[1,0],[-1,0],[0,1],[0,-1]])
    bx = np.array([3,3,2,2])
    Au = np.array([[1],[-1]])
    bu = np.array([1,1])
    # Define agent
    A = np.array([[1,1],[0,1]])
    B = np.array([[0.5],[1]])
    C = np.array([[1/3,0],[0,1/2],[0,0]])
    D = np.array([[0],[0],[1]])
    E = np.array([[2,0],[0,2],[0,0]])
    F = np.array([[0],[0],[3]])
    #x0 = np.array([[2],[0]])
    x0 = np.array([2,0.5])
    agent = Agent(A,B,C,D,E,F,x0,W)
    # Horizon
    N = 5
    # tightening constraint mpc
    mpc_policy = TighteningMPC(target_set, constr_set, N, agent)
    Q = np.identity(2)
    R = np.array([[5]])
    M = 3 
    mpc_policy.K = mpc_policy.nilpotent_lqr(Q,R,M)    
    #print(mpc_policy.K)
    #mpc_policy.K = [np.array([[-0.5154,-1.1038]]),np.array([[-1,-1.5]]),np.array([[-0.4,-1.2]]),np.array([[0,0]]),np.array([[0,0]])]
    L = mpc_policy.recursion_l()
    constr = mpc_policy.recursion_y(L)
    target = mpc_policy.recursion_target(L)    
    #print([constr[j].b for j in range(len(constr))])

    # Get XF
    constr_last = polytope(constr[-1].A@C, constr[-1].b)
    target_last = polytope(target[-1].A@E, target[-1].b)
    x2_zero = polytope(np.array([[0,1],[0,-1]]),np.array([0,0]))
    XF = polytope_intersection(constr_last, target_last)
    XF = polytope_intersection(XF, x2_zero)
    # AF = np.array([[1,0],[-1,0]])
    # bF = np.array([100,100])
    # XF = polytope(AF,bF)

    # T step evolution    
    T = 9
    x1_list = [x0[0]]
    x2_list = [x0[1]]
    t_list = [0]
    # Get disturbance for T steps
    w_candidate = 0.2*np.random.random((2,T))-0.1
    #print(w_candidate)
    for t in range(T):        
        # MPC control policy
        mpc_policy.state_update(agent.x)
        u = mpc_policy.solve_mpc(target, constr, XF)        
        #print(u)
        # update with u_input
        u_input = u[:,0]
        #print(u_input)
        agent.x = agent.update(u_input, w_candidate[:,t])
        x1_list.append(agent.x[0])
        x2_list.append(agent.x[1])
        t_list.append(t+1)
    print("check")
    print(x1_list)
    print(agent.x)

    fig = plt.figure(0)
    plt.plot(t_list,x1_list)
    plt.ylabel("x1", fontsize = 12)
    plt.xlabel("time" , fontsize =12, labelpad = 10)
    plt.title("x1 evolution", fontsize = 17)
    plt.legend(loc = 'best')
    fig.savefig('x1.png')

    fig1 = plt.figure(1)
    plt.plot(t_list,x2_list)
    plt.ylabel("x2", fontsize = 12)
    plt.xlabel("time" , fontsize =12, labelpad = 10)
    plt.title("x2 evolution", fontsize = 17)
    plt.legend(loc = 'best')
    fig1.savefig('x2.png')


    fig2 = plt.figure(2)
    plt.plot(x1_list,x2_list)
    plt.ylabel("x1", fontsize = 12)
    plt.xlabel("x2" , fontsize =12, labelpad = 10)
    plt.title("x1-x2 evolution", fontsize = 17)
    plt.legend(loc = 'best')
    fig2.savefig('x1x2 traj.png')

    # Iteration for 100 uncertainties
    n_iter = 100
    for i in range(n_iter):
        # agent initialization
        agent = Agent(A,B,C,D,E,F,x0,W)
        # T step evolution    
        T = 9
        x1_list = [x0[0]]
        x2_list = [x0[1]]
        t_list = [0]    
        # Get disturbance for T steps
        w_candidate = 0.2*np.random.random((2,T))-0.1
        #print(w_candidate)
        for t in range(T):  
            # MPC control policy
            mpc_policy.state_update(agent.x)
            u = mpc_policy.solve_mpc(target, constr, XF)        
            #print(u)
            # update with u_input
            u_input = u[:,0]
            #print(u_input)
            agent.x = agent.update(u_input, w_candidate[:,t])
            x1_list.append(agent.x[0])
            x2_list.append(agent.x[1])
            t_list.append(t+1)
        # print("check")
        # print(x1_list)
        # print(agent.x)

        fig = plt.figure(0)
        plt.plot(t_list,x1_list)
        plt.ylabel("x1", fontsize = 12)
        plt.xlabel("time" , fontsize =12, labelpad = 10)
        plt.title("x1 evolution", fontsize = 17)
        plt.legend(loc = 'best')
        #fig.savefig('x1_100.png')

        fig1 = plt.figure(1)
        plt.plot(t_list,x2_list)
        plt.ylabel("x2", fontsize = 12)
        plt.xlabel("time" , fontsize =12, labelpad = 10)
        plt.title("x2 evolution", fontsize = 17)
        plt.legend(loc = 'best')
        #fig1.savefig('x2_100.png')


        fig2 = plt.figure(2)
        plt.plot(x1_list,x2_list)
        plt.ylabel("x1", fontsize = 12)
        plt.xlabel("x2" , fontsize =12, labelpad = 10)
        plt.title("x1-x2 evolution", fontsize = 17)
        plt.legend(loc = 'best')
        #fig2.savefig('x1x2 traj_100.png')


    fig.savefig('x1_100_t9.png')
    fig1.savefig('x2_100_t9.png')
    fig2.savefig('x1x2 traj_100_t9.png')



