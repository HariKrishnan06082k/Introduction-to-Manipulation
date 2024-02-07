import numpy as np
import matplotlib.pyplot as plt
from assignment_3_helper import LCPSolve, assignment_3_render


# DEFINE GLOBAL PARAMETERS
L = 0.4
MU = 0.3
EP = 0.5
dt = 0.01
m = 0.3
g = np.array([0., -9.81, 0.])
rg = 1./12. * (2 * L * L) #TODO: Rename this to rg_squared since it is $$r_g^2$$ - Do it also in the master
M = np.array([[m, 0, 0], [0, m, 0], [0, 0, m * rg]])
Mi = np.array([[1./m, 0, 0], [0, 1./m, 0], [0, 0, 1./(m * rg)]])
DELTA = 0.001
T = 150


def get_contacts(q):
    """
        Return jacobian of the lowest corner of the square and distance to contact
        :param q: <np.array> current configuration of the object
        :return: <np.array>, <float> jacobian and distance
    """
    # ------------------------------------------------
    # FILL WITH YOUR CODE

    jac = None  # TODO: Replace None with your result
    phi = None

    x,y,theta = q[0],q[1],q[2]
    
    r_0 = np.array([[-1*L/2,-1*L/2, 1],
                  [1*L/2,-1*L/2, 1],
                  [1*L/2,1*L/2, 1],
                  [-1*L/2,1*L/2, 1]])

    w_R_0 = np.array([[np.cos(theta), -np.sin(theta), x],
                  [np.sin(theta), np.cos(theta), y],
                  [0,0,0]])
    
    r_w = (np.dot(w_R_0,r_0.T)[:-1, :]).T
    #print(r_w)

    min_idx = np.argmin(r_w[:, 1])
    closest_corner = r_w[min_idx]
    
    phi = closest_corner[1]
    dx = closest_corner[0] - x
    dy = closest_corner[1] - y
    J_t = np.array([1, 0, -dy]) 
    J_n = np.array([0, 1, dx])
    jac = np.vstack((J_t, J_n)).T 
    # ------------------------------------------------
    return jac, phi



def form_lcp(jac, v):
    """
        Return LCP matrix and vector for the contact
        :param jac: <np.array> jacobian of the contact point
        :param v: <np.array> velocity of the center of mass
        :return: <np.array>, <np.array> V and p
    """
    # ------------------------------------------------
    # FILL WITH YOUR CODE

    Jt = jac[:, 0].reshape(-1, 1)  
    Jn = jac[:, 1].reshape(-1, 1)  

    M_inv = Mi

    JnT_M_inv = np.matmul(Jn.T, M_inv)
    JtT_M_inv = np.matmul(Jt.T, M_inv)

    JnT_M_inv_Jn = np.matmul(JnT_M_inv, Jn)
    JnT_M_inv_Jt = np.matmul(JnT_M_inv, Jt)
    JtT_M_inv_Jn = np.matmul(JtT_M_inv, Jn)
    JtT_M_inv_Jt = np.matmul(JtT_M_inv, Jt)

    V = np.array([
        [JnT_M_inv_Jn[0][0] * dt, -JnT_M_inv_Jt[0][0] * dt, JnT_M_inv_Jt[0][0] * dt, 0],
        [-JtT_M_inv_Jn[0][0] * dt, JtT_M_inv_Jt[0][0] * dt, -JtT_M_inv_Jt[0][0] * dt, 1],
        [JtT_M_inv_Jn[0][0] * dt, -JtT_M_inv_Jt[0][0] * dt, JtT_M_inv_Jt[0][0] * dt, 1],
        [MU, -1, -1, 0]
    ])

    fe = m * g  
    JnT_M_inv_fe = np.matmul(JnT_M_inv, fe)
    JtT_M_inv_fe = np.matmul(JtT_M_inv, fe)

    p1 = (np.matmul(Jn.T, v) * (1 + EP) + dt * JnT_M_inv_fe).item()
    p2 = (-np.matmul(Jt.T, v) + dt * JtT_M_inv_fe).item()
    p3 = (np.matmul(Jt.T, v) + dt * JtT_M_inv_fe).item()

    p = np.array([p1, p2, p3, 0])
    return V,p


def step(q, v):
    """
        predict next config and velocity given the current values
        :param q: <np.array> current configuration of the object
        :param v: <np.array> current velocity of the object
        :return: <np.array>, <np.array> q_next and v_next
    """
    # ------------------------------------------------
    # FILL WITH YOUR CODE

    f_e = m*g  
    
    jac, phi = get_contacts(q)

    if phi <= DELTA:  # Within contact threshold
        V, p = form_lcp(jac, v)
        f_c = lcp_solve(V, p)

        # Update velocity and position
        v_next = v + dt * np.matmul(Mi, f_e+ jac[:,1]*f_c[0] - jac[:,0]*f_c[1] + jac[:,0]*f_c[2])
        q_next = q + dt * v_next + np.array([0, DELTA, 0]) 

    else:
        # If not in contact, just f_e
        a = np.dot(Mi, f_e)
        v_next = v + dt * a
        q_next = q + dt * v_next

    # ------------------------------------------------
    return q_next, v_next


def simulate(q0, v0):
    """
        predict next config and velocity given the current values
        :param q0: <np.array> initial configuration of the object
        :param v0: <np.array> initial velocity of the object
        :return: <np.array>, <np.array> q and v trajectory of the object
    """
    # ------------------------------------------------
    # FILL WITH YOUR CODE

    q = np.zeros((3, T))  # TODO: Replace with your result
    v = np.zeros((3, T))

    q[:, 0] = q0
    v[:, 0] = v0
    
    for i in range(1, T):
        q[:, i], v[:, i] = step(q[:, i-1], v[:, i-1])
    # ------------------------------------------------
    return q, v


def lcp_solve(V, p):
    """
        DO NOT CHANGE -- solves the LCP
        :param V: <np.array> matrix of the LCP
        :param p: <np.array> vector of the LCP
        :return: renders the trajectory
    """
    sol = LCPSolve(V, p)
    f_r = sol[1][:3]
    return f_r


def render(q):
    """
        DO NOT CHANGE -- renders the trajectory
        :param q: <np.array> configuration trajectory
        :return: renders the trajectory
    """
    assignment_3_render(q)


if __name__ == "__main__":
    # to test your final code, use the following initial configs
    q0 = np.array([0.0, 1.5, np.pi / 180. * 30.])
    v0 = np.array([0., -0.2, 0.])
    q, v = simulate(q0, v0)

    plt.plot(q[1, :])
    plt.show()

    render(q)




