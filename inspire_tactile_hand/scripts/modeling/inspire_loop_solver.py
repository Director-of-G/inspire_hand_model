import math
import numpy as np
from scipy import stats
import sympy as sp
from sympy import S

from matplotlib import pyplot as plt


def get_unit_axis(theta):
    """
        return: [cos(theta), sin(theta)]
    """
    return np.array([math.cos(theta), math.sin(theta)])

def plot_line_between_P1_and_P2(P1, P2):
    plt.plot([P1[0], P2[0]], [P1[1], P2[1]])

class FingerParams(object):
    def __init__(self):
        """
            linkage length Lx in meters
            linkage angle Ax in radians
        """
        
        # mutable length
        self.L12 = 0.0

        # fixed length
        self.L0 = 0.0
        self.L3 = 0.0
        self.L4 = 0.0
        self.L5 = 0.0
        self.L7 = 0.0
        self.L8 = 0.0

        # mutable angles
        self.A120 = 0.0
        self.A78 = 0.0
        self.A47 = 0.0
        self.A58 = 0.0
        self.A09 = 0.0

        # fixed angles
        self.G1 = 0.0
        self.G2 = 0.0
        self.L_j12 = 0.0
        self.R_j01 = 0.0
        self.R_j23 = 0.0
        self.R_j34 = 0.0
        self.R_j05 = 0.0

class FingerKinematics(object):
    def __init__(self, params:FingerParams):
        self.params_ = params

    def compute_LandA_from_state(self, state):
        """
            Compute the linkage lengths and angles from the model state
        """
        prismatic_length = state[0]
        self.params_.L12 = prismatic_length + self.params_.L_j12

    def compute_state_from_LandA(self):
        """
            Compute the model state from linkage lengths and angles
        """
        q1 = self.params_.A120 - self.params_.R_j01                 # joint?01
        
        L0 = self.params_.L0
        L3 = self.params_.L3
        L12 = self.params_.L12
        A123 = np.arccos((L3**2 + L12**2 - L0**2) / (2*L3*L12))
        q2 = A123 - self.params_.R_j23                              # joint?23

        q3 = self.params_.R_j34 - self.params_.A47                  # joint?34

        q4 = self.params_.R_j05 - self.params_.A58                  # joint?05

        return np.array([q1, q2, q3, q4])

    def solve_triangle_mechanics(self):
        """
            Solve the triangle loop from linkage params
        """
        L0 = self.params_.L0
        L12 = self.params_.L12
        L3 = self.params_.L3
        self.params_.A120 = np.arccos((L0**2 + L12**2 - L3**2) / (2 * L0 * L12))

    def solve_four_linkage_mechanics(self):
        """
            Solve the four-linkage loop from linkage params
        """
        L0 = self.params_.L0
        L12 = self.params_.L12
        L3 = self.params_.L3

        G1 = self.params_.G1
        G2 = self.params_.G2

        A03_ = np.arccos((L0**2 + L3**2 - L12**2) / (2 * L0 * L3))
        A78_ = np.pi - (2*np.pi - G1 - G2 - A03_)
        self.params_.A78 = A78_

        # print("A78: ", A78_)

        L4 = self.params_.L4
        L5 = self.params_.L5
        L7 = self.params_.L7
        L8 = self.params_.L8

        A47, A58 = sp.symbols('A47 A58')
        eq = [
                L4 * sp.cos(A47) - L5 * sp.cos(A58) + L8 + L7 * math.cos(A78_),
                L4 * sp.sin(A47) - L5 * sp.sin(A58) + L7 * math.sin(A78_)
            ]

        result = sp.solve(eq, [A47, A58])

        assert len(result) == 2

        if abs(result[0][0]) < abs(result[1][0]):
            self.params_.A47 = result[0][0]
            self.params_.A58 = result[0][1]
        else:
            self.params_.A47 = result[1][0]
            self.params_.A58 = result[1][1]
        # print("A47: ", self.params_.A47)

    def plot_linkage_structure(self):
        """
            Plot the linkage structure
        """
        plt.clf()

        pA = np.array([0, 0])
        pB = pA + self.params_.L0 * get_unit_axis(np.pi)
        pC = pB + self.params_.L7 * get_unit_axis(-self.params_.G2+np.pi+self.params_.A78)
        pD = pA + self.params_.L12 * get_unit_axis(np.pi-self.params_.A120)
        pE = pB + self.params_.L8 * get_unit_axis(-self.params_.G2)
        pF = pE + self.params_.L5 * get_unit_axis((np.pi-self.params_.G2)+self.params_.A58)

        # print("lCF: ", np.linalg.norm(pC-pF))

        dots = np.array([pA, pB, pC, pD, pE, pF])

        plt.scatter(dots[:, 0], dots[:, 1])
        
        plot_line_between_P1_and_P2(pA, pB)
        plot_line_between_P1_and_P2(pB, pC)
        plot_line_between_P1_and_P2(pA, pD)
        plot_line_between_P1_and_P2(pB, pD)
        plot_line_between_P1_and_P2(pB, pE)
        plot_line_between_P1_and_P2(pE, pF)
        plot_line_between_P1_and_P2(pC, pF)

        plt.gca().set_aspect("equal")
        plt.xlim(-0.15, 0.01)
        plt.ylim(-0.05, 0.05)

        plt.show()


# Basic linkage solver
def FourLinkageSolver(object):
    """
        The linkage should be restricted in the cross shape
        L1 crosses L3
    """
    def __init__(self, L1, L2, L3, L4):
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3
        self.L4 = L4

    def solve(self, A1):
        L1 = self.L1
        L2 = self.L2
        L3 = self.L3
        L4 = self.L4

        A2, A3 = sp.symbols('A2 A3')
        A4 = A2 + A3 - A1
        
        eq = [
                L1 * sp.cos(A1) + L2 * sp.cos(A1 + (np.pi - A2)) - (L4 + L3 * sp.cos(np.pi - A4)),
                L1 * sp.sin(A1) + L2 * sp.sin(A1 + (np.pi - A2)) - (L3 * sp.sin(np.pi - A4))
            ]
        result = sp.solve(eq, [A2, A3])

        assert len(result) == 2

        for ri in result:
            a1 = A1
            a2, a3 = ri[0], ri[1]
            a4 = a2 + a3 - a1
            # check all constraints
            if 0 < a2 < np.pi and 0 < a3 < np.pi and \
                a2 + a3 < np.pi and a1 + a4 < np.pi:
                return np.array([a1, a2, a3, a4])


def ThreeLinkageSolver(object):
    def __init__(self, L1, L2, L3):
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3

    def solve(self, L2):
        L1 = self.L1
        L2 = L2
        L3 = self.L3
        
        assert abs(L1 - L3) < L2 < L1 + L3

        a1 = np.arccos((L1**2 + L3**2 - L2**2) / (2 * L1 * L3))
        a2 = np.arccos((L1**2 + L2**2 - L3**2) / (2 * L1 * L2))
        a3 = np.pi - a1 - a2

        return np.array([a1, a2, a3])


def ThumbKinematics(object):
    """
        We need to get the zero-point angles
    """
    def __init__(self, L43, L44, L45_46, L47, L48, L49, L50, L51, L52, L53, L54):
        self.tri_loop = ThreeLinkageSolver(
            L1=L51, L2=L45_46, L3=L52
        )
        self.four_loop_1 = FourLinkageSolver(
            L1=L44, L2=L43, L3=L50, L4=L53
        )
        self.four_loop_2 = FourLinkageSolver(
            L1=L47, L2=L48, L3=L49, L4=L54
        )
        self.gamma1 = 0.0
        self.gamma2 = 0.0
        self.gamma3 = 0.0
        self.gamma4 = 0.0

    def set_zero_point(self):
        """
            Set the angles when all joints are zero
        """
        self.J434_0 = 0.0
        self.J445_0 = 0.0
        self.J456_0 = 0.0       # prismatic
        self.J467_0 = 0.0       # fixed
        self.J478_0 = 0.0
        self.J4350_0 = 0.0      # separate
        self.J449_0 = 0.0       # separate

    def set_gamma(self, gamma):
        assert len(gamma) == 4
        self.gamma1, self.gamma2, self.gamma3, self.gamma4 = gamma

    def set_input(self, J456):
        self.L45_46 = J456 + self.J456_0

    def solve(self):
        tri_result = self.tri_loop.solve(self.L45_46)
        A51_52, A51_4546, A52_4546 = tri_result

        A44_53 = A51_52 + self.gamma3 + self.gamma4
        four_result_1 = self.four_loop_1.solve(A44_53)
        _, A43_44, A43_50, A50_53 = four_result_1

        A47_54 = 2*np.pi - (self.gamma1+self.gamma2+self.gamma3+self.gamma4) - \
                    A51_52
        four_result_2 = self.four_loop_2.solve(A47_54)
        _, A47_48, A48_49, A49_54 = four_result_2

        # solve angles
        J434 = self.J434_0 - A43_44
        J445 = self.J445_0 - A51_4546
        J456 = self.L45_46
        J467 = self.J467_0
        J478 = A47_48 - self.J478_0
        J4350 = A43_50 - self.J4350_0
        J449 = A49_54 - self.J449_0


def ThumbRootKinematics(object):
    """
        We need to get the zero-point angles
    """
    def __init__(self, L41_42, L55, L56):
        self.tri_loop = ThreeLinkageSolver(
            L1=L56, L2=L41_42, L3=L55
        )

    def set_zero_point(self):
        """
            Set the angles when all joints are zero
        """
        self.J401_0 = 0.0
        self.J412_0 = 0.0       # prismatic
        self.J423_0 = 0.0
    
    def set_input(self, J412):
        self.L41_42 = J412 + self.J412_0

    def solve(self):
        tri_result = self.tri_loop.solve(self.L41_42)
        A56_55, A56_4142, A55_4142 = tri_result

        # solve angles
        J401 = self.J401_0 - A56_4142
        J412 = self.L41_42
        J423 = A56_55 - self.J423_0

        return np.array([J401, J412, J423])


index_params = FingerParams()
index_params.L0 = 104.10e-3
index_params.L3 = 7.00e-3
index_params.L4 = 8.00e-3
index_params.L5 = 36.5e-3
index_params.L7 = 36.46e-3
index_params.L8 = 8.07e-3

index_params.G1 = 2.3391
index_params.G2 = 1.0013
index_params.L_j12 = 100.0e-3
index_params.R_j01 = 0.05561
index_params.R_j23 = 2.16847
index_params.R_j34 = -1.26630
index_params.R_j05 = 0.75929

index_params.L12 = 100.00e-3

index_kin = FingerKinematics(index_params)
length_arr = np.linspace(-0.0005, 0.0089, 20)
jpos_arr = np.zeros((len(length_arr), 4))

for idx, length in enumerate(length_arr):
    print("{0}/{1}".format(idx, len(length_arr)))
    index_kin.compute_LandA_from_state(state=[length])
    index_kin.solve_triangle_mechanics()
    index_kin.solve_four_linkage_mechanics()
    # index_kin.plot_linkage_structure()
    jpos_arr[idx] = index_kin.compute_state_from_LandA()

plt.figure()

for i in range(4):
    plt.plot(length_arr, jpos_arr[:, i], label="q{}".format(i+1))
    result = stats.linregress(length_arr, jpos_arr[:, i])
    print("slope={0}, intercept={1}, rvalue={2}".format(result.slope, result.intercept, result.rvalue))
plt.legend()
plt.show()
