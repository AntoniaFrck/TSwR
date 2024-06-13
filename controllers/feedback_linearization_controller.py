import numpy as np
from models.manipulator_model import ManiuplatorModel
from .controller import Controller


class FeedbackLinearizationController(Controller):
    def __init__(self, Tp):
        self.model = ManiuplatorModel(Tp)

    def calculate_control(self, x, q_r, q_r_dot, q_r_ddot):
        q1, q2, q1_dot, q2_dot = x
        M = self.model.M(x)
        C = self.model.C(x)
        q_ddot_r = q_r_ddot
        v = q_ddot_r - np.dot(C, np.array([q1_dot, q2_dot]))

        tau = np.dot(M, v)

        return tau


