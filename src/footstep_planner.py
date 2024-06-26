import numpy as np
from pydrake.all import MathematicalProgram, OsqpSolver
from typing import Tuple, Dict
from scipy.interpolate import CubicSpline


class FootstepPlanner:
    # TODO: COMMENT THIS  BACK IN
    # def __init__(
    #     self,
    #     robot_params: Dict[str, float],
    #     planner_params: Dict[str, float],
    #     weights: Dict[str, np.ndarray],
    # ) -> None:
    #     """
    #     Model predictive control (MPC)-based footstep planner using linear inverted
    #     pendulum (LIP) model for efficient planning
    #     """

    #     self.z_0 = robot_params["com_height"]  # [m]
    #     self.yaw = robot_params["yaw"]  # [rad] TODO: remove?
    #     self.s = robot_params["starting_stance_foot"]
    #     self.d = robot_params["axial_foot_distance"] / 2  # [m]

    #     self.dt = planner_params["swing_duration"]  # [s]
    #     self.N = planner_params["horizon"]

    #     self.Q_q = weights["Q_q"]
    #     self.Q_dq = weights["Q_dq"]
    #     self.Q_p = weights["Q_p"]
    #     self.Q_dp = weights["Q_dp"]

    #     # LIP Dynamics
    #     self.A, self.B = self.generate_dynamics(self.dt)

    #     # Debugging
    #     self.enable_plotting = False
    #     self.use_predicted_results = True

    # TODO: COMMENT THIS OUT
    def __init__(self):
        pass

    def calculate(
        self,
        q_com: np.ndarray,
        q_dot_com: np.ndarray,
        p: np.ndarray,
        v_des: np.ndarray,
        dt: float,
        t: float,
    ) -> np.ndarray:
        """
        TODO
        """
        # q_0 is the 4x1 state associated with the LIP dynmaics which
        # constists of [x_com, y_com, v_x_com, v_y_com]
        q_0 = np.array([q_com[0], q_com[1], q_dot_com[0], q_dot_com[1]])
        p_next = self.setup_and_solve_quadratic_program(q_0, p_0, v_des, dt)

    def setup_and_solve_quadratic_program(
        self, q_0: np.ndarray, p_0: np.ndarray, v_des: np.ndarray, dt: float
    ) -> np.ndarray:
        """
        TODO
        """
        # Set up optimization parameters
        prog = MathematicalProgram()
        q = np.zeros((4, self.N), dtype="object")
        for i in range(self.N):
            q[:, i] = prog.NewContinuousVariables(4, "q_" + str(i))
        p = np.zeros((2, self.N), dtype="object")
        for i in range(self.N):
            p[:, i] = prog.NewContinuousVariables(2, "p_" + str(i))

        # Extract desired parameters
        v_x_des, v_y_des, omega_des = v_des  # desired velocities [m/s]
        dx = 2 * v_x_des * self.dt  # change in x-position [m]
        dy = 2 * v_y_des * self.dt  # change in y-position [m]
        dtheta = omega_des * self.dt  # change in heading [rad]

        # Initial condition constraint. We must consider that the dynamics
        # matrices used to generate q_1 might use a shorter dt since we
        # run the MPC at a faster rate. Thus, we might already be inside
        # the swing phase
        A_0, B_0 = self.generate_dynamics(dt)
        q_1 = A_0 @ q_0 + B_0 @ p_0
        prog.AddBoundingBoxConstraint(q_1, q_1, q[:, 0].flatten())

        # LIP Dynamics Constraint
        for i in range(self.N - 1):
            for j in range(4):
                prog.AddLinearEqualityConstraint(
                    q[:, i + 1].flatten()[j]
                    == (self.A @ q[:, i] + self.B @ p[:, i]).flatten()[j]
                )

        # TODO: remove maybe???
        p_des_his = np.zeros((2, self.N))

        # Cost
        def R(theta: float) -> np.ndarray:
            """
            Helper function to calculate rotation between global
            and body coordinate frames

            Arguments:
                theta (float) - heading (yaw angle) [rad]

            Return:
                (np.ndarray) - rotation vector
            """
            return np.array([np.cos(theta), np.sin(theta)])

        theta_0 = self.yaw
        p_des_0 = p_0
        m_0 = p_des_0 + self.d * R(theta_0 + self.s * np.pi / 2)
        theta_im1 = theta_0
        m_im1 = m_0
        for i in range(0, self.N - 1):
            theta_i = theta_im1 + dtheta
            m_i = m_im1 + dx * R(theta_im1) + dy * R(theta_im1 + np.pi / 2)
            p_des_i = m_i + self.d * R(theta_0 - self.s * (-1) ** (i + 1) * np.pi / 2)
            p_des_his[:, i] = p_des_i
            v_i = v_x_des * R(theta_i) + v_y_des * R(theta_i + np.pi / 2)

            theta_ip1 = theta_i + dtheta
            m_ip1 = m_i + dx * R(theta_i) + dy * R(theta_i + np.pi / 2)
            p_des_ip1 = m_ip1 + self.d * R(
                theta_0 - self.s * (-1) ** (i + 2) * np.pi / 2
            )

            prog.AddCost(
                (q[0:2, i + 1] + q[0:2, i] - 2 * m_i).T
                @ self.Q_q
                @ (q[0:2, i + 1] + q[0:2, i] - 2 * m_i)
            )
            prog.AddCost(
                ((q[0:2, i + 1] - q[0:2, i]) - self.dt * v_i).T
                @ self.Q_dq
                @ ((q[0:2, i + 1] - q[0:2, i]) - self.dt * v_i)
            )
            prog.AddCost((p[:, i] - p_des_i).T @ self.Q_p @ (p[:, i] - p_des_i))
            prog.AddCost(
                ((p[:, i + 1] - p[:, i]) - (p_des_ip1 - p_des_i)).T
                @ self.Q_dp
                @ ((p[:, i + 1] - p[:, i]) - (p_des_ip1 - p_des_i))
            )
            theta_im1 = theta_i
            m_im1 = m_i

        # Solve
        solver = OsqpSolver()
        result = solver.Solve(prog)
        com_xy_location = result.GetSolution(q)[0:2, :]
        com_xy_velocity = result.GetSolution(q)[2:4, :]
        footstep_xy_location = result.GetSolution(p)

        if self.enable_plotting:
            # print(com_xy_location[:, -3])
            # print(com_xy_velocity[:, -3])
            # print(footstep_xy_location[:, -4])

            import matplotlib.pyplot as plt

            # plt.figure()
            # plt.scatter(np.arange(1, self.N + 1, 1), footstep_xy_location[0, :], label="x")
            # plt.scatter(np.arange(1, self.N + 1, 1), footstep_xy_location[1, :], label="y")
            # plt.legend()
            # plt.show()

            plt.figure()
            plt.title("CoM and Foot Position for LIP-base MPC")
            plt.xlabel("x")
            plt.ylabel("y")
            plt.scatter(q_0[0], q_0[1], label="original CoM location")
            plt.scatter(p_0[0], p_0[1], label="original location of single foot")
            plt.scatter(
                p_des_his[0, :-1], p_des_his[1, :-1], label="desired foot location"
            )
            # plt.scatter(footstep_xy_location[0, 0], footstep_xy_location[1, 0], label="feet")
            # plt.scatter(com_xy_location[0, 0], com_xy_location[1, 0], label="com")

            plt.scatter(
                footstep_xy_location[0, :-1],
                footstep_xy_location[1, :-1],
                label="foot location",
            )
            plt.scatter(com_xy_location[0, :-1], com_xy_location[1, :-1], label="com")

            # plt.scatter(p_des_his[0, :-1], p_des_his[1, :-1], label="feet")

            # plt.legend()
            plt.show()

        if self.use_predicted_results:
            next_footstep_location = p_des_his[:, 0]
            return next_footstep_location
        else:
            next_footstep_location = footstep_xy_location[:, 0]
            return next_footstep_location

    def generate_swing_trajectory(
        t_i: float,
        t_f: float,
        p_i: np.ndarray,
        p_f: np.ndarray,
        max_height: float,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        TODO
        """
        t_m = (t_f - t_i) / 2
        x_i, y_i, z_i = p_i
        x_f, y_f, z_f = p_f

        x_m = (x_f - x_i) / 2
        y_m = (y_f - y_i) / 2
        z_m = max_height

        x_traj = CubicSpline(
            np.array([t_i, t_m, t_f]), np.array([x_i, x_m, x_f]), bc_type="clamped"
        )
        y_traj = CubicSpline(
            np.array([t_i, t_m, t_f]), np.array([y_i, y_m, y_f]), bc_type="clamped"
        )
        z_traj = CubicSpline(
            np.array([t_i, t_m, t_f]), np.array([z_i, z_m, z_f]), bc_type="clamped"
        )

        return x_traj, y_traj, z_traj

    def generate_dynamics(self, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        TODO
        """
        h = np.exp(dt / self.tau)
        A = np.array(
            [
                [
                    (1 / (2 * h)) + (h / 2),
                    0,
                    (-self.tau / (2 * h)) + (self.tau * h / 2),
                    0,
                ],
                [
                    0,
                    (1 / (2 * h)) + (h / 2),
                    0,
                    (-self.tau / (2 * h)) + (self.tau * h / 2),
                ],
                [
                    (h / (2 * self.tau)) - (1 / (2 * self.tau * h)),
                    0,
                    (h / 2) + (1 / (2 * h)),
                    0,
                ],
                [
                    0,
                    (h / (2 * self.tau)) - (1 / (2 * self.tau * h)),
                    0,
                    (h / 2) + (1 / (2 * h)),
                ],
            ]
        )  # autonomy matrix
        B = np.array(
            [
                [(-1 / (2 * h)) - (h / 2) + 1, 0],
                [0, (-1 / (2 * h)) - (h / 2) + 1],
                [(1 / (2 * self.tau * h)) - (h / (2 * self.tau)), 0],
                [0, (1 / (2 * self.tau * h)) - (h / (2 * self.tau))],
            ]
        )  # control matrix

        return A, B


if __name__ == "__main__":
    planner = FootstepPlanner()
    planner.setup_and_solve_quadratic_program(
        np.zeros(4), np.array([0, -0.25]), np.array([0.2, 0, 0]), 0
    )


# TODO: For reference
# # LIP parameters
# self.g = 9.81  # acceleration due to gravity [m/s^2]
# self.z_0 = 1  # [m]
# self.tau = np.sqrt(self.z_0 / self.g)  # [s]

# # TODO
# self.yaw = 0

# # Footstep planner parameters
# self.d = 0.25  # half of distance between left and right foot [m]
# self.dt = 0.5  # swing duration [s]
# self.s = 1  # starting stance foot
# self.N = 10  # horizon

# # LIP Dynamics
# self.A, self.B = self.generate_dynamics(self.dt)

# # Weights
# self.Q_q = np.eye(2)
# self.Q_dq = np.eye(2)
# self.Q_p = np.eye(2)
# self.Q_dp = np.eye(2)
