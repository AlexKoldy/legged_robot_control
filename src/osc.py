import numpy as np
from pydrake.all import (
    LeafSystem,
    MultibodyPlant,
    Parser,
    Context,
    BasicVector,
    MathematicalProgram,
    JacobianWrtVariable,
    OsqpSolver,
    le,
)
from typing import Tuple

from utils import PointOnFrame
from objectives.com_position_tracking_objective import (
    CenterOfMassPositionTrackingObjective,
)
from objectives.swing_foot_position_tracking_objective import (
    SwingFootPositionTrackingObjective,
)
from footstep_planner import FootstepPlanner
from finite_state_machine import FiniteStateMachine

import time


class OperationalSpaceController(LeafSystem):
    def __init__(
        self,
        filename: str,
    ):
        """
        TODO
        """
        LeafSystem.__init__(self)

        # Set up plant
        self.plant = MultibodyPlant(0.0)
        self.parser = Parser(self.plant)
        self.parser.AddModelFromFile(filename)
        self.plant.Finalize()
        self.context = self.plant.CreateDefaultContext()

        # State input port
        self.robot_state_input_port_index = self.DeclareVectorInputPort(
            "x", self.plant.num_positions() + self.plant.num_velocities()
        ).get_index()

        # Set up output port
        self.torque_output_port = self.DeclareVectorOutputPort(
            "u", self.plant.num_actuators(), self.calculate
        )

        # Contact points
        # - Taken from https://github.com/rcywongaa/humanoid/blob/master/Atlas.py
        self.contact_points = {
            "LEFT": {
                "LEFT HEEL": PointOnFrame(
                    self.plant.GetBodyByName("l_foot").body_frame(),
                    np.array([-0.0876, 0.066, -0.07645]),
                ),
                "RIGHT HEEL": PointOnFrame(
                    self.plant.GetBodyByName("l_foot").body_frame(),
                    np.array([-0.0876, -0.0626, -0.07645]),
                ),
                "LEFT TOE": PointOnFrame(
                    self.plant.GetBodyByName("l_foot").body_frame(),
                    np.array([0.1728, 0.066, -0.07645]),
                ),
                "RIGHT TOE": PointOnFrame(
                    self.plant.GetBodyByName("l_foot").body_frame(),
                    np.array([0.1728, -0.0626, -0.07645]),
                ),
            },
            "RIGHT": {
                "LEFT HEEL": PointOnFrame(
                    self.plant.GetBodyByName("r_foot").body_frame(),
                    np.array([-0.0876, 0.0626, -0.07645]),
                ),
                "RIGHT HEEL": PointOnFrame(
                    self.plant.GetBodyByName("r_foot").body_frame(),
                    np.array([-0.0876, -0.066, -0.07645]),
                ),
                "LEFT TOE": PointOnFrame(
                    self.plant.GetBodyByName("r_foot").body_frame(),
                    np.array([0.1728, 0.0626, -0.07645]),
                ),
                "RIGHT TOE": PointOnFrame(
                    self.plant.GetBodyByName("r_foot").body_frame(),
                    np.array([0.1728, -0.066, -0.07645]),
                ),
            },
        }

        # TODO: clean up gains

        # Tracking objectives
        self.tracking_objectives = {
            "com": CenterOfMassPositionTrackingObjective(
                self.plant,
                self.context,
                100 * np.eye(3),
                10 * np.eye(3),
            ),
            # "swing_foot": SwingFootPositionTrackingObjective(
            #     self.plant,
            #     self.context,
            #     100 * np.eye(3),
            #     10 * np.eye(3),
            # ),
        }

        # Tracking costs
        self.tracking_costs = {
            "com": np.eye(3),
        }

        # Set up finite state machine
        self.fsm = FiniteStateMachine(1)

        # TODO: REMOVE THIS AND DO IT PROPERLY
        planner = FootstepPlanner()

        self.start_time = time.time()

    def calculate(self, context: Context, output: BasicVector) -> None:
        """
        TODO
        """
        # For demo
        if time.time() - self.start_time <= 12:
            u = self.setup_and_solve_quadratic_program(context)
        else:
            print("OSC OFF")
            u = self.setup_and_solve_quadratic_program(context)
            u = np.zeros_like(u)

        output.SetFromVector(u)

    def setup_and_solve_quadratic_program(self, context: Context) -> np.ndarray:
        """
        TODO
        """

        # Get state
        x = self.EvalVectorInput(context, self.robot_state_input_port_index).get_value()

        # Get time and prompt state machine for support mode
        t = context.get_time()
        support_mode = self.fsm.get_state(t)

        support_mode = "DOUBLE"

        # TODO: planner inputs

        # Update the plant context with current position and velocity
        self.plant.SetPositionsAndVelocities(self.context, x)

        # Set up OSC control problem with MathematicalProgram and
        # decision variables
        prog = MathematicalProgram()
        u = prog.NewContinuousVariables(self.plant.num_actuators(), "u")
        v_dot = prog.NewContinuousVariables(self.plant.num_velocities(), "v_dot")

        if support_mode == "LEFT" or support_mode == "RIGHT":
            lambda_c = prog.NewContinuousVariables(12, "lambda_c")  # TODO
        elif support_mode == "DOUBLE":
            lambda_c = prog.NewContinuousVariables(24, "lambda_c")  # TODO
        else:
            raise Exception("Invalid support mode")

        # Add quadratic cost on desired acceleration
        for objective_name in self.tracking_objectives.keys():
            objective = self.tracking_objectives[objective_name]

            # Update tracking objective
            objective.calculate(
                # t,
            )

            y_ddot_cmd_i = objective.y_ddot_cmd
            J_i = objective.J
            J_dot_v_i = objective.J_dot_v
            W_i = self.tracking_costs[objective_name]

            Q_i = 2 * J_i.T @ W_i @ J_i
            b_i_T = (
                -y_ddot_cmd_i.T @ W_i @ J_i
                - y_ddot_cmd_i.T @ W_i.T @ J_i
                + J_dot_v_i.T @ W_i @ J_i
                + J_dot_v_i.T @ W_i.T @ J_i
            )
            c_i = (
                y_ddot_cmd_i.T @ W_i @ y_ddot_cmd_i
                - y_ddot_cmd_i.T @ W_i @ J_dot_v_i
                - J_dot_v_i.T @ W_i @ y_ddot_cmd_i
                + J_dot_v_i.T @ W_i @ J_dot_v_i
            )

            prog.AddQuadraticCost(Q_i, b_i_T, c_i, v_dot, is_convex=True)

        # Add quadratic cost on v_dot
        Q_eye = 0.00001 * np.eye(self.plant.num_velocities())
        prog.AddQuadraticCost(
            2 * Q_eye,
            np.zeros(self.plant.num_velocities()),
            v_dot,
            is_convex=True,
        )

        # Calculate terms in the manipulator equation and add
        # forward dynamics constraint
        J_c, J_c_dot_v = self.calculate_contact_jacobian(support_mode)
        M = self.plant.CalcMassMatrix(self.context)
        Cv = self.plant.CalcBiasTerm(self.context)
        G = -self.plant.CalcGravityGeneralizedForces(self.context)
        B = self.plant.MakeActuationMatrix()

        prog.AddLinearEqualityConstraint(
            M @ v_dot + Cv + G - B @ u - J_c.T @ lambda_c,
            np.zeros((self.plant.num_velocities(), 1)),
        )

        # Add Contact Constraint
        prog.AddLinearEqualityConstraint(J_c_dot_v + J_c @ v_dot, np.zeros((24, 1)))

        # Add friction cone constraint assuming mu = 1. We use a
        # linearized version of the cone to preserve the convexity
        # of the constraints
        friction_cone_constraint = np.array(
            [
                [-1, 0, -1],
                [1, 0, -1],
                [0, -1, -1],
                [0, 1, -1],
            ]
        )
        if support_mode == "LEFT" or support_mode == "RIGHT":
            A = np.kron(np.eye(4), friction_cone_constraint)
            b = np.zeros(16)
            prog.AddLinearConstraint(le(A @ lambda_c, b))

        elif support_mode == "DOUBLE":
            A = np.kron(np.eye(8), friction_cone_constraint)
            b = np.zeros(32)
            prog.AddLinearConstraint(le(A @ lambda_c, b))

        # Warmstart using previous solution (if it exists)
        try:
            prog.SetInitialGuess(u, self.u)
        except:
            pass
        try:
            prog.SetInitialGuess(v_dot, self.v_dot)
        except:
            pass
        try:
            prog.SetInitialGuess(lambda_c, self.lambda_c)
        except:
            pass

        # Solve the quadratic program
        solver = OsqpSolver()
        prog.SetSolverOption(solver.id(), "max_iter", 2000)

        result = solver.Solve(prog)

        # If we exceed iteration limits use the previous solution
        if not result.is_success():
            u_sol = self.u
        else:
            u_sol = result.GetSolution(u)
            self.u = u_sol
            self.v_dot = result.GetSolution(v_dot)
            self.lambda_c = result.GetSolution(lambda_c)

        return u_sol

    def calculate_contact_jacobian(
        self, support_mode: str
    ) -> Tuple[np.ndarray, np.ndarray]:
        """ """
        if support_mode == "LEFT" or support_mode == "RIGHT":
            J = np.zeros((12, self.plant.num_velocities()))
            J_dot_v = np.zeros((12,))

            for i, point_to_track in enumerate(
                list(self.contact_points[support_mode].values())
            ):
                J[3 * i : 3 * (i + 1), :] = (
                    self.plant.CalcJacobianTranslationalVelocity(
                        self.context,
                        JacobianWrtVariable.kV,
                        point_to_track.frame,
                        point_to_track.point,
                        self.plant.world_frame(),
                        self.plant.world_frame(),
                    )
                )

                J_dot_v[3 * i : 3 * (i + 1)] = (
                    self.plant.CalcBiasTranslationalAcceleration(
                        self.context,
                        JacobianWrtVariable.kV,
                        point_to_track.frame,
                        point_to_track.point,
                        self.plant.world_frame(),
                        self.plant.world_frame(),
                    ).ravel()
                )
        elif support_mode == "DOUBLE":
            J = np.zeros((24, self.plant.num_velocities()))
            J_dot_v = np.zeros((24,))

            for i, point_to_track in enumerate(
                list(self.contact_points["LEFT"].values())
            ):
                J[3 * i : 3 * (i + 1), :] = (
                    self.plant.CalcJacobianTranslationalVelocity(
                        self.context,
                        JacobianWrtVariable.kV,
                        point_to_track.frame,
                        point_to_track.point,
                        self.plant.world_frame(),
                        self.plant.world_frame(),
                    )
                )

                J_dot_v[3 * i : 3 * (i + 1)] = (
                    self.plant.CalcBiasTranslationalAcceleration(
                        self.context,
                        JacobianWrtVariable.kV,
                        point_to_track.frame,
                        point_to_track.point,
                        self.plant.world_frame(),
                        self.plant.world_frame(),
                    ).ravel()
                )

            for i, point_to_track in enumerate(
                list(self.contact_points["RIGHT"].values())
            ):
                J[12 + (3 * i) : 12 + (3 * (i + 1)), :] = (
                    self.plant.CalcJacobianTranslationalVelocity(
                        self.context,
                        JacobianWrtVariable.kV,
                        point_to_track.frame,
                        point_to_track.point,
                        self.plant.world_frame(),
                        self.plant.world_frame(),
                    )
                )

                J_dot_v[12 + (3 * i) : 12 + (3 * (i + 1))] = (
                    self.plant.CalcBiasTranslationalAcceleration(
                        self.context,
                        JacobianWrtVariable.kV,
                        point_to_track.frame,
                        point_to_track.point,
                        self.plant.world_frame(),
                        self.plant.world_frame(),
                    ).ravel()
                )

        return J, J_dot_v

    def get_output_port(self):
        """ """
        return self.torque_output_port

    def get_state_input_port(self):
        """"""
        return self.get_input_port(self.robot_state_input_port_index)
