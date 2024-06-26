def SetupAndSolveQP(self,  context: Context) -> Tuple[np.ndarray, MathematicalProgram]:
        # Get state and current time
        x = self.EvalVectorInput(context, self.robot_state_input_port_index).get_value()
        t = context.get_time()

        # Update the plant context with the current position and velocity
        self.plant.SetPositionsAndVelocities(self.plant_context, x)

        # Set up MathematicalProgram to solve QP
        prog = MathematicalProgram()

        # Make decision variables
        u = prog.NewContinuousVariables(self.plant.num_actuators(), "u")
        v_dot = prog.NewContinuousVariables(self.plant.num_velocities(), "v_dot")
        lambda_c = prog.NewContinuousVariables(3, "lambda_c")

        # Add quadratic cost on desired acceleration
        for objective_name in self.tracking_objectives:
            objective = self.tracking_objectives[objective_name]

            # Update tracking objective
            objective.update()

            
            y_ddot_cmd_i = objective.y_ddot_cmd
            J_i = objective.J
            J_dot_v_i = objective.J_dot_v
            W_i = self.tracking_costs[traj_name]
            
            Q_i = 2 * J_i.T @ W_i @ J_i
            b_i_T = -y_ddot_cmd_i.T @ W_i @ J_i - y_ddot_cmd_i.T @ W_i.T @ J_i + J_dot_v_i.T @ W_i @ J_i + J_dot_v_i.T @ W_i.T @ J_i
            c_i = y_ddot_cmd_i.T @ W_i @ y_ddot_cmd_i - y_ddot_cmd_i.T @ W_i @ J_dot_v_i - J_dot_v_i.T @ W_i @ y_ddot_cmd_i + J_dot_v_i.T @ W_i @ J_dot_v_i

            prog.AddQuadraticCost(Q_i, b_i_T, c_i, v_dot, is_convex=True)

        # TODO: change this description
        # Add quadratic cost on v_dot using self.gains.w_v_dot * Identity
        Q_eye = self.gains.w_v_dot * np.eye(self.plant.num_velocities())
        prog.AddQuadraticCost(2 * Q_eye, np.zeros(self.plant.num_velocities()), v_dot, is_convex=True)

        # Calculate terms in the manipulator equation
        J_c, J_c_dot_v = self.CalculateContactJacobian(fsm)
        M = self.plant.CalcMassMatrix(self.plant_context)
        Cv = self.plant.CalcBiasTerm(self.plant_context)

        G = -self.plant.CalcGravityGeneralizedForces(self.plant_context)  
        B = self.plant.MakeActuationMatrix()
        
        prog.AddLinearEqualityConstraint(M @ v_dot + Cv + G - B @ u - J_c.T @ lambda_c, np.zeros((self.num_positions(), 1)))

        # TODO: Add Contact Constraint
        # for i in range(3):
        #     prog.AddLinearEqualityConstraint((J_c @ vdot)[i] == -J_c_dot_v[i])
        prog.AddLinearEqualityConstraint(J_c_dot_v + J_c @ vdot, np.zeros((3, 1)))

        # TODO: Add Friction Cone Constraint assuming mu = 1
        A = np.array([[-1, 0, -1], [0, 0, 0], [1, 0, -1]])
        b = np.array([0, 0, 0])
        prog.AddLinearConstraint((A @ lambda_c)[0] <= b[0])
        prog.AddLinearConstraint((A @ lambda_c)[1] <= b[1])
        prog.AddLinearConstraint((A @ lambda_c)[2] <= b[2])


        # Solve the QP
        solver = OsqpSolver()
        prog.SetSolverOption(solver.id(), "max_iter", 2000)

        result = solver.Solve(prog)

        # If we exceed iteration limits use the previous solution
        if not result.is_success():
            usol = self.u
        else:
            usol = result.GetSolution(u)
            self.u = usol


        return usol, prog
