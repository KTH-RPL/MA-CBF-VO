classdef h_cbf_system < matlab.System
    % h_cbf_system Implements a controller for Simulink using a CBF-VO approach.
    %   This class uses Control Barrier Functions (CBF) together with a
    %   Velocity Obstacle (VO) method as described in:
    %   https://arxiv.org/abs/2409.10117
    %   It drives the agent toward its goal using a PID controller while
    %   filtering the control inputs to avoid collisions.
    
    %% Public (Nontunable) Properties
    properties (Nontunable)
        id = 0;                 % Unique agent ID (integer)
        config = struct();      % General simulation configuration
        agents_info = {};       % Cell array with information for every agent
    end

    %% Private Properties (Internal States)
    properties (Access = private)
        u_optim;                % Optimization variable (control input)
        u_symbolic;             % Symbolic variable for checking inequalities
        
        lambda_optim;           % Slack variables for soft constraints
        
        t_prev;                 % Previous timestep
        u_prev;                 % Previous control input
        
        p_err_prev;             % Previous position error (p_goal - p)
        
        problem;                % Optimization problem structure
        
        current_goal = 1;       % Current goal index among available sub-goals
        
        step_function_handler;  % Function handle for step execution
        
        max_iterations_fail;    % Maximum iterations allowed before failure

        n_obstacles = 0;        % Number of obstacles in the environment
    end

    %% Protected Methods
    methods (Access = protected)

        function [sz_u, sz_g, sz_c, sz_f] = getOutputSizeImpl(obj)
            % getOutputSizeImpl Return the size of each output port.
            sz_u = [2 1];
            sz_g = [1 1];
            sz_c = [1 1];
            sz_f = [1 1];
        end

        function [u_tp, g_tp, c_tp, f_tp] = getOutputDataTypeImpl(obj)
            % getOutputDataTypeImpl Return the data type of each output port.
            u_tp = "double";
            g_tp = "boolean";
            c_tp = "double";
            f_tp = "boolean";
        end

        function [u_c, g_c, c_c, f_c] = isOutputComplexImpl(obj)
            % isOutputComplexImpl Specify that outputs are not complex.
            u_c = false;
            g_c = false;
            c_c = false;
            f_c = false;
        end

        function [u_fx, g_fx, c_fx, f_fx] = isOutputFixedSizeImpl(obj)
            % isOutputFixedSizeImpl Specify that outputs have fixed sizes.
            u_fx = true;
            g_fx = true;
            c_fx = true;
            f_fx = true;
        end

        function setupImpl(obj)
            % setupImpl Initialize internal states and variables.
            obj.resetImpl();
            
            obj.u_optim = optimvar("u_optim", 2, 1);
            obj.u_symbolic = sym('u_%d', [2 1], 'real');
            
            obj.n_obstacles = numel(obj.agents_info) - 1; % Exclude itself
            if obj.n_obstacles > 0
                obj.lambda_optim = optimvar("lambda_optim", obj.n_obstacles, 1, "LowerBound", 0);
            end
            
            obj.problem = optimproblem;
            
            if obj.agents_info{obj.id}.passive
                obj.step_function_handler = @obj.passiveStep;
            else
                obj.step_function_handler = @obj.controlStep;
            end
        end

        function resetImpl(obj)
            % resetImpl Reset internal states at the start of simulation.
            obj.t_prev = 0;
            obj.u_prev = [0; 0];
            obj.p_err_prev = [0; 0];
            obj.current_goal = 1;
            obj.max_iterations_fail = obj.config.max_iterations_fail;
        end

        function [u, goal_reached, cost, exit_flag] = stepImpl(obj, t, x, X_obs)
            % stepImpl Execute one simulation step.
            obj.t_prev = t;
            [u, goal_reached, cost, exit_flag] = obj.step_function_handler(t, x, X_obs);
        end

        function [u, goal_reached, cost, exit_flag] = passiveStep(~, ~, ~, ~)
            % passiveStep For passive agents: zero acceleration, goal always reached.
            u = [0; 0];
            goal_reached = true;
            cost = 0;
            exit_flag = false;
        end

        function [u, goal_reached, cost, exit_flag] = controlStep(obj, t, x, X_obs)
            % controlStep Compute control input using optimization with CBF.
            exit_flag = false;  % Indicator for smooth simulation termination
            
            % Extract position and velocity (Simulink may provide 3D data).
            p = x(1:2);
            v = x(4:5);
            
            % Compute desired control via PID.
            u_desired = obj.desiredControl(p, v);
            
            % Filter obstacles and extract their states.
            [p_obs, v_obs, dim_obs] = obj.filterObstacles(X_obs);
            
            % Set up collision constraints.
            [T_col, lambda_terms] = obj.setupProblemConstraints(p, v, p_obs, v_obs, dim_obs);
            
            % Combine slack terms based on configuration.
            if obj.config.consider_only_min_t_col
                [T_col_min, obs_index] = min(T_col);
                lambda_sum = 1 / T_col_min * (lambda_terms{obs_index})^2;
            else
                lambda_coeffs = 1 ./ T_col;
                lambda_sum = 0;
                for i = 1:obj.n_obstacles
                    lambda_sum = lambda_sum + lambda_coeffs(i) * (lambda_terms{i})^2;
                end
            end
            
            % Set objective function based on configuration.
            if obj.config.include_VO_as_hard_constraint
                obj.problem.Objective = obj.config.k_u * norm(u_desired - obj.u_optim)^2;
            else
                obj.problem.Objective = obj.config.k_u * norm(u_desired - obj.u_optim)^2 + ...
                                         obj.config.k_vo * lambda_sum;
            end
            
            % Solve the optimization problem.
            [solution, cost, solve_status] = solve(obj.problem);
            u_solution = solution.u_optim;
            
            if solve_status < 0
                warning("Optimization did not find any feasible point... aborting...");
                obj.max_iterations_fail = obj.max_iterations_fail - 1;
                if obj.max_iterations_fail <= 0
                    exit_flag = true;
                end
            end
            
            % Limit control input within allowed bounds.
            try
                ones_vec = ones([2 1]);
                u_solution = max(min(u_solution, obj.config.u_max * ones_vec / sqrt(2)), ...
                                 -obj.config.u_max * ones_vec / sqrt(2));
            catch
                warning("Error while limiting control input. u_solution = %f", u_solution);
            end
            
            u = u_solution;
            obj.u_prev = u;
            goal_reached = obj.goalReached(p);
        end

        function u = desiredControl(obj, p, v)
            % desiredControl Compute the desired control using PID.
            p_goal = obj.agents_info{obj.id}.p_goal(1:2, obj.current_goal);
            p_err = p_goal - p;
            
            v_target = (p_err / norm(p_err)) * obj.config.v_mod;
            u_k = obj.config.P * (v_target - v);
            u_d = obj.config.D * ((p_err - obj.p_err_prev) / obj.config.timestep);
            
            u = u_k + u_d;
            ones_vec = ones([2 1]);
            u = max(min(u, obj.config.u_max * ones_vec), -obj.config.u_max * ones_vec);
            
            obj.p_err_prev = p_err;
        end

        function [T_collision, lambda_terms] = setupProblemConstraints(obj, p, v, p_obs, v_obs, dim_obs)
            % setupProblemConstraints Define collision constraints for obstacles.
            obj.problem.Constraints = struct();
            
            if obj.n_obstacles == 0
                T_collision = [];
                lambda_terms = {};
                return;
            end
            
            T_collision = -ones([obj.n_obstacles, 1]);
            lambda_terms = cell([obj.n_obstacles, 1]);
            
            p_A = p; v_A = v;
            R_A = obj.agents_info{obj.id}.dim(1) * (1 + obj.config.geometric_tol);
            for i = 1:obj.n_obstacles
                p_B = p_obs(:, i); v_B = v_obs(:, i);
                R_B = dim_obs(1, i) * (1 + obj.config.geometric_tol);
                
                constraintName = sprintf("cons%d", i);
                [ttc, lambda_t] = obj.includeAgentCBFConstraint(p_A, v_A, p_B, v_B, ...
                                            R_A, R_B, constraintName, i);
                T_collision(i) = ttc;
                lambda_terms{i} = lambda_t;
            end
        end

        function [t_col, lambda_terms] = includeAgentCBFConstraint(obj, p_A, v_A, p_B, v_B, ...
                                                    R_A, R_B, const_name, obs_id)
            % includeAgentCBFConstraint Add agent-specific CBF constraints.
            %   Includes a hard CBF for safety and, if configured, a soft
            %   VO-based constraint.
            delta = (R_A + R_B);
            
            if ~obj.config.include_VO_as_hard_constraint && obj.config.use_safe_cbf
                p_AB = p_B - p_A; 
                v_AB = v_B - v_A; 
                d_AB = norm(p_AB);
                v_ij = dot(v_AB, p_AB / d_AB);
                if v_ij < 0
                    args_h_scbf = {p_A, p_B, v_A, v_B, obj.u_optim, obj.config.u_max, ...
                                   delta, obj.config.alpha_c};
                    obj.problem.Constraints.(const_name) = h_scbf_ineq_fun(args_h_scbf{:}) >= 0;
                end
            end
            
            % Set up VO-based constraints.
            args_h_vo = {p_A, p_B, v_A, v_B, obj.u_optim, R_A, R_B, obj.config.alpha_vo};
            args_test = args_h_vo;
            args_test{5} = obj.u_symbolic;
            
            t_col = -1;
            lambda_terms = -1;
            try
                % Check if the constraint is influenced by the control input.
                const_valid = any(abs(double(jacobian(h_vo_inequality_fun(args_test{:}), obj.u_symbolic))) > 0);
                if const_valid
                    if obj.config.include_VO_as_hard_constraint
                        constNameVO = sprintf("%s_VO", const_name);
                        obj.problem.Constraints.(constNameVO) = h_vo_inequality_fun(args_h_vo{:}) >= 0;
                        lambda_terms = -1;
                        t_col = -1;
                    else
                        t_col = timeToCollisionVconstBall(p_A, p_B, v_A, v_B, R_A, R_B);
                        if isempty(t_col)
                            t_col = inf;
                            lambda_terms = 0;
                        else
                            constNameVO = sprintf("%s_VO", const_name);
                            obj.problem.Constraints.(constNameVO) = h_vo_inequality_fun(args_h_vo{:}) >= -obj.lambda_optim(obs_id);
                            lambda_terms = obj.lambda_optim(obs_id);
                        end
                    end
                end
            catch
                lambda_terms = 0;
                warning("Error during CBF constraint validity test.");
            end
        end

        function [p_obs, v_obs, dim_obs] = filterObstacles(obj, X_obs)
            % filterObstacles Extract obstacle positions, velocities, and dimensions.
            n_agents = numel(obj.agents_info);
            dim_obs = zeros([2, n_agents]);
            for i = 1:n_agents
                dim_obs(:, i) = obj.agents_info{i}.dim;
            end
            
            % Remove self from the obstacle list.
            X_obs(:, obj.id) = [];
            dim_obs(:, obj.id) = [];
            
            if isempty(X_obs)
                p_obs = [];
                v_obs = [];
            else
                p_obs = X_obs(1:2, :);
                v_obs = X_obs(4:5, :);
            end
        end

        function reached = goalReached(obj, p)
            % goalReached Check if the current goal has been reached.
            p_goal = obj.agents_info{obj.id}.p_goal;
            if any(p_goal >= 1e6)
                reached = true;
                return;
            end
            
            n_goals = size(p_goal, 2);
            if n_goals > 1
                pG = p_goal(1:2, obj.current_goal);
                has_subgoals = true;
            else
                pG = p_goal(1:2);
                has_subgoals = false;
            end
            
            error_p = p - pG;
            reached = norm(error_p) < obj.config.goal_reached_tol;
            
            if reached && has_subgoals
                if obj.current_goal < n_goals
                    obj.current_goal = obj.current_goal + 1;
                    reached = false; % Continue to next sub-goal.
                end
            end
        end
    end
end
