function [dq, info] = QP_step_VF(robot, q, p_goal, opts)
% QP_step_VF  One step of QP-based virtual fixture velocity control.
%
% Solves the quadratic program:
%
%   min   (1/2) dq' H dq + f' dq
%   s.t.  A_ineq * dq <= b_ineq        (sphere + wall constraints)
%         lb <= dq <= ub               (joint limits + max step)
%
% where:
%   H = J_tip'*J_tip + lambda*I  [+ mu*J_perp'*J_perp for part b]
%   f = -J_tip' * v_d
%   v_d = saturated step toward p_goal
%
% The 3 mm sphere constraint activates once the tip enters the sphere
% around p_goal and prevents the tip from moving back out.
%
% The virtual wall constraint (part c) prevents the tip from penetrating
% a half-space defined by a point and outward normal.
%
% Inputs:
%   robot  - robot struct from KR120_params
%   q      - n-by-1 current joint angles (radians)
%   p_goal - 3-by-1 goal position for tool tip (meters)
%   opts   - options struct (all fields optional):
%     .L_tool  : tool length in meters           (default: 0.1)
%     .lambda  : Tikhonov regularisation weight  (default: 1e-4)
%     .mu      : tool-shaft direction weight [b] (default: 0)
%     .d_max   : sphere constraint radius (m)    (default: 0.003)
%     .step    : max Cartesian step per iter (m) (default: 0.02)
%     .dq_max  : max joint step per iter (rad)   (default: 0.05)
%     .wall    : virtual wall struct [c]:
%                  .n      - 3-by-1 outward unit normal (toward robot side)
%                  .p      - 3-by-1 point on wall
%                  .margin - activation distance (m)    (default: 0.05)
%
% Outputs:
%   dq   - n-by-1 optimal joint displacement
%   info - struct: .dist, .exitflag, .n_constraints, .v_d

    if nargin < 4, opts = struct(); end

    L_tool = opt(opts, 'L_tool', 0.1);
    lambda = opt(opts, 'lambda', 1e-4);
    mu     = opt(opts, 'mu',     0);
    d_max  = opt(opts, 'd_max',  0.003);
    step   = opt(opts, 'step',   0.02);
    dq_max = opt(opts, 'dq_max', 0.05);
    wall   = opt(opts, 'wall',   []);

    n = robot.n_dof;

    % --- Forward kinematics and Jacobians ---
    [p_tip, J_tip3, T_ee] = tool_tip_fk(robot, q, L_tool);
    R_ee    = T_ee(1:3, 1:3);
    Js      = J_space(robot.Slist, q);
    J_omega = Js(1:3, :);   % 3-by-n angular part of space Jacobian

    % --- Desired Cartesian step (saturated toward p_goal) ---
    e   = p_goal - p_tip;
    d   = norm(e);
    v_d = min(d, step) * e / max(d, 1e-10);

    % --- QP objective matrices ---
    H = J_tip3' * J_tip3 + lambda * eye(n);
    f = -J_tip3' * v_d;

    % Part (b): add cost term that penalises transverse angular velocity
    % of the tool shaft.  J_perp maps dq to the angular velocity component
    % perpendicular to the tool axis — the part that actually rotates the shaft.
    if mu > 0
        tool_axis = R_ee * [0; 0; 1];
        J_perp    = (eye(3) - tool_axis * tool_axis') * J_omega;
        H = H + mu * (J_perp' * J_perp);
    end

    % Symmetrise H to guard against floating-point asymmetry
    H = (H + H') / 2;

    % --- Inequality constraints: A_ineq * dq <= b_ineq ---
    A_ineq = zeros(0, n);
    b_ineq = zeros(0, 1);

    % 3 mm sphere constraint: once inside, outward radial velocity <= 0.
    % n_out points from p_goal outward toward p_tip.
    if d <= d_max && d > 1e-9
        n_out  = -e / d;                              % unit outward normal
        A_ineq = [A_ineq; n_out' * J_tip3];
        b_ineq = [b_ineq; 0];
    end

    % Virtual wall constraint (part c): tip must stay on the positive side
    % of the wall.  n_wall points toward the free (robot) side.
    % Activate when tip is within margin of wall.
    if ~isempty(wall)
        margin = opt(wall, 'margin', 0.05);
        pen    = wall.n' * (p_tip - wall.p);          % signed dist: positive = safe side
        if pen <= margin
            A_ineq = [A_ineq; -wall.n' * J_tip3];    % -n'*J*dq <= 0 → n'*J*dq >= 0
            b_ineq = [b_ineq; 0];
        end
    end

    % --- Simple bounds: joint limits clipped to max step size ---
    lb = max(robot.joint_limits(:,1) - q, -dq_max * ones(n,1));
    ub = min(robot.joint_limits(:,2) - q,  dq_max * ones(n,1));

    % --- Solve QP ---
    qp_opts = optimoptions('quadprog', 'Display', 'off');
    if isempty(A_ineq)
        [dq, ~, flag] = quadprog(H, f, [], [], [], [], lb, ub, [], qp_opts);
    else
        [dq, ~, flag] = quadprog(H, f, A_ineq, b_ineq, [], [], lb, ub, [], qp_opts);
    end

    if isempty(dq) || flag < 0
        dq = zeros(n, 1);
    end

    info.dist          = d;
    info.exitflag      = flag;
    info.n_constraints = size(A_ineq, 1);
    info.v_d           = v_d;
end

% -------------------------------------------------------------------------
function v = opt(s, field, default)
    if isstruct(s) && isfield(s, field)
        v = s.(field);
    else
        v = default;
    end
end
