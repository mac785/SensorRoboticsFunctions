function [p_tip, J_tip3, T_ee] = tool_tip_fk(robot, q, L_tool)
% tool_tip_fk  Tool tip position and 3-by-n linear Jacobian.
%
%   The cylindrical tool extends L_tool meters along the z-axis of the
%   end-effector body frame.  The tip position in world frame is:
%
%       p_tip = p_ee + R_ee * [0; 0; L_tool]
%
%   Its linear velocity is related to joint rates by:
%
%       v_tip = v_ee + omega_ee x r  =  (J_v - skew(r)*J_omega) * dq
%
%   giving the 3-by-n tool-tip Jacobian:
%
%       J_tip = J_v - skew(r) * J_omega
%
%   where J_v and J_omega are the linear and angular rows of the
%   space-form Jacobian, and r = R_ee * [0;0;L_tool].
%
% Inputs:
%   robot  - robot struct from KR120_params
%   q      - n-by-1 joint angles (radians)
%   L_tool - tool length in meters (default: 0.1)
%
% Outputs:
%   p_tip  - 3-by-1 tool tip position in world frame (meters)
%   J_tip3 - 3-by-n linear Jacobian of the tool tip
%   T_ee   - 4-by-4 end-effector SE(3) transform

    if nargin < 3, L_tool = 0.1; end

    T_ee = FK_space(robot.M, robot.Slist, q);
    R_ee = T_ee(1:3, 1:3);
    p_ee = T_ee(1:3, 4);

    r     = R_ee * [0; 0; L_tool];   % offset from EE origin to tip (world frame)
    p_tip = p_ee + r;

    if nargout < 2, return; end

    Js      = J_space(robot.Slist, q);
    J_omega = Js(1:3, :);             % 3-by-n angular part
    J_v     = Js(4:6, :);             % 3-by-n linear part

    % The space Jacobian's linear part J_v gives the velocity of the body
    % point instantaneously at the WORLD ORIGIN, not the EE origin.
    % Velocity of any point at absolute position p is:
    %   v_p = v_s + omega_s x p  =  (J_v - skew(p_tip)*J_omega) * dq
    % So the tool-tip Jacobian uses p_tip (absolute), not r (relative offset).
    J_tip3 = J_v - vecToSO3(p_tip) * J_omega;
end
