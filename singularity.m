function [is_singular, w, types] = singularity(robot, thetalist, tol)
% singularity: Analytically identifies singularity configurations of a
%              6-DOF serial robot with spherical wrist (e.g., KUKA KR120).
%
% A configuration is singular when det(Js) = 0, i.e., the Jacobian loses
% full rank and certain end-effector velocities become unachievable.
%
% For wrist-partitioned robots (arm + spherical wrist), three independent
% singularity types exist:
%
%   1. WRIST singularity:
%      Condition: sin(theta5) = 0  (theta5 = 0 or pi)
%      Cause:     Joints 4 and 6 (both rotating about X at home) become
%                 coaxial — one DOF is lost at the wrist.
%
%   2. SHOULDER singularity:
%      Condition: wrist center lies on the J1 rotation axis (world Z-axis)
%      Cause:     J1 cannot contribute to end-effector motion in this pose.
%
%   3. ELBOW singularity:
%      Condition: arm fully extended or folded
%                 (dist(shoulder, wrist) = L1+L2 or |L1-L2|)
%      Cause:     Joints 2 and 3 become coplanar — arm Jacobian loses rank.
%
% Inputs:
%   robot     - robot struct from KR120_params()
%   thetalist - n x 1 joint angle vector (radians)
%   tol       - singularity proximity tolerance (default: 1e-3)
%
% Outputs:
%   is_singular - logical, true if any singularity condition is active
%   w           - manipulability measure:  w = sqrt(det(Js * Js'))
%                 (approaches 0 near singularities)
%   types       - cell array of strings naming active singularity types

    if nargin < 3, tol = 1e-3; end

    %% ---- Numeric manipulability measure ----
    Js = J_space(robot.Slist, thetalist);
    w  = sqrt(abs(det(Js * Js')));

    types = {};

    %% ---- 1. Wrist singularity: sin(theta5) = 0 ----
    % Derivation: the wrist is an X-Y-X Euler sequence (joints 4,5,6).
    % det(Jwrist) is proportional to sin(theta5); when theta5 = 0 or pi,
    % axes J4 and J6 (both about X at home) align -> one DOF lost.
    if abs(sin(thetalist(5))) < tol
        types{end+1} = sprintf('WRIST: theta5 = %.4f rad  [sin(theta5) = %.2e]', ...
                               thetalist(5), sin(thetalist(5)));
    end

    %% ---- 2. Shoulder singularity: wrist center on J1 (world Z) axis ----
    % Joints 4-6 all rotate about axes through the wrist center, so they
    % do not move it.  Compute wrist center by applying only joints 1-3
    % to its home position.
    %
    % Home wrist-center (arm extended along +X): p_wc0 = [a1+a2+a3; 0; d1+dz]
    p_wc_home = [robot.a(1)+robot.a(2)+robot.a(3); 0; robot.d(1)+robot.dz];
    T_arm = eye(4);
    for i = 1:min(3, robot.n_dof)
        T_arm = T_arm * MatrixExp6(vecToSE3(robot.Slist(:,i) * thetalist(i)));
    end
    p_wc = T_arm(1:3,1:3) * p_wc_home + T_arm(1:3,4);

    xy_dist = norm(p_wc(1:2));
    if xy_dist < tol
        types{end+1} = sprintf('SHOULDER: wrist center on J1 axis  [xy-dist = %.2e m]', xy_dist);
    end

    %% ---- 3. Elbow singularity: arm at its reach limit ----
    % 2-link arm: L1 = a2 (upper arm), L2 = sqrt(a3^2+dz^2) (forearm — includes z-drop).
    % Singularity when dist(shoulder, wrist) = L1+L2 (extended)
    %                                        or |L1-L2| (folded).
    T1         = MatrixExp6(vecToSE3(robot.Slist(:,1) * thetalist(1)));
    p_shoulder = T1(1:3,1:3) * [robot.a(1); 0; robot.d(1)] + T1(1:3,4);

    L1  = robot.a(2);
    L2  = sqrt(robot.a(3)^2 + robot.dz^2);
    d_sw = norm(p_wc - p_shoulder);

    if abs(d_sw - (L1 + L2)) < tol
        types{end+1} = sprintf('ELBOW: fully EXTENDED  [dist=%.4f = L1+L2=%.4f m]', d_sw, L1+L2);
    elseif abs(d_sw - abs(L1 - L2)) < tol
        types{end+1} = sprintf('ELBOW: fully FOLDED  [dist=%.4f = |L1-L2|=%.4f m]', d_sw, abs(L1-L2));
    end

    %% ---- SVD-based catch-all ----
    % Find singular values below the relative threshold; catches any cases
    % not covered by the three analytical conditions above.
    [~, S, ~] = svd(Js);
    sigma     = diag(S);
    if ~isempty(sigma) && sigma(1) > 1e-10
        small_idx = find(sigma / sigma(1) < tol);
        already_numeric = any(cellfun(@(s) contains(s, 'NUMERIC'), types));
        if ~isempty(small_idx) && ~already_numeric
            numeric_msgs = arrayfun( ...
                @(k) sprintf('NUMERIC: sigma_%d = %.2e  (ratio = %.2e)', ...
                             k, sigma(k), sigma(k)/sigma(1)), ...
                small_idx, 'UniformOutput', false);
            types = [types, numeric_msgs(:)'];
        end
    end

    is_singular = ~isempty(types) || (w < tol);
    if isempty(types)
        types = {'none'};
    end

    %% ---- Console report ----
    fprintf('\n--- Singularity Analysis ---\n');
    fprintf('  Manipulability  w = %.6f\n', w);
    fprintf('  Is singular:    %d\n', is_singular);
    for k = 1:numel(types)
        fprintf('    * %s\n', types{k});
    end
    fprintf('----------------------------\n');
end
