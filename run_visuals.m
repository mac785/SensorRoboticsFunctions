%% run_visuals.m
% Launches all required graphical outputs for ME397 THA2.
%
% Parts covered:
%   (b)  FK_space  — joint frames and screw axes (space form)
%   (c)  FK_body   — joint frames and screw axes (body form)
%   (g)  ellipsoid_plot_angular — angular velocity manipulability ellipsoid
%   (g)  ellipsoid_plot_linear  — linear  velocity manipulability ellipsoid
%
% Run from the SensorRoboticsFunctions directory:
%   >> run_visuals

clc;
addpath('.');
robot = KR210_params();

% -----------------------------------------------------------------------
% Configurations used for demonstration
% -----------------------------------------------------------------------
theta_home = zeros(6,1);                        % home (all joints at 0)
theta_bent = [pi/6; pi/4; -pi/3; pi/4; pi/3; pi/6];  % visually interesting

% -----------------------------------------------------------------------
% (b)  FK_space — space-form PoE with frame/screw-axis visualisation
% -----------------------------------------------------------------------
fprintf('--- (b) FK_space visualisation ---\n');

fprintf('  Home configuration (all joints = 0):\n');
T_sp_home = FK_space(robot.M, robot.Slist, theta_home, true, robot.q_joints);
title('(b) FK\_space — Home configuration (\theta = 0)');
fprintf('    EE position: [%.4f, %.4f, %.4f] m\n', T_sp_home(1:3,4)');

fprintf('  Bent configuration:\n');
T_sp_bent = FK_space(robot.M, robot.Slist, theta_bent, true, robot.q_joints);
title('(b) FK\_space — Bent configuration');
fprintf('    EE position: [%.4f, %.4f, %.4f] m\n', T_sp_bent(1:3,4)');

% -----------------------------------------------------------------------
% (c)  FK_body — body-form PoE with frame/screw-axis visualisation
% -----------------------------------------------------------------------
fprintf('\n--- (c) FK_body visualisation ---\n');

fprintf('  Home configuration:\n');
T_bd_home = FK_body(robot.M, robot.Blist, theta_home, true, robot.q_joints);
title('(c) FK\_body — Home configuration (\theta = 0)');
fprintf('    EE position: [%.4f, %.4f, %.4f] m\n', T_bd_home(1:3,4)');

fprintf('  Bent configuration:\n');
T_bd_bent = FK_body(robot.M, robot.Blist, theta_bent, true, robot.q_joints);
title('(c) FK\_body — Bent configuration');
fprintf('    EE position: [%.4f, %.4f, %.4f] m\n', T_bd_bent(1:3,4)');

% Confirm space and body forms agree
err = norm(T_sp_bent - T_bd_bent, 'fro');
fprintf('  ||T_space - T_body||_F = %.2e  (should be ~0)\n', err);

% -----------------------------------------------------------------------
% (g)  Manipulability ellipsoids — non-singular configuration
% -----------------------------------------------------------------------
% Note: theta_home has theta5=0 (wrist singularity) which collapses the
%       full Jacobian. Use a non-singular config for meaningful ellipsoids.
theta_ns = [0.1; 0.2; -0.3; 0.4; 0.5; 0.6];

fprintf('\n--- (g) Manipulability ellipsoids (non-singular config) ---\n');
fprintf('  theta = [0.1, 0.2, -0.3, 0.4, 0.5, 0.6] rad\n');

fprintf('  Angular velocity ellipsoid:\n');
ellipsoid_plot_angular(robot, theta_ns);
title('(g) Angular Velocity Manipulability Ellipsoid');

fprintf('  Linear velocity ellipsoid:\n');
ellipsoid_plot_linear(robot, theta_ns);
title('(g) Linear Velocity Manipulability Ellipsoid');

% Also show ellipsoids at the bent configuration for comparison
fprintf('\n  Ellipsoids at bent configuration (for comparison):\n');
ellipsoid_plot_angular(robot, theta_bent);
title('(g) Angular Velocity Manipulability Ellipsoid — Bent config');

ellipsoid_plot_linear(robot, theta_bent);
title('(g) Linear Velocity Manipulability Ellipsoid — Bent config');

fprintf('\nAll figures generated. Use the figure toolbar to rotate/zoom.\n');
