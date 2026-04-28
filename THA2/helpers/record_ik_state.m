function s = record_ik_state(robot, thetalist, T_curr, V_b, lambda, t_start)
% record_ik_state  Package per-iteration metrics into an IK history entry.
%
% Computes manipulability metrics from the current configuration and bundles
% everything into a struct matching the collect_ik_history output format.
% Called by IK solvers when the caller requests the optional history output.
%
% T_curr and V_b are passed in rather than recomputed — the solver already
% has them from the current loop iteration, so there is no extra FK call.
%
% Inputs:
%   robot     - robot struct from KR120_params()
%   thetalist - n×1 current joint angles (rad)
%   T_curr    - 4×4 current FK transform  (already computed in solver loop)
%   V_b       - 6×1 body twist error      (already computed in solver loop)
%   lambda    - DLS damping factor for this step (0 for NR / JT / RR)
%   t_start   - timer handle from tic() at the start of the solve
%
% Output:
%   s - struct with fields matching collect_ik_history entries:
%         .theta, .T_curr, .omg_err, .lin_err,
%         .kappa, .iso, .vol_lin, .vol_ang,
%         .lambda, .converged, .elapsed_s
%       .converged is always false here — the caller sets it to true on the
%       final entry when convergence is detected.

    Js    = J_space(robot.Slist, thetalist);
    sigma = svd(Js);
    if sigma(end) < 1e-10
        kappa = 1e8;  iso = 0;
    else
        kappa = sigma(1) / sigma(end);
        iso   = sigma(end) / sigma(1);
    end
    sigma_w = svd(Js(1:3,:));
    sigma_v = svd(Js(4:6,:));

    s.theta     = thetalist;
    s.T_curr    = T_curr;
    s.omg_err   = norm(V_b(1:3));
    s.lin_err   = norm(V_b(4:6));
    s.kappa     = min(kappa, 1e8);
    s.iso       = iso;
    s.vol_ang   = (4/3) * pi * prod(sigma_w(1:3));
    s.vol_lin   = (4/3) * pi * prod(sigma_v(1:3));
    s.lambda    = lambda;
    s.converged = false;
    s.elapsed_s = toc(t_start);
end
