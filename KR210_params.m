function robot = KR210_params()
% KR210_params: Returns the kinematic parameters of the KUKA KR210 R2700
% as a struct for use with FK, Jacobian, and IK functions.
%
% Source: KUKA Deutschland GmbH, "KR 210 R2700 extra Technical Data"
%         Lynch & Park, "Modern Robotics," Ch. 4 (PoE convention)
%
% All dimensions in meters, angles in radians.

%% --- Link dimensions (from KUKA datasheet) ---
d1 =  0.675;   % base to shoulder height
a1 =  0.350;   % shoulder x-offset
a2 =  1.350;   % upper arm length
a3 =  0.055;   % small elbow offset
d4 =  1.400;   % elbow to wrist length
d6 =  0.215;   % wrist to end-effector

%% --- Home configuration M (end-effector frame at theta = 0) ---
% Position: end-effector sits at [a1+a2+a3+d4, 0, d1] with d6 along +x
% Orientation: identity (x forward, z up at home)
p_home = [a1 + a2 + a3 + d4 + d6;   % x
          0;                          % y
          d1];                        % z

robot.M = [1, 0, 0, p_home(1);
           0, 1, 0, p_home(2);
           0, 0, 1, p_home(3);
           0, 0, 0, 1         ];

%% --- Space-form screw axes S_i = [omega_i; v_i] where v_i = -omega_i x q_i ---
% q_i is a point on joint i's axis at the home configuration

% Joint 1: rotates about Z at origin
w1 = [0; 0; 1];
q1 = [0; 0; 0];

% Joint 2: rotates about Y at shoulder
w2 = [0; 1; 0];
q2 = [a1; 0; d1];

% Joint 3: rotates about Y at elbow
w3 = [0; 1; 0];
q3 = [a1; 0; d1 + a2];

% Joint 4: rotates about X along forearm axis
w4 = [1; 0; 0];
q4 = [a1 + a3 + d4; 0; d1 + a2];

% Joint 5: rotates about Y at wrist
w5 = [0; 1; 0];
q5 = [a1 + a3 + d4; 0; d1 + a2];

% Joint 6: rotates about X at end-effector
w6 = [1; 0; 0];
q6 = [a1 + a3 + d4; 0; d1 + a2];

% Build Slist: each column is one screw axis [omega; v]
ws = {w1, w2, w3, w4, w5, w6};
qs = {q1, q2, q3, q4, q5, q6};

robot.Slist = zeros(6, 6);
for i = 1:6
    v = -cross(ws{i}, qs{i});
    robot.Slist(:, i) = [ws{i}; v];
end

%% --- Body-form screw axes B_i = Ad(M^-1) * S_i ---
robot.Blist = zeros(6, 6);
AdMinv = adjoint_map(inv_transform(robot.M));
for i = 1:6
    robot.Blist(:, i) = AdMinv * robot.Slist(:, i);
end

%% --- Joint limits (radians) ---
robot.joint_limits = deg2rad([
   -185,  185;   % Joint 1
   -140,    5;   % Joint 2
   -120,  155;   % Joint 3
   -350,  350;   % Joint 4
   -122.5, 122.5;% Joint 5
   -350,  350;   % Joint 6
]);

%% --- Metadata ---
robot.name   = 'KUKA KR210 R2700 Extra';
robot.n_dof  = 6;
robot.d      = [d1; 0; 0; d4; 0; d6];
robot.a      = [a1; a2; a3; 0; 0; 0];

end