function robot = KR120_params()
% KR120_params: Returns the kinematic parameters of the KUKA KR120 R2500 Pro
% (Quantec Nano series) as a struct for use with FK, Jacobian, and IK functions.
%
% Source: kr120r2500pro.urdf (kuka_experimental ROS package)
%         Lynch & Park, "Modern Robotics," Ch. 4 (PoE convention)
%
% All dimensions in meters, angles in radians.

%% --- Link dimensions (from URDF joint origins) ---
d1 =  0.675;   % base to shoulder height         (joint_a1 z)
a1 =  0.350;   % shoulder x-offset               (joint_a2 x)
a2 =  1.150;   % upper arm length                (joint_a3 x)
a3 =  1.000;   % elbow to wrist-centre length    (joint_a4 x)
dz = -0.041;   % wrist-centre z drop             (joint_a4 z)
d6 =  0.215;   % wrist to end-effector           (tool0 x)

% Wrist-centre world position at home (theta = 0):
%   x = a1 + a2 + a3 = 2.500
%   z = d1 + dz      = 0.634
wc = [a1 + a2 + a3; 0; d1 + dz];  % [2.500; 0; 0.634]

%% --- Home configuration M (end-effector frame at theta = 0) ---
% tool0 fixed joint: xyz="0.215 0 0", rpy="0 pi/2 0"
%   -> EE position = wc + [d6; 0; 0] = [2.715; 0; 0.634]
%   -> EE orientation = Ry(+90 deg) = [0 0 1; 0 1 0; -1 0 0]
p_ee = wc + [d6; 0; 0];   % [2.715; 0; 0.634]
Ry90 = [0, 0, 1; 0, 1, 0; -1, 0, 0];

robot.M = [Ry90, p_ee; 0, 0, 0, 1];

%% --- Space-form screw axes S_i = [omega_i; v_i] where v_i = -omega_i x q_i ---
% Joint axes and positions from URDF (world frame at home, theta = 0).

% Joint 1: about -Z at base origin
w1 = [0; 0; -1];
q1 = [0; 0; 0];

% Joint 2: about +Y at shoulder [a1, 0, d1]
w2 = [0; 1; 0];
q2 = [a1; 0; d1];

% Joint 3: about +Y at elbow [a1+a2, 0, d1]
w3 = [0; 1; 0];
q3 = [a1 + a2; 0; d1];

% Joint 4: about -X at wrist centre (joint_a4 has xyz="1.0 0 -0.041" from link_3)
w4 = [-1; 0; 0];
q4 = wc;

% Joint 5: about +Y at wrist centre (joint_a5 has xyz="0 0 0" from link_4)
w5 = [0; 1; 0];
q5 = wc;

% Joint 6: about -X at wrist centre (joint_a6 has xyz="0 0 0" from link_5)
w6 = [-1; 0; 0];
q6 = wc;

% Build Slist: each column is one screw axis [omega; v]
ws = {w1, w2, w3, w4, w5, w6};
qs = {q1, q2, q3, q4, q5, q6};

% Store physical joint positions for visualization
robot.q_joints = [q1, q2, q3, q4, q5, q6];  % 3x6, columns are joint positions at home

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

%% --- Joint limits (radians) — from URDF ---
robot.joint_limits = [
   -3.22885911619,  3.22885911619;   % Joint 1  (~+/-185 deg)
   -2.70526034059,  0.610865238198;  % Joint 2  (~-155 to +35 deg)
   -2.26892802759,  2.68780704807;   % Joint 3  (~-130 to +154 deg)
   -6.10865238198,  6.10865238198;   % Joint 4  (~+/-350 deg)
   -2.26892802759,  2.26892802759;   % Joint 5  (~+/-130 deg)
   -6.10865238198,  6.10865238198;   % Joint 6  (~+/-350 deg)
];

%% --- Metadata ---
robot.name   = 'KUKA KR120 R2500 Pro (Quantec Nano)';
robot.n_dof  = 6;
robot.d      = [d1; 0; 0; 0; 0; d6];   % [d1, 0, 0, 0, 0, d6]
robot.a      = [a1; a2; a3; 0; 0; 0];  % [a1, a2, a3, 0, 0, 0]
robot.dz     = dz;                       % z-drop to wrist centre (joint_a4 z-offset)

end
