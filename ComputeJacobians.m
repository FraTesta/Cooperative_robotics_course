function [uvms] = ComputeJacobians(uvms)
% compute the relevant Jacobians here
% joint limits
% manipulability
% tool-frame position control
% vehicle-frame position control
% horizontal attitude 
% minimum altitude
% preferred arm posture ( [-0.0031 1.2586 0.0128 -1.2460] )
%
% remember: the control vector is:
% [q_dot; p_dot] 
% [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
% with the vehicle velocities projected on <v>
%
% therefore all task jacobians should be of dimensions
% m x 13  
% where m is the row dimension of the task, and of its reference rate

% Jt structure
% computation for tool-frame Jacobian (tool frame is the manipulator frame)
% [omegax_t omegay_t omegaz_t xdot_t ydot_t zdot_t] = Jt ydot
% [angular velocities; linear velocities]
%
% Ste is the rigid body transformation from vehicle-frame to end-effector
% (see relative basic jacobian matrix on Pino's notes )
% frame projected on <v> (vehicle-frame)
uvms.Ste = [eye(3) zeros(3);  -skew(uvms.vTe(1:3,1:3)*uvms.eTt(1:3,4)) eye(3)];
% uvms.bJe contains the arm end-effector wrt arm base Jacobian (6x7).
% Top three rows are angular velocities, bottom three linear velocities
% (Ste serve per rappresentare bJe w.r.t <v> frame.
% Il vettore grosso centrale si occupa di riportare la rotazione sempre rispetto al <v>)
% Mentre la matrice centrale serve per proiettare il tutto rispetto al <v>
% frame (see projection of J matrix outo another frame)
uvms.Jt_a  = uvms.Ste * [uvms.vTb(1:3,1:3) zeros(3,3); zeros(3,3) uvms.vTb(1:3,1:3)]  * uvms.bJe;

% vehicle contribution is simply a rigid body transformation from vehicle
% frame to tool frame. Notice that linear and angular velocities are
% swapped due to the different definitions of the task and control
% variables
uvms.Jt_v = [zeros(3) eye(3); eye(3) -skew(uvms.vTt(1:3,4))];
% juxtapose the two Jacobians (J of the tool that depends on the arm +
% depends of the vehicle contributions) to reach the goal with the e.e. (tool frame)
uvms.Jt = [uvms.Jt_a uvms.Jt_v];
%% Manipulability
% compute manipulability Jacobian
[Jmu_a, uvms.mu] = ComputeManipulability(uvms.bJe, uvms.djdq);
uvms.Jmu = [Jmu_a zeros(1,6)];

%% EX 1 (Vehicle control)
% For exercise 1 Instead make a single J = [angular vel ; linear vel] I
% chose to keep the two velocities separate in other to have two differet
% tasks (but it's the same thing ). The objective is to reach the goal
% position with the vehile frame and not with the e.e.

% see notes 
% Vehicle frame Jacobians
uvms.Jvpos = [ zeros(3,7)  uvms.wTv(1:3,1:3) zeros(3,3)];  % jacobian for the position (linear vel)
uvms.Jvatt = [zeros(3,7) zeros(3,3)  uvms.wTv(1:3,1:3)];  % jacobian for the attitude (angular vel)
% As I said I didn't join the two jacobians as usually  
%uvms.Jv = [uvms.Jv_linear ; uvms. Jv_angular];

%% Horizontal Attitude 
% Missalignment of Kw w.r.t Kv (so veichle parallal w.r.t the ground)
% see notes
w_kw = [0 0 1]'; % n vector k axis of frame 
v_kv = [0 0 1]';
v_kw = uvms.vTw(1:3,1:3) * w_kw; % vRw * w_kw 

uvms.v_rho = ReducedVersorLemma(v_kw,v_kv); % compute the missalignment vector projected on <v>

if norm(uvms.v_rho) == 0
    uvms.v_n = zeros(3,1);
    
    else
    uvms.v_n = uvms.v_rho/norm(uvms.v_rho);
end


uvms.Jha = [zeros(1,7) zeros(1,3) (uvms.v_n)' ]; 

%% Ex 2 
%(1 meter from the seafloor)
% see the notes, it's just the projection of the lin velocity projected on
% <v>
uvms.Jact = [zeros(1,7) v_kw' zeros(1,3)];

altitude = [0 0 uvms.sensorDistance]';
uvms.v_altitude = v_kw' * altitude;

%% Underactuation ex

uvms.Jua = [zeros(6,7) eye(6)];

%% Ex3 
% define the J of the landing task of the second action
% which is the same of the Ex 2 sice we want to mantain the altitude to a
% certain value
uvms.Jla = [zeros(1,7) v_kw' zeros(1,3)];

%% Alignment to the rock
uvms.v_iv = [1 0 0]';
uvms.w_iv =uvms.vTw(1:3,1:3) * uvms.v_iv;

w_d = uvms.w_rock_center - uvms.p(1:3); % controllare verso
P = [1 0 0; 0 1 0; 0 0 0];
w_dp = P * w_d;
uvms.v_dp = uvms.vTw(1:3,1:3)*w_dp;

uvms.v_rho_r = ReducedVersorLemma(uvms.v_iv,uvms.v_dp);

uvms.v_n_r = uvms.v_rho_r/norm(uvms.v_rho_r);

uvms.Jlr = uvms.v_n_r'*[zeros(3,7) -1/(norm(uvms.v_dp)^2)*skew(uvms.v_dp) -eye(3,3)]; 
%% Vehicle constrainrs 
Jvc_lin = [zeros(3,7) eye(3) zeros(3)];
Jvc_ang = [zeros(3,7) zeros(3) eye(3)];
uvms.Jvc = [Jvc_lin; Jvc_ang];
%% Joint limits
uvms.Jjl = [eye(7) zeros(7,3) zeros(7,3)];
%% Joint Prefered shape 
uvms.Jps = [eye(4) zeros(4,3) zeros(4,3) zeros(4,3)];
end