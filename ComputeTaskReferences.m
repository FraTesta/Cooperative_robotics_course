function [uvms] = ComputeTaskReferences(uvms, mission)
% compute the task references here

% reference for tool-frame position control task
% We use the Cartesian Error function to compute the distance (lin) and
% misalignmen vectors (ang) (open the script in simulation_scripts directory)
% we pass the position of the goal w.r.t the vehicle and the position of
% the tool w.r.t to the <v>
% the returned values are projected on the common frame (in this case <v> )
[ang, lin] = CartError(uvms.vTg , uvms.vTt); 

uvms.xdot.t = 0.2 * [ang; lin]; % generation of the reference velocities where 0.2 is the Lambda (see notes)
% limit the requested velocities...
uvms.xdot.t(1:3) = Saturate(uvms.xdot.t(1:3), 0.2);
uvms.xdot.t(4:6) = Saturate(uvms.xdot.t(4:6), 0.2);

%% Ex1
% compute the distance and the misalginment from the goal to the vehicle
% projected on the world frame 
[w_vang,w_vlin] = CartError(uvms.wTgv , uvms.wTv); 

% Saturate the mas velocities to 0.5 
uvms.xdot.vpos(1:3,:) = Saturate(0.5* w_vlin, 1); 
uvms.xdot.vatt(1:3,:) = Saturate(0.5* w_vang, 1);

%% Ex allignment w.r.t the ground
uvms.xdot.ha = 0.2 * (0 - norm(uvms.v_rho));

%% Ex 2
% 1 metere from the seafloor
uvms.xdot.act = 1 * (10.3 - uvms.v_altitude);
%uvms.xdot.act = 0.2 * (1 - uvms.sensorDistance); % uvms.sensorDistance gives me "d" (see notes), whose time derivative is v_Vvw
% so I have to multiply it with v_Kw' in order to get its projection "a" 
% which is the real distance (see notes and ComputeJacobians). 1 is the
% desired value (1 metere from the seafloor)

%% underactuated ex
uvms.xdot.ua = uvms.p_dot; % just the w_x feedback as in the notes
%% Ex3 
% define the task vector for landing 
uvms.xdot.la = 0.5 * (0 - norm(uvms.v_altitude));
%% Landing aligned with rock
% theta() = ReducedVersorLemma(uvms.v_dp,uvms.v_iv);
uvms.xdot.lr = 0.5 * (0 - norm(uvms.v_rho_r));
% uvms.xdot.lr = Saturate(0.2, uvms.v_rho_r);
