function MainRobust
addpath('./simulation_scripts');
clc;
clear;
close all

% Simulation variables (integration and final time)
deltat = 0.005;
end_time = 15; %25
loop = 1;
maxloops = ceil(end_time/deltat);

% this struct can be used to evolve what the UVMS has to do
mission.phase = 1;
mission.phase_time = 0;

% Rotation matrix to convert coordinates between Unity and the <w> frame
% do not change
wuRw = rotation(0,-pi/2,pi/2);
vRvu = rotation(-pi/2,0,-pi/2);

% pipe parameters
u_pipe_center = [-10.66 31.47 -1.94]'; % in unity coordinates
pipe_center = wuRw'*u_pipe_center;     % in world frame coordinates
pipe_radius = 0.3;

% rock position 
rock_center = [12.2025   37.3748  -39.8860]'; % in world frame coordinates

% UDP Connection with Unity viewer v2
% Matlab speaks with visualization tool through UDP 
% THE SCRIPT RECIVES THE ALTITUDE OF THE ROBOT FROM THE VIEWER!!!!
uArm = udp('127.0.0.1',15000,'OutputDatagramPacketSize',28);
uVehicle = udp('127.0.0.1',15001,'OutputDatagramPacketSize',24);
fopen(uVehicle);
fopen(uArm);
uAltitude = dsp.UDPReceiver('LocalIPPort',15003,'MessageDataType','single');
uAltitude.setup();



% initialize uvms structure (declare here all T and variables related to the uvms structure)
uvms = InitUVMS('Robust'); % see initUVMS.m



% INITIAL JOINT POSITION.
%You can change these values to initialize the simulation with a 
% different starting position for the arm
uvms.q = [-0.0031 0 0.0128 -1.2460 0.0137 0.0853-pi/2 0.0137]'; 

% INITIAL POSITION OF THE VEHICLE (POSITION AND ROTATION)
% the vector contains the values in the following order
% [x y z r(rot_x) p(rot_y) y(rot_z)]
% RPY angles are applied in the following sequence!!!!!!!!!!!!!!!!!!!!!!!!
% R(rot_x, rot_y, rot_z) = Rz (rot_z) * Ry(rot_y) * Rx(rot_x)
% uvms.v_init_pose = [10.5 37.5 -38   0 -0.06 0.5]'; % 1.1
% uvms.v_init_pose = [10.5 37.5 -38   0 0.5 0.5]'; % 1.1 with pitch
% uvms.v_init_pose = [8.5 38.5 -36   0 -0.06 0.5]'; % 1.2
uvms.v_init_pose = [8.5 38.5 -36  0 -0.06 0.5]'; % ex 3.1


% uvms.v_init_pose = [8.5 38.5 -38   0 -0.06 0.5]'; % original
uvms.p = uvms.v_init_pose; % Ex 1 

% defines the goal position for the end-effector/tool position task
%uvms.goalPosition = [12.2025   37.3748  -39.8860]';  %rock
uvms.goalPosition = rock_center;
uvms.wRg = rotation(0, pi, pi/2);
uvms.wTg = [uvms.wRg uvms.goalPosition; 0 0 0 1];

% defines the tool control point
uvms.eTt = eye(4);

%Vehicle control goal 
%uvms.vgoalPosition = [10.2025   37.3748  -38.8860+2]'; % goal position w.r.t veichle frame 
% uvms.vgoalPosition = [12 30 -33]'; % 1.1 inventato
% uvms.vgoalPosition = [10.5 37.5 -38]'; % 1.2
uvms.vgoalPosition = [10.5 37.5 -35]';
%uvms.vgoalPosition = rock_center;
% uvms.wRgv = rotation(0 ,0 ,0); % R matrix goal w.r.t vehicle projected on world frame in order to have the goal frame parallel to ground
% uvms.wRgv = rotation(0 , pi/3 ,0); % R matrix to place the goal 45° w.r.t. the ground, use it to test the allignment ground task
uvms.wRgv = rotation(0, -0.06 ,0.5); %1.2
uvms.wTgv = [uvms.wRgv uvms.vgoalPosition; 0 0 0 1]; % new matrix which rappresent the goal from the veichle

% Preallocation (data structure to plot what we want)
plt = InitDataPlot(maxloops, uvms);

tic
for t = 0:deltat:end_time
    % update all the involved variables
    uvms = UpdateTransforms(uvms);
    uvms = ComputeJacobians(uvms);
    uvms = ComputeTaskReferences(uvms, mission); % to compute the objective vectors (e.g. missalignment vector )
    uvms = ComputeActivationFunctions(uvms, mission);
    
    % receive altitude information from unity
    uvms = ReceiveUdpPackets(uvms, uAltitude);
    
    % main kinematic algorithm initialization (returns linear and angular velocities and joint velocities 
    % as a vector called ydotbar )
    % ydotbar order is [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
    % the vector of the vehicle linear and angular velocities are assumed
    % projected on <v> (vehicle frame)  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    
    ydotbar = zeros(13,1);
    Qp = eye(13); 
    
    % TASK DEFINITION 
    % iCAT_task compute the pseudoinvers and requires: Ativation function, Jacobian, reference
    % velocities and two parameters for the SVD pseudoinversion
    % add all the other tasks here!!!!!
    % the sequence of iCAT_task calls defines the priority
     
     %[Qp, ydotbar] = iCAT_task(uvms.A.ua,  uvms.Jua,    Qp, ydotbar, uvms.xdot.ua,  0.0001,   0.01, 10); % underactuated task, which must be at the top priority (Add disturbance down in this script)
     [Qp, ydotbar] = iCAT_task(uvms.A.act,  uvms.Jact,    Qp, ydotbar, uvms.xdot.act,  0.0001,   0.01, 10); % Ex2: mantain 1m distasnce from the seaflor 
     [Qp, ydotbar] = iCAT_task(uvms.A.ha,  uvms.Jha,    Qp, ydotbar, uvms.xdot.ha,  0.0001,   0.01, 10); % misallignment of Kw (vehicle parallel w.r.t the ground)
     [Qp, ydotbar] = iCAT_task(uvms.A.vpos,  uvms.Jvpos,    Qp, ydotbar, uvms.xdot.vpos,  0.0001,   0.01, 10); % Ex1 position control task to reach the goal with the <v> frame
     [Qp, ydotbar] = iCAT_task(uvms.A.vatt,  uvms.Jvatt,    Qp, ydotbar, uvms.xdot.vatt,  0.0001,   0.01, 10); % Ex1 altitude control task to reach the goal with the <v> frame
     [Qp, ydotbar] = iCAT_task(uvms.A.lr,  uvms.Jlr,    Qp, ydotbar, uvms.xdot.lr,  0.0001,   0.01, 10); % Ex1 position control task to reach the goal with the <v> frame
     [Qp, ydotbar] = iCAT_task(uvms.A.la,  uvms.Jla,    Qp, ydotbar, uvms.xdot.la,  0.0001,   0.01, 10); % Ex3 landing task 
%      [Qp, ydotbar] = iCAT_task(uvms.A.t,  uvms.Jt,    Qp, ydotbar, uvms.xdot.t,  0.0001,   0.01, 10); % tool frame task (e.e. (tool frame) reaches the goal )
     
     %[....]
     [Qp, ydotbar] = iCAT_task( eye(13),   eye(13),    Qp, ydotbar, zeros(13,1),  0.0001,   0.01, 10);    % it stops the movement (this task should be the last one)
     
    
    
    % split y in q_dot and p_dot  for integration
    uvms.q_dot = ydotbar(1:7);
    uvms.p_dot = ydotbar(8:13);
    
    % Integration
	uvms.q = uvms.q + uvms.q_dot*deltat;
    
    %% Ex underactuation indtroduce some noise on wx in order to make it
    % Supose underactuation of roll (w_x), to simulate it we introduce some
    % disturbances on rool, try to run the code without compensation 
    % This disturbance affect the dynamics of the e.e. (put tool task under "ha")
    
    %uvms.p_dot(4) = 0.5*sin(2*pi*0.5*t);
    %% 
    % beware: p_dot should be projected on <v> !!!!!!!!!!!!!!!!!!!!!!!!!!!
    uvms.p = integrate_vehicle(uvms.p, uvms.p_dot, deltat); 
    
    % check if the mission phase should be changed
    mission.phase_time = mission.phase_time + deltat; % to increment the time variable of the mission 
    [uvms, mission] = UpdateMissionPhase(uvms, mission);
    
    % send packets to Unity viewer
    SendUdpPackets(uvms,wuRw,vRvu,uArm,uVehicle);
        
    % collect data for plots
    plt = UpdateDataPlot(plt,uvms,t,loop);
    loop = loop + 1;
   
    % add debug prints here
    if (mod(t,0.1) == 0)
        t = mission.phase_time
        %uvms.sensorDistance  % sensor distance
%         [w_vang,w_vlin] = CartError(uvms.wTgv , uvms.wTv);  % real distance
%          floor = w_vlin(3) - uvms.v_altitude
%         goal_distance = norm(w_vlin)
        %nlin = norm(w_vlin);
        %nang = norm(w_vang);
        
        phase = mission.phase % which action is executed 
        %goal_plan_distance = uvms.plan_goal_dist % for Ex 3
        %activ = uvms.A.act
%         alt = uvms.v_altitude
         uvms.v_rho_r
    end

    % enable this to have the simulation approximately evolving like real
    % time. Remove to go as fast as possible
    SlowdownToRealtime(deltat);
end

fclose(uVehicle);
fclose(uArm);

PrintPlot(plt);

end