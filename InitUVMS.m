function [uvms] = InitUVMS(robotname)
% CONTAINS ALL THE VARIABLES INITIALIZATIONS, VECTORS, MATRICES ... WHICH
% DEFINES THE ROBOT STRUCTURE 

% uvms.vTb
% transformation matrix between the arm base w.r.t. vehicle frame
% expresses how the base of the arm is attached to the vehicle
% do NOT change, since it must be coherent with the visualization tool
if (strcmp(robotname, 'DexROV'))    
    % do NOT change
    uvms.vTb = [rotation(pi, 0, pi) [0.167 0 -0.43]'; 0 0 0 1]; 
else
    if (strcmp(robotname, 'Robust'))
        % do NOT change
        uvms.vTb = [rotation(0, 0, pi) [0.85 0 -0.42]'; 0 0 0 1];
    end
end

uvms.q_dot = [0 0 0 0 0 0 0]';
uvms.p_dot = [0 0 0 0 0 0]';

% joint limits corresponding to the actual MARIS arm configuration
uvms.jlmin  = [-2.9;-1.6;-2.9;-2.95;-2.9;-1.65;-2.8];
uvms.jlmax  = [2.9;1.65;2.9;0.01;2.9;1.25;2.8];

% to be computed at each time step
uvms.wTv = eye(4,4);
uvms.wTt = eye(4,4);
uvms.vTw = eye(4,4);
uvms.vTe = eye(4,4);
uvms.vTt = eye(4,4);
uvms.vTg = eye(4,4);
uvms.Ste = eye(6,6);
uvms.bTe = eye(4,4);
uvms.bJe = eye(6,7);
uvms.djdq = zeros(6,7,7);
uvms.mu  = 0;
uvms.phi = zeros(3,1);
uvms.sensorDistance = 0;



uvms.wTgv = eye(4,4); % ex 1 take a look at UpdateTransforms to see how it's compute 

uvms.Jjl = [];
uvms.Jmu = [];
uvms.Jha = [];
uvms.Jt_a = [];
uvms.Jt_v = [];
uvms.Jt = [];
uvms.Jha = [];
uvms.Jact = []; %jacobian for altitude control task
uvms.Jua = []; % underactuated J
uvms.Jla = []; % for landing of Ex 3
uvms.Jlr = zeros(3,13); % landing aligned to the rock 
uvms.Jvc = [];
uvms.Jjl = [];
uvms.Jps = [];


uvms.xdot.jl = [];
uvms.xdot.mu = [];
uvms.xdot.ha = 0;
uvms.xdot.t = [];
uvms.xdot.ha = [];
uvms.xdot.act = []; %altitude control task for ex2
uvms.xdot.ua = [];
uvms.xdot.lr = [];
uvms.xdot.vc = [];
uvms.xdot.jl_min = [];
uvms.xdot.jl_max = [];
uvms.xdot.ps = [];
    
uvms.A.jl = zeros(7,7);
uvms.A.mu = 0;
uvms.A.ha = zeros(1,1);
uvms.A.t = zeros(6,6);
uvms.A.ha = 0;
uvms.A.act = 0;
uvms.A.ua = 0; % activation underactuated 
uvms.A.la = 0; 
uvms.A.lr = 0;
uvms.A.vc_lin = zeros(3);
uvms.A.vc_ang = zeros(3);
uvms.A.vc = zeros(6);
uvms.A.jl_min = zeros(7);
uvms.A.jl_max = zeros(7);
uvms.A.ps = zeros(4);

uvms.Aa.vpos = zeros(3);
uvms.Aa.vatt = zeros(3);
uvms.Aa.ha = zeros(1);
uvms.Aa.t = zeros(6);
uvms.Aa.act = zeros(1);
uvms.Aa.ua = zeros(6);
uvms.Aa.la = zeros(1);
uvms.Aa.lr = zeros(1);
uvms.Aa.vc = zeros(6);
uvms.Aa.jl = zeros(7);
uvms.Aa.ps = zeros(4);
uvms.Aa.mu = 0;

uvms.v_init_pose = zeros(3:1);
uvms.v_rho= zeros(3:1); % for es 2
uvms.v_n = zeros(3,1); % for es 2
uvms.v_altitude = 0;
uvms.plan_goal_dist = eye(1); % for es 3 
uvms.v_rho_r = zeros(3:1);
uvms.w_rock_center = [12.2025   37.3748  -39.8860]';
uvms.v_dp = zeros(3,1);
uvms.v_iv = zeros(3,1);
uvms.pref_shape = zeros(4,1);

 uvms.Aact.minTre = 0;
 uvms.Aact.maxTre = 0;
 
 uvms.changePhaseTime = 0;
 uvms.changePhaseTime2 = 0;
 
 uvms.EX = 0;
end

