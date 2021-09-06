% VARIABLES FOR PLOTTING
function [plt] = InitDataPlot( maxloops, uvms)
    plt.t = zeros(1, maxloops);
    plt.q = zeros(7, maxloops);
    plt.q_dot = zeros(7, maxloops);

    plt.p = zeros(6, maxloops);
    plt.p_dot = zeros(6, maxloops);

    plt.xdot_jl = zeros(7, maxloops);
    plt.xdot_mu = zeros(1, maxloops);
    plt.xdot_t = zeros(6, maxloops);

    plt.a = zeros(11, maxloops);
    
    plt.changePhaseTime = 0;
    plt.changePhaseTime2 = 0; 
    %% Path
    % Path of the vehicle frame
    plt.v_goal.x = 0;
    plt.v_goal.y = 0;
    plt.v_goal.z = 0;
    % Path of the tool frame
    plt.tool_path.x = 0;
    plt.tool_path.y = 0;
    plt.tool_path.z = 0;
    
    plt.v_initPos = uvms.v_init_pose;
    plt.goalPos = uvms.goalPosition;
     plt.v_goalPos = uvms.vgoalPosition;
    
    plt.final_v_pose = zeros(3,1);
    plt.final_t_pose = zeros(3,1);
    
    %% Misalignmet vector vheicle w.r.t. world frame
    plt.misAlig.x = 0;
    plt.misAlig.y = 0;
    plt.misAlig.z = 0;
    
    plt.v_misNorm = 0;
    plt.g_misNorm = 0;
    plt.v_distNorm = 0;
    plt.g_distNorm = 0;
%%  Altitude control 
    plt.altitude = 0;
    plt.unsafeAlt = 0;
    plt.A.act = 0;
    
    plt.minTre = 0;
    plt.maxTre = 0;
%% Landing
    plt.floor = 0;
    
    plt.Ala = 0;
    
%% landing alig with rock
    plt.misRock = 0;

    plt.A.mis = 0;
    plt.A.alt = 0;
    plt.A.all = 0;
%% Joint Limits
    plt.A.jl_min = 0;
    plt.A.jl_max = 0;
%% Prefered Shape
    plt.prefShape = [];
%% Ex6 
    plt.A.ua = 0;
end

