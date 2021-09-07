function [ plt ] = UpdateDataPlot( plt, uvms, t, loop, mission )

% this function samples the variables contained in the structure uvms
% and saves them in arrays inside the struct plt
% this allows to have the time history of the data for later plots

% you can add whatever sampling you need to do additional plots
% plots are done in the PrintPlot.m script

plt.t(loop) = t;

plt.toolPos(:, loop) = uvms.wTt(1:3,4);

plt.q(:, loop) = uvms.q;
plt.q_dot(:, loop) = uvms.q_dot;

plt.jlmin(:,loop) = uvms.jlmin;
plt.jlmax(:,loop) = uvms.jlmax;

plt.p(:, loop) = uvms.p;
plt.p_dot(:, loop) = uvms.p_dot;

%plt.xdot_jl(:, loop) = uvms.xdot.jl;
%plt.xdot_mu(:, loop) = uvms.xdot.mu;
plt.xdot_t(:, loop) =  blkdiag(uvms.wTv(1:3,1:3), uvms.wTv(1:3,1:3))*uvms.xdot.t;

plt.a(1:7, loop) = diag(uvms.A.jl);
plt.a(8, loop) = uvms.A.mu;
plt.a(9, loop) = uvms.A.ha(1,1);

plt.toolx(:,loop) = uvms.wTt(1,4);
plt.tooly(:,loop) = uvms.wTt(2,4);

plt.A.t(1:6, loop)= diag(uvms.A.t);
%% Path towards goal 
uvms.wTgv = [uvms.wRgv uvms.vgoalPosition; 0 0 0 1];
plt.v_goal.x(:,loop) = uvms.wTgv(1,4);
plt.v_goal.y(:,loop) = uvms.wTgv(2,4);
plt.v_goal.z(:,loop) = uvms.wTgv(3,4);

%% path of the vehicle frame
plt.v_goal.x(:,loop) = uvms.wTv(1,4);
plt.v_goal.y(:,loop) = uvms.wTv(2,4);
plt.v_goal.z(:,loop) = uvms.wTv(3,4);
%% path of the tool frame
plt.tool_path.x(:,loop) = uvms.wTt(1,4);
plt.tool_path.y(:,loop) = uvms.wTt(2,4);
plt.tool_path.z(:,loop) = uvms.wTt(3,4);

plt.final_v_pose = uvms.wTv(1:3,4);
plt.final_t_pose = uvms.wTt(1:3,4);

%% Horizontal attitude control
plt.misAlig.x(:,loop) = uvms.v_rho(1);% misalignment vector 
plt.misAlig.y(:,loop) = uvms.v_rho(2);
plt.misAlig.z(:,loop) = uvms.v_rho(3);
plt.v_rho(:,loop) = norm(uvms.v_rho);


%% misalignment vector components 
% plt.misAlig.x(:,loop) = uvms.v_n(1);
% plt.misAlig.y(:,loop) = uvms.v_n(2);
% plt.misAlig.z(:,loop) = uvms.v_n(3);

%% Horizontal Attitude, Misalignment and distance norm w.r.t. the vehicle and tool goals 
% vehicle errors
[w_vgang,w_vglin] = CartError(uvms.wTgv , uvms.wTv);
plt.v_misNorm(:,loop) = norm(w_vgang);
plt.v_distNorm(:,loop) = norm(w_vglin);
% tool errors 
% [w_vang,w_vlin] = CartError(uvms.wTg , uvms.wTv);
% plt.g_misNorm(:,loop) = norm(w_vang);
% plt.g_distNorm(:,loop) = norm(w_vlin);
[w_vang,w_vlin] = CartError(uvms.wTg , uvms.wTt);
plt.g_misNorm(:,loop) = norm(w_vang);
plt.g_distNorm(:,loop) = norm(w_vlin);
plt.misActFunc(:,loop) = uvms.A.ha;
%% Altitude 
plt.altitude(:,loop) = uvms.v_altitude;
plt.unsafeAlt(:,loop) = 1;
plt.A.act(:,loop) = uvms.A.act;

plt.minTre(:,loop) = uvms.Aact.minTre;
plt.maxTre(:,loop) = uvms.Aact.maxTre;
%% Landing
plt.floor(:,loop) = w_vglin(3) - uvms.v_altitude;

plt.Ala(:,loop) = uvms.A.la;
%% align with rock
plt.misRock(:,loop) = norm(uvms.v_rho_r);
plt.A.mis(:,loop) = uvms.Aa.lr;
plt.A.alt(:,loop) = DecreasingBellShapedFunction(0.8, 1.5, 0, 1 , uvms.v_altitude);
plt.A.all(:,loop) = uvms.A.lr;
%% V Null vell 
plt.A.vc(1:6, loop) = diag(uvms.A.vc); 
%% Joint limits 
plt.A.jl_min(1:7,loop) = diag(uvms.A.jl_min);
plt.A.jl_max(1:7,loop) = diag(uvms.A.jl_max);
%% Prefered Shape
plt.prefShape(:,loop) = uvms.pref_shape;
%% Ex6 
% plt.A.ua(:,loop) = uvms.A.ua(4,4);
%% Update time
if mission.phase == 2  
    plt.changePhaseTime = uvms.changePhaseTime;
end 
if mission.phase == 3
    plt.changePhaseTime2 = uvms.changePhaseTime2;
end