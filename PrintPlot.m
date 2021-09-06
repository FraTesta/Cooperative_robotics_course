function [ ] = PrintPlot( plt, uvms )

% some predefined plots
% you can add your own

% figure(1);
% subplot(2,1,1);
% hplot = plot(plt.t, plt.q);
% set(hplot, 'LineWidth', 1);
% hold on;
% hplot2 = plot(plt.t, plt.jlmin);
% set(hplot2, 'LineWidth', 1);
% legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
% subplot(2,1,2);
% hplot = plot(plt.t, plt.q_dot);
% set(hplot, 'LineWidth', 1);
% legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7');

% 
% figure(2);
% subplot(3,1,1);
figure(2)
subplot(2,1,1)
hplot = plot(plt.t, plt.p);
set(hplot, 'LineWidth', 1);
legend('x','y','z','roll','pitch','yaw');
subplot(2,1,2);
hplot = plot(plt.t, plt.p_dot);
set(hplot, 'LineWidth', 1);
legend('xdot', 'ydot','zdot','\omega_x','\omega_y','\omega_z');
% 
% 
% figure(3);
% hplot = plot(plt.t, plt.a(1:7,:));
% set(hplot, 'LineWidth', 2);
% legend('Ajl_11','Ajl_22','Ajl_33','Ajl_44','Ajl_55','Ajl_66','Ajl_77');
% 
% figure(4);
% hplot = plot(plt.t, plt.a(8:9,:));
% set(hplot, 'LineWidth', 2);
% legend('Amu', 'Aha');

%% Path to the goal 
figure(5);
plot3(plt.v_goal.x, plt.v_goal.y, plt.v_goal.z); % v path
% hold on;
% plot3(plt.tool_path.x, plt.tool_path.y, plt.tool_path.z,'Color','g'); % tool path
hold on;
scatter3(plt.v_initPos(1), plt.v_initPos(2), plt.v_initPos(3),'MarkerFaceColor',[0 1 0]); % v start
hold on;
scatter3(plt.final_v_pose(1), plt.final_v_pose(2), plt.final_v_pose(3)); % v final
% hold on;
% scatter3(plt.final_t_pose(1), plt.final_t_pose(2), plt.final_t_pose(3),'MarkerFaceColor',[0 1 0]); % tool final
hold on;
scatter3(plt.v_goalPos(1), plt.v_goalPos(2), plt.v_goalPos(3),'MarkerFaceColor',[1 0 0]); % vehicle goal
% hold on;
% scatter3(plt.goalPos(1), plt.goalPos(2), plt.goalPos(3),'MarkerFaceColor',[0 0 1]); % tool goal
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
title("Path towards goals");
%set(hplot, 'LineWidth', 1);
% legend( 'Path of the vehicle frame','Path of the tool frame','Vehicle Start position',' Vehicle Final position','Final tool position','Vehicle Goal position','Tool Goal position');
%% Without Tool
legend( 'Path of the vehicle frame','Vehicle Start position',' Vehicle Final position','Vehicle Goal position');

%% Horizontal Attitude Misalignment vector  (unit vector)
figure(6);
% title("Misalignment of the vehicle w.r.t the seafloor")
subplot(3,1,1);
hplot = plot(plt.t, plt.misAlig.x);
set(hplot,'Color','red', 'LineWidth', 2);
legend( 'Misalignment along x');
xlabel('time [s]');
ylabel('[rad]');
subplot(3,1,2);
hplot = plot(plt.t, plt.misAlig.y);
set(hplot, 'LineWidth', 2);
legend( 'Misalignment along y');
xlabel('time [s]');
ylabel('[rad]');
subplot(3,1,3);
hplot = plot(plt.t, plt.misAlig.z);
set(hplot,'Color','green', 'LineWidth', 2);
% legend( 'Misalignment along z');
xlabel('time [s]');
ylabel('[rad]');

%% rho norm
figure
hplot = plot(plt.t, plt.v_rho);
set(hplot,'LineWidth', 2)
legend( 'Misalignment');
xlabel('time [s]');
ylabel('[rad]');

%% Dinstance and misalignment norm w.r.t. the tool and vehicle goals 

figure(7)
%subplot(2,2,1);
subplot(1,2,1); %Without tool
hplot = plot(plt.t,  plt.v_misNorm);
%title('Misalignment norm w.r.t. the vehicle goal');
xlabel('t[s]');
ylabel('[rad]');
set(hplot, 'LineWidth', 2);
legend('Misalignment norm');
% subplot(2,2,2);
subplot(1,2,2); % Without Tool
hplot = plot(plt.t, plt.v_distNorm);
% title('Distance norm w.r.t. the vehicle goal');
xlabel('t[s]');
ylabel('[m]');
set(hplot, 'LineWidth', 2);
legend('Distance norm');
% subplot(2,2,3);
% hplot = plot(plt.t, plt.g_misNorm);
% title('Misalignment norm w.r.t. the tool goal');
% xlabel('t[s]');
% ylabel('[rad]');
% set(hplot,'Color','green', 'LineWidth', 2);
% legend('Misalignment norm');
% subplot(2,2,4);
% hplot = plot(plt.t, plt.g_distNorm);
% title('Distance norm w.r.t. the tool goal');
% xlabel('t[s]');
% ylabel('[m]');
% set(hplot,'Color','green', 'LineWidth', 2);
% legend('Misalignment norm');
%% Horizontal altidtude Activation function
figure(8);
hplot = plot(plt.t, plt.misActFunc);
title('Activation Function of Horizontal altitude');
xlabel('time [s]');
ylabel('Activation value');
set( hplot, 'LineWidth', 2);
ylim([0 1]);
legend( 'Activation function');
%% Altitude 
% figure(9);
% subplot(2,1,1);
% hplot1 = plot(plt.t, plt.altitude);
% set( hplot1, 'Color','green','LineWidth', 2);
% title('Altitude of the vehicle');
% xlabel('time [s]');
% ylabel('altitude [m]');
% hold on;
% hplot2 = plot(plt.t, plt.unsafeAlt,'--');
% set( hplot2,'Color','red', 'LineWidth', 2);
% 
% hold on;
% hplot3 = plot(plt.t, plt.minTre,'--');
% set( hplot3,'Color','cyan', 'LineWidth', 2);
% hold on;
% hplot4 = plot(plt.t, plt.maxTre,'--');
% set( hplot4,'Color','blue', 'LineWidth', 2);
% legend( 'Altitude of the vehicle','Unsafe altitude','Min activation threshold','Max activation threshold');
% 
% subplot(2,1,2);
% hplot = plot(plt.t, plt.A.act);
% set( hplot, 'LineWidth', 2);
% title('Activation of the safety altitude control');
% xlabel('time [s]');
% ylabel('altitude [m]');
% legend( 'Activation function ');
%% Landing
% figure(10);
% subplot(2,1,1);
% hplot1 = plot(plt.t, plt.altitude);
% set( hplot1, 'Color','green','LineWidth', 2);
% title('Altitude in landing task');
% xlabel('time [s]');
% ylabel('altitude [m]');
% legend( 'Altitude of the vehicle');
% 
% subplot(2,1,2);
% hplot1 = plot(plt.t, plt.Ala);
% set( hplot1,'LineWidth', 2);
% title('Activation of landing task');
% xlabel('time [s]');
% ylabel('altitude [m]');
% legend( 'Activation function');
%% align with rock
% figure(11);
% subplot(2,2,1);
% hplot1 = plot(plt.t, plt.misRock);
% set( hplot1,'Color','red','LineWidth', 2);
% hold on;
% hplot1 = plot(plt.t, plt.altitude);
% set( hplot1,'Color','green','LineWidth', 2);
% title('Misalignment w.r.t. the rock');
% legend('Misalignment w.r.t. the rock','Vehicle altitude ')
% xlabel('time [s]');
% ylabel('[rad]');
% 
% subplot(2,2,2);
% hplot2 = plot(plt.t, plt.A.mis);
% set( hplot2,'LineWidth', 2);
% title('Activation function (misalignment)');
% xlabel('time [s]');
% ylabel('Activation');
% subplot(2,2,3);
% hplot3 = plot(plt.t, plt.A.alt);
% set( hplot3,'LineWidth', 2);
% title('Activation function (Altitude)');
% xlabel('time [s]');
% ylabel('Activation');
% 
% subplot(2,2,4);
% hplot4 = plot(plt.t, plt.A.all);
% set( hplot4,'Color','green','LineWidth', 2);
% title('Complete Activation function');
% xlabel('time [s]');
% ylabel('Activation')
%% Joits limits
% figure(12);
% title('Joint Limits Activation functions');
% subplot(2,1,1);
% title('Activation min joint limits task');
% hplot1 = plot(plt.t, plt.A.jl_min(1:7,:));
% set( hplot1,'LineWidth', 2);
% legend('Aq1','Aq2','Aq3','Aq4','Aq5','Aq6','Aq7');
% xlabel('time [s]');
% ylabel('Activation');
% 
% subplot(2,1,2);
% title('Activation max joint limits task');
% hplot1 = plot(plt.t, plt.A.jl_max(1:7,:));
% set( hplot1,'LineWidth', 2);
% legend('Aq1','Aq2','Aq3','Aq4','Aq5','Aq6','Aq7');
% xlabel('time [s]');
% ylabel('Activation');
%% Prefered Shape 
% figure(13);
% subplot(2,2,1);
% hplot1 = plot(plt.t, plt.q(1,:));
% set( hplot1,'LineWidth', 2);
% hold on;
% hplot2 = plot(plt.t, plt.prefShape(1,:),'r--');
% set( hplot2,'LineWidth', 2);
% legend('q_1','preferred shape q_1');
% xlabel('time [s]');
% ylabel('Angular Position [rad]')
% 
% subplot(2,2,2);
% hplot1 = plot(plt.t, plt.q(2,:));
% set( hplot1,'LineWidth', 2);
% hold on;
% hplot2 = plot(plt.t, plt.prefShape(2,:),'r--');
% set( hplot2,'LineWidth', 2);
% legend('q_2','preferred shape q_2');
% xlabel('time [s]');
% ylabel('Angular Position [rad]')
% 
% subplot(2,2,3);
% hplot1 = plot(plt.t, plt.q(3,:));
% set( hplot1,'LineWidth', 2);
% hold on;
% hplot2 = plot(plt.t, plt.prefShape(3,:),'r--');
% set( hplot2,'LineWidth', 2);
% legend('q_3','preferred shape q_3');
% xlabel('time [s]');
% ylabel('Angular Position [rad]')
% 
% subplot(2,2,4);
% hplot1 = plot(plt.t, plt.q(4,:));
% set( hplot1,'LineWidth', 2);
% 
% hold on;
% hplot2 = plot(plt.t, plt.prefShape(4,:),'r--');
% set( hplot2,'LineWidth', 2);
% legend('q_4','preferred shape q_4');
% xlabel('time [s]');
% ylabel('Angular Position [rad]')

%% Ex 6 
% figure(15);
% title('Activation Function Underactuated constraints');
% hplot1 = plot(plt.t, plt.A.ua);
% set( hplot1,'LineWidth', 2);
end

