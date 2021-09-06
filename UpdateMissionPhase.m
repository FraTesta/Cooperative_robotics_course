function [uvms, mission] = UpdateMissionPhase(uvms, mission)
% This functions handles the transitions between multiple actions

% mission.phase_time (declared in the MainRobust) is a variable that count
% the time passed, which we can use to generate transitions (see notes)

%% Action Implemented:
% A1: horizontal Altitude and Vehicle position and orientation
% A1: tool control and horizontal altitude

%% Mission for robustsim
%% Ex 2
      switch mission.phase % initialize to 1 
        case 1 % action A1
            uvms.Aa.vpos = eye(3); % active 
            uvms.Aa.vatt = eye(3); % active
            uvms.Aa.ha = eye(1); % it's a scalar 
            uvms.Aa.t = zeros(6); % it's deactivated 
            uvms.Aa.act = eye(1);
            uvms.Aa.la = zeros(1);
            uvms.Aa.mu = 0;
            %criteria on which we change action 
            [w_vang,w_vlin] = CartError(uvms.wTgv , uvms.wTv); % distance from the goal 
            if(norm(w_vlin(1:2)) < 0.15) % if it is under 10 cm
                mission.phase = 2; % switch to second action phase 
                uvms.changePhaseTime = mission.phase_time;
                mission.phase_time = 0; % reset the time variable for the next activation and dectivation functions 

            end 
            
           case 2
            uvms.Aa.t = zeros(6);           
            uvms.Aa.ha = eye(1); % keep horizontal altitude active
            %deactivate vpose and vatt
            uvms.Aa.vpos = zeros(3);
            uvms.Aa.vatt = zeros(3);
            uvms.Aa.la = eye(1);
            uvms.Aa.act = zeros(1);
      end
%% Ex 4
      switch mission.phase % initialize to 1 
        case 1 % action A1
            uvms.Aa.vpos = eye(3); % active 
            uvms.Aa.vatt = eye(3); % active
            uvms.Aa.ha = eye(1); % it's a scalar 
            uvms.Aa.t = zeros(6); % it's deactivated 
            uvms.Aa.act = eye(1);
            uvms.Aa.la = zeros(1);
            uvms.Aa.mu = 0;
            %criteria on which we change action 
            [w_vang,w_vlin] = CartError(uvms.wTgv , uvms.wTv); % distance from the goal 
            if(norm(w_vlin(1:2)) < 0.15) % if it is under 10 cm
                mission.phase = 2; % switch to second action phase 
                uvms.changePhaseTime = mission.phase_time;
                mission.phase_time = 0; % reset the time variable for the next activation and dectivation functions 

            end 
       case 2
            uvms.Aa.t = zeros(6);           
            uvms.Aa.ha = eye(1);
            uvms.Aa.vpos = zeros(3);
            uvms.Aa.vatt = zeros(3);
            uvms.Aa.la = eye(1);
            uvms.Aa.lr = eye(1);
            uvms.Aa.act = zeros(1);
           if(uvms.v_altitude < 0.05) % if it is under 10 cm
                uvms.mission.phase  = 3;
                uvms.changePhaseTime2 = mission.phase_time;
                uvms.mission.phase_time = 0;
           end
      case 3
            uvms.Aa.t = eye(6);           
            uvms.Aa.ha = eye(1); 
            uvms.Aa.vpos = zeros(3);
            uvms.Aa.vatt = zeros(3);
            uvms.Aa.la = eye(1);
            uvms.Aa.lr = eye(1);
            uvms.Aa.act = zeros(1);
      end 
%% Bho
%    switch mission.phase % initialize to 1 
%        case 1 % action A1
%             uvms.Aa.vpos = eye(3); % active 
%             uvms.Aa.vatt = eye(3); % active
%             uvms.Aa.ha = eye(1); % it's a scalar 
%             uvms.Aa.t = zeros(6); % it's deactivated 
%             
%             % criteria on which we change action 
%             [w_vang,w_vlin] = CartError(uvms.wTgv , uvms.wTv); % distance from the goal 
%             if(norm(w_vlin) < 0.1) % if it is under 10 cm
%                 uvms.mission.phase  = 2; % switch to second action phase 
%                 uvms.mission.phase_time = 0; % reset the time variable for the next activation and dectivation functions 
%             end 
% %             
%             %% As3 mission for landing and alignment with the rock
%             uvms.Aa.vpos = eye(3);
%             uvms.Aa.vatt = eye(3);
%             uvms.Aa.ha = eye(1);
%             uvms.Aa.act = eye(1);
%             uvms.Aa.la = 0;
%             uvms.Aa.lr = 0;
%             uvms.Aa.t = zeros(6);
%             uvms.Aa.vc = zeros(6);
%             uvms.Aa.jl = zeros(7);
%             [w_vang,w_vlin] = CartError(uvms.wTgv , uvms.wTv); % distance from the goal
%             if(norm(w_vlin(1:2)) < 0.2)
%                 uvms.mission.phase  = 2;
%                 uvms.mission.phase_time = 0;
%             end
% %                         
% %             
%         case 2

% % %             uvms.Aa.t = IncreasingBellShapedFunction(0, 2, 0, 1 , mission.phase_time) * eye(6);           
% % %             uvms.Aa.ha = eye(1); % keep horizontal altitude active
% % %             %deactivate vpose and vatt
% % %             uvms.Aa.vpos = DecreasingBellShapedFunction(0, 2, 0, 1 , mission.phase_time) * eye(3);
% % %             uvms.Aa.vatt = DecreasingBellShapedFunction(0, 2, 0, 1 , mission.phase_time) * eye(3);
%% As3 mission for landing and alignment with the rock
%             uvms.Aa.vpos = zeros(3);
%             uvms.Aa.vatt = zeros(3);
%             uvms.Aa.ha = eye(1);
% 
%             uvms.Aa.t = zeros(6);
%             uvms.Aa.act = 0;
%             uvms.Aa.la = IncreasingBellShapedFunction(0, 1, 0, 1, mission.phase_time);
%             uvms.Aa.lr = IncreasingBellShapedFunction(0, 0.5, 0.5, 1, norm(uvms.v_rho_r));
%             uvms.Aa.lr = IncreasingBellShapedFunction(0, 1.57, 0, 0.5, norm(uvms.v_rho_r));
%             uvms.Aa.lr = 1;
%             uvms.Aa.vc = zeros(6);
%             uvms.Aa.jl = zeros(7);
%             if(uvms.v_altitude < 0.05) % if it is under 10 cm
%                 uvms.mission.phase  = 3;
%                 uvms.mission.phase_time = 0;
%            end
%                           
%         case 3
%             uvms.Aa.vpos = zeros(3);
%             uvms.Aa.vatt = zeros(3);
%             uvms.Aa.ha = eye(1);
%             %uvms.Aa.act = 0;
%             uvms.Aa.act = 0;
%             uvms.Aa.la = 0;
%             uvms.Aa.lr = 0;
%             uvms.Aa.t = IncreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time);
%             uvms.Aa.vc = eye(6);
%             uvms.Aa.jl = eye(7);
% end

%% Mission for Dextrov 

%% Ex 5
% EX5.1 No target position, the tool rules
% uvms.Aa.ha = eye(1);
% uvms.Aa.t = eye(6);
% % uvms.Aa.ps = eye(4);
% uvms.Aa.ps = zeros(4);
% uvms.Aa.jl = eye(7);

%% Ex 6
%     switch mission.phase % initialize to 1 
%          case 1
%             uvms.Aa.vpos = eye(3); % active 
%             uvms.Aa.vatt = eye(3); % active
%             uvms.Aa.ha = eye(1); % it's a scalar 
%             uvms.Aa.t = eye(6);%zeros(6); % it's deactivated 
%             uvms.Aa.ps = zeros(4);
%             uvms.Aa.ua = zeros(6);
%             % planar dinstance from the goal (only along x and y)
%             if(norm(uvms.wTgv - uvms.wTv) < 0.5) % if it is under 10 cm
%                 disp('---Navigation Accomplished---')
%                 uvms.mission.phase  = 2;
%                 uvms.mission.phase_time = 0;
%             end
%          case 2
%             uvms.Aa.vpos = eye(3); % active 
%             uvms.Aa.vatt = eye(3); % active
%             uvms.Aa.ha = eye(1); % it's a scalar 
%             uvms.Aa.t = IncreasingBellShapedFunction(0, 1, 0, 1, mission.phase_time);
% %             uvms.Aa.ua = eye(6);
%             uvms.Aa.ps = eye(4);
%     end
end

