function [uvms, mission] = UpdateMissionPhase(uvms, mission)
% This functions handles the transitions between multiple actions

% mission.phase_time (declared in the MainRobust) is a variable that count
% the time passed, which we can use to generate transitions (see notes)

%% Action Implemented:
% A1: horizontal Altitude and Vehicle position and orientation
% A1: tool control and horizontal altitude

% ATTT!!!! the tool control must be under the vpost and vatt priority in
% the priority list  !!!!!!!!!!!!!!!!!!!!!!!!

    switch mission.phase % initialize to 1 
        case 1 % action A1
%             uvms.Aa.vpos = eye(3); % active 
%             uvms.Aa.vatt = eye(3); % active
%             uvms.Aa.ha = eye(1); % it's a scalar 
%             uvms.Aa.t = zeros(6); % it's deactivated 
%             
%             % criteria on which we change action 
%             [w_vang,w_vlin] = CartError(uvms.wTgv , uvms.wTv); % distance from the goal 
%             if(norm(w_vlin) < 0.1) % if it is under 10 cm
%                 mission.phase = 2; % switch to second action phase 
%                 mission.phase_time = 0; % reset the time variable for the next activation and dectivation functions 
%             end 
%             
            %% Ex3
            uvms.Aa.vpos = eye(3);
            uvms.Aa.vatt = eye(3);
            uvms.Aa.ha = eye(1);
            uvms.Aa.act = eye(1);
            uvms.Aa.la = 0;
            uvms.Aa.lr = 0;
            [w_vang,w_vlin] = CartError(uvms.wTgv , uvms.wTv); % distance from the goal
            uvms.plan_goal_dist = sqrt(w_vlin(1)^2 + w_vlin(2)^2); % planar dinstance from the goal (only along x and y)
            if(uvms.plan_goal_dist < 0.05) % if it is under 10 cm
                mission.phase = 2;
                mission.phase_time = 0;
            end
                        
            
        case 2
%             % I activate the tool control and deactivate the vehicle
%             % controls but maintaining the horizontal altitude control
%             uvms.Aa.t = IncreasingBellShapedFunction(0, 2, 0, 1 , mission.phase_time) * eye(6);
%             % Returns tha activation function which depends on time pass (mission.phase_time)
%             % I multiply for eye(6) since the IncreasingBell.. returns a
%             % scalar so I have to multiply for I6x6 to give the proper
%             % dimension
%             
%             uvms.Aa.ha = eye(1); % keep horizontal altitude active
%             %deactivate vpose and vatt
%             uvms.Aa.vpos = DecreasingBellShapedFunction(0, 2, 0, 1 , mission.phase_time) * eye(3);
%             uvms.Aa.vatt = DecreasingBellShapedFunction(0, 2, 0, 1 , mission.phase_time) * eye(3);
            %% Ex3
            uvms.Aa.vpos = zeros(3);
            uvms.Aa.vatt = zeros(3);
            uvms.Aa.ha = eye(1);
            %uvms.Aa.act = DecreasingBellShapedFunction(0, 2, 0, 1 , mission.phase_time);
            uvms.Aa.act = 0;
            uvms.Aa.la = IncreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time);
            uvms.Aa.lr = eye(1);
    end
end

