function [uvms, mission] = UpdateMissionPhase(uvms, mission)
% This functions handles the transitions between multiple actions

% mission.phase_time (declared in the MainRobust) is a variable that count
% the time passed, which we can use to generate transitions (see notes)

%% Action Implemented:
% A1: horizontal Altitude and Vehicle position and orientation
% A1: tool control and horizontal altitude

%% Mission for robustsim
% ex2 = 2
% ex4 = 4
% ex4 current = 41 
uvms.EX = 3;
%% Ex 2
if uvms.EX == 2
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
end

%% EX3
if uvms.EX == 3
   switch mission.phase % initialize to 1 
        case 1 % action A1
            uvms.Aa.vpos = IncreasingBellShapedFunction(0,2, 0, 1, mission.phase_time); % active 
            uvms.Aa.vatt = IncreasingBellShapedFunction(0,2, 0, 1, mission.phase_time); % active
            uvms.Aa.ha = eye(1); % it's a scalar 
            uvms.Aa.t = zeros(6); % it's deactivated 
            uvms.Aa.act = eye(1);
            uvms.Aa.la = zeros(1);
            uvms.Aa.lr = zeros(1);
            uvms.Aa.mu = 0;
            %criteria on which we change action 
            [w_vang,w_vlin] = CartError(uvms.wTgv , uvms.wTv); % distance from the goal 
            if(norm(w_vlin(1:2)) < 0.15) % if it is under 10 cm
                mission.phase = 2; % switch to second action phase 
                uvms.changePhaseTime = mission.phase_time;
                mission.phase_time = 0; % reset the time variable for the next activation and dectivation functions 

            end 
            
           case 2 
            uvms.Aa.t = IncreasingBellShapedFunction(0,0.5, 0, 1, mission.phase_time);
            uvms.Aa.lr = eye(1);
%             uvms.Aa.lr = IncreasingBellShapedFunction(0, 1.57, 0.1, 0.5, norm(uvms.v_rho_r));
            uvms.Aa.ha = eye(1); 
            uvms.Aa.vpos = DecreasingBellShapedFunction(0,1, 0, 1, mission.phase_time);
            uvms.Aa.vatt = DecreasingBellShapedFunction(0,1, 0, 1, mission.phase_time);
            uvms.Aa.vc = eye(6);
            uvms.Aa.la = eye(1);
            uvms.Aa.act = zeros(1);
            uvms.Aa.mu = 1;
   end 
 end 
      

%% Ex 4
if uvms.EX == 4
      switch mission.phase % initialize to 1 
        case 1 % action A1
            uvms.Aa.vpos = eye(3); % active 
            uvms.Aa.vatt = eye(3); % active
            uvms.Aa.ha = eye(1); % it's a scalar 
            uvms.Aa.t = zeros(6); % it's deactivated 
            uvms.Aa.act = eye(1);
            uvms.Aa.la = zeros(1);
            uvms.Aa.mu = 0;
            uvms.Aa.lr = 0;
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
            uvms.Aa.la = IncreasingBellShapedFunction(0, 0.3, 0, 1, uvms.v_altitude);
            uvms.Aa.lr = IncreasingBellShapedFunction(0, 0.5, 0, 1,mission.phase_time);
            uvms.Aa.act = zeros(1);
           if(uvms.v_altitude < 0.15) % if it is under 10 cm
                mission.phase  = 3;
                uvms.changePhaseTime2 = uvms.changePhaseTime + mission.phase_time;
                mission.phase_time = 0;
           end
      case 3
            uvms.Aa.t = IncreasingBellShapedFunction(0,0.5, 0, 1, mission.phase_time);           
            uvms.Aa.ha = eye(1); 
            uvms.Aa.vpos = zeros(3);
            uvms.Aa.vatt = zeros(3);
            uvms.Aa.vc = eye(6);
            uvms.Aa.la = zeros(1);
            uvms.Aa.lr = zeros(1);
            uvms.Aa.act = zeros(1);
            uvms.Aa.mu = 1;
      end 
end
% With current effect 
if uvms.EX == 41
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
            uvms.Aa.t = IncreasingBellShapedFunction(0,0.5, 0, 1, mission.phase_time);           
            uvms.Aa.ha = eye(1); 
            uvms.Aa.vpos = zeros(3);
            uvms.Aa.vatt = zeros(3);
            uvms.Aa.vc = eye(6);
            uvms.Aa.la = zeros(1);
            uvms.Aa.lr = zeros(1);
            uvms.Aa.act = zeros(1);
            uvms.Aa.mu = 1;
      end 
   end 
if uvms.EX == 42
      switch mission.phase % initialize to 1 
        case 1 % action A1
            uvms.Aa.vpos = eye(3); % active 
            uvms.Aa.vatt = eye(3); % active
            uvms.Aa.ha = eye(1); % it's a scalar 
            uvms.Aa.t = zeros(6); % it's deactivated 
            uvms.Aa.act = eye(1);
            uvms.Aa.la = zeros(1);
            uvms.Aa.mu = 0;
            uvms.Aa.lr = 0;
            uvms.Aa.jl = zeros(7);
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
            uvms.Aa.la = IncreasingBellShapedFunction(0, 0.3, 0, 1, uvms.v_altitude);
            uvms.Aa.lr = IncreasingBellShapedFunction(0, 0.5, 0, 1,mission.phase_time);
            uvms.Aa.act = zeros(1);
           if(uvms.v_altitude < 0.15) % if it is under 10 cm
                mission.phase  = 3;
                uvms.changePhaseTime2 = uvms.changePhaseTime + mission.phase_time;
                mission.phase_time = 0;
           end
      case 3
            uvms.Aa.t = IncreasingBellShapedFunction(0,0.5, 0, 1, mission.phase_time);           
            uvms.Aa.ha = eye(1); 
            uvms.Aa.vpos = zeros(3);
            uvms.Aa.vatt = zeros(3);
            uvms.Aa.vc = eye(6);
            uvms.Aa.la = zeros(1);
            uvms.Aa.lr = zeros(1);
            uvms.Aa.act = zeros(1);
            uvms.Aa.mu = 1;
            uvms.Aa.jl = eye(7);
      end 
end 

            




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
%                 mission.phase  = 3;
%                 mission.phase_time = 0;
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

end

