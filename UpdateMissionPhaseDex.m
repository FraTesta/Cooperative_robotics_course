function [uvms, mission] = UpdateMissionPhaseDex(uvms, mission)


%% Ex 5
if uvms.EX == 5
% EX5.1 No target position, the tool rules
    uvms.Aa.ha = eye(1);
    uvms.Aa.t = eye(6);
    uvms.Aa.ps = eye(4);
    uvms.Aa.jl = eye(7);
end
%% EX5.2

if uvms.EX == 52
    switch mission.phase % initialize to 1 
         case 1
            uvms.Aa.vpos = eye(3); % active 
            uvms.Aa.vatt = eye(3); % active
            uvms.Aa.ha = eye(1); % it's a scalar 
            uvms.Aa.t = zeros(6);%zeros(6); % it's deactivated 
            uvms.Aa.ps = eye(4);

%             planar dinstance from the goal (only along x and y)
            if(uvms.wTgv - uvms.wTv < 0.5) % if it is under 10 cm
                disp('---Navigation Accomplished---')
                mission.phase = 2;
                uvms.changePhaseTime = mission.phase_time;
                mission.phase_time = 0;
            end
         case 2
            uvms.Aa.mu = 1;
            uvms.Aa.vpos =  zeros(3); % active 
            uvms.Aa.vatt =  zeros(3); % active
            uvms.Aa.ha = eye(1); % it's a scalar 
            uvms.Aa.t = eye(6);
            uvms.Aa.ps = eye(4);
            end
end   
%% Ex 6
if uvms.EX == 6 
    switch mission.phase % initialize to 1 
         case 1
            uvms.Aa.vpos = eye(3); % active 
            uvms.Aa.vatt = eye(3); % active
            uvms.Aa.ha = eye(1); % it's a scalar 
            uvms.Aa.t = zeros(6);%zeros(6); % it's deactivated 
            uvms.Aa.ps = zeros(4);
            uvms.Aa.ua = zeros(6);
%             planar dinstance from the goal (only along x and y)
            if(norm(uvms.wTgv - uvms.wTv) < 0.5) % if it is under 10 cm
                disp('---Navigation Accomplished---')
                mission.phase = 2;
                uvms.changePhaseTime = mission.phase_time;
                mission.phase_time = 0;
            end
         case 2
            uvms.Aa.vpos = eye(3); % active 
            uvms.Aa.vatt = eye(3); % active
            uvms.Aa.ha = eye(1); % it's a scalar 
            uvms.Aa.t = eye(6);
            uvms.Aa.ua = eye(6);
            uvms.Aa.ps = eye(4);
            uvms.Aa.mu = 1;
    end
end
    end