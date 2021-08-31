function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here
% ATT !!! this A matrix consider the activation function of the single task
% (first term of the multiplication) and the activation func. due to the
% multiple actions switching that you can see in "UpdateMissionPhase" (second term)

% arm tool position control
%uvms.A.t = eye(6);   % da sostituire in altri es

 uvms.Aact.minTre = 10;
 uvms.Aact.maxTre = 10.3;
%% Without Actions 
 uvms.A.vpos = eye(3); 
 uvms.A.vatt = eye(3);
 uvms.A.t = eye(6);
%  uvms.A.act = DecreasingBellShapedFunction( uvms.Aact.minTre, uvms.Aact.maxTre, 0, 1 , uvms.v_altitude);
% % % uvms.A.act = IncreasingBellShapedFunction(0.3, 0.5, 0, 1, (1-uvms.sensorDistance));
%  uvms.A.ha = IncreasingBellShapedFunction(0.1, 0.2, 0, 1 , norm(uvms.v_rho)); % see the notes to see this function
%  uvms.A.la = eye(1);
% uvms.A.lr = eye(1);
uvms.A.vc_lin = eye(3);
uvms.A.vc_ang = eye(3);

%% With Multipe actions implementation

% uvms.A.vpos = eye(3) * uvms.Aa.vpos;
% uvms.A.vatt = eye(3) * uvms.Aa.vatt;
% uvms.A.ha = IncreasingBellShapedFunction(0.1, 0.2, 0, 1 , norm(uvms.v_rho)) * uvms.Aa.ha;
% uvms.A.act = DecreasingBellShapedFunction(1, 1.5, 0, 1 , uvms.v_altitude) * uvms.Aa.act;
% uvms.A.la = eye(1) * uvms.Aa.la;
% uvms.A.lr = DecreasingBellShapedFunction(1, 2.0, 0, 1 , uvms.v_altitude) * uvms.Aa.lr;
%% underactuation 
% select only the underactuated component (w_x) as explained in the notes
uvms.A.ua = diag([0 0 0 1 0 0 ]);


end 