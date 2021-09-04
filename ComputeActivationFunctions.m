function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here
% ATT !!! this A matrix consider the activation function of the single task
% (first term of the multiplication) and the activation func. due to the
% multiple actions switching that you can see in "UpdateMissionPhase" (second term)

% arm tool position control
%uvms.A.t = eye(6);   % da sostituire in altri es

 uvms.Aact.minTre = 1;
 uvms.Aact.maxTre = 1.3;
%% Without Actions 
%  uvms.A.vpos = eye(3); 
%  uvms.A.vatt = eye(3);
%  uvms.A.t = eye(6);
%  uvms.A.act = DecreasingBellShapedFunction( uvms.Aact.minTre, uvms.Aact.maxTre, 0, 1 , uvms.v_altitude);

%  uvms.A.ha = IncreasingBellShapedFunction(0.1, 0.2, 0, 1 , norm(uvms.v_rho)); % see the notes to see this function
%  uvms.A.la = eye(1);
% uvms.A.lr = eye(1);
% uvms.A.vc = eye(6);
% uvms.A.jl = eye(7);
% for j= 1:7
% uvms.A.jl_min(j,j) = DecreasingBellShapedFunction(uvms.jlmin(j) + 0.3, uvms.jlmin(j), 0, 1, uvms.q(j)) ;
% uvms.A.jl_max(j,j) = IncreasingBellShapedFunction(uvms.jlmax(j) - 0.3, uvms.jlmax(j), 0, 1, uvms.q(j)) ;
% end
% uvms.A.ps = eye(4);

uvms.A.mu = DecreasingBellShapedFunction(0.02, 0.05, 0, 1, uvms.mu);
%% With Multipe actions implementation

uvms.A.vpos = eye(3) * uvms.Aa.vpos;
uvms.A.vatt = eye(3) * uvms.Aa.vatt;
uvms.A.ha = IncreasingBellShapedFunction(0.1, 0.2, 0, 1 , norm(uvms.v_rho)) * uvms.Aa.ha;
uvms.A.act = DecreasingBellShapedFunction(1, 1.5, 0, 1 , uvms.v_altitude) * uvms.Aa.act;
uvms.A.la = eye(1) * uvms.Aa.la;
% uvms.A.lr = DecreasingBellShapedFunction(0, 10, 0, 1 , uvms.v_altitude) * uvms.Aa.lr;
% solo altitudine

% combinazione mis, alt
% uvms.A.lr = DecreasingBellShapedFunction(1, 3, 0, 0.5 , uvms.v_altitude) + uvms.Aa.lr;

%senza nulla
uvms.A.lr = 1 * uvms.Aa.lr;
uvms.A.vc = eye(6) * uvms.Aa.vc ;
uvms.A.t = eye(6) * uvms.Aa.t;
for j= 1:7
uvms.A.jl_min(j,j) = DecreasingBellShapedFunction(uvms.jlmin(j) - 0.3, uvms.jlmin(j), 0, 1, uvms.q(j)) ;
uvms.A.jl_max(j,j) = IncreasingBellShapedFunction(uvms.jlmax(j) + 0.3, uvms.jlmax(j), 0, 1, uvms.q(j)) ;
end
uvms.A.ps = eye(4) * uvms.Aa.ps;
%% underactuation 
% select only the underactuated component (w_x) as explained in the notes
uvms.A.ua = diag([0 0 0 1 0 0 ]) * uvms.Aa.ua;


end 