function xd = stableDS_3D(x)
% STABLEDS_3D - Stable 3D dynamical system
% A = - I ; 
% Define coupling strengths
a =[1,-1];
b =[1,-1];
c = [-1,-1] ;

%% 
xd = -x;
%xd(1,:) =  a(1) * x(3,:) + a(2)* x(1,:);
%xd(2,:) =  b(1) * x(1,:) + b(2)* x(2,:);
%xd(3,:) = c(1) * x(2,:)+ c(2) * x(3,:);

end

