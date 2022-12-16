clear all;
close all;
clc;
fclose all;
bdclose all;

% cost function
Q = [100,0;0,5];
R = 0.5;
del_t = 0.5;
w = 10;
A = [1,del_t;w*del_t,1];

B = [0;-w*del_t];
N = 1000;
K= struct;
k = struct;
P_next = Q;
p_next = -Q*[10;0];
horizon_len = floor(10/del_t);
for n = horizon_len:-1:1
qn = -Q*state_der(n);
Kn = -inv(R + B'*P_next*B)*B'*P_next*A;
Pn = Q + A'*P_next*A + A'*P_next*B*Kn;
kn = -inv(R + B'*P_next*B)*B'*p_next;
pn = qn + A'*p_next + A'*P_next*B*kn;
K(n).value = Kn;
k(n).value = kn;
P_next = Pn;
p_next = pn;
end
% 
% u = struct;
% z = struct;
% 
% z0 = [0;0];
% z_curr = z0;
% for i = 1:horizon_len
% u(i).value = K(i).value*(z_curr)+k(i).value;
% z_next = A*z_curr + B*u(i).value;
% z(i).Value = z_next;
% z_curr = z_next;
% end

WalkingManSimulation(K,k,del_t)


function z = state_der(timestep)
del_t = 0.5;

if(timestep<(10/del_t))
z = [timestep*del_t*0.4;0];
else
z = [4;0];
end
end




