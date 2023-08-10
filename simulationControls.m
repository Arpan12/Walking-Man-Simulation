clear all;
close all;
clc;
fclose all;
bdclose all;
desired_state=0;
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
horizon_len = floor(12/del_t);
der_state_array =zeros(horizon_len,2);

for i = 1:horizon_len
del_t = 0.5;
%global desired_state;
r = randi([4 6],1);
v = r/10;
desired_state = desired_state+v*del_t;
der_state_array(i,:) = [desired_state;0]; 

end


for n = horizon_len:-1:1


qn = -Q*der_state_array(n,:)';%state_der(desired_state,n);
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



% function z = state_der(desired_state,timestep)
% del_t = 0.5;
% %global desired_state;
% r = randi([4 6],1);
% v = r/10;
% desired_state = desired_state+v*del_t;
% if(timestep<(10/del_t))
% %z = [timestep*del_t*v;0];
% %desired_state = desired_state+v*del_t;
% z = [desired_state;0];
% else
% z = [desired_state;0];
% end
% end




