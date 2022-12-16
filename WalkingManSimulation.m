function WalkingManSimulation(K,k,control_Interval)
% x_init = 0;
% y_init = l;
% 
% x_velinit = 0;
% y_velinit = 0;
theta=0;
%assume(theta,'real')
omega = 0;
horizon_len = 10;
del_t = 0.001;
w = 10;
thetaSolPlot = zeros(horizon_len*100,1);
axis(gca,'equal');
axis([-2 10 -2 2]);
grid on;
xcom_prev=0;
x_com_vel=0;
ycom_prev =1;
y_com_vel=0;
cop_idx=1;
xcop_prev=0;
xcop=0;
% xcop=K(cop_idx).value*[xcom_prev;x_com_vel]+k(cop_idx).value;
for h = 1:horizon_len*1000
    
    if(mod(h,floor(control_Interval/del_t))==1)
        xcop_prev = xcop;
        xcop = K(cop_idx).value*[xcom_prev;x_com_vel]+k(cop_idx).value;
        cop_idx = cop_idx+1;
         
    end
    %cop_idx = h;
    %xcop=0;
    theta = asin((-xcop+xcom_prev));
    thetaSolPlot(h) = theta;
    %theta = atan((xcom_prev)/(ycom_prev));
    ang_acc = w*sin(theta);
    
    x_com_acc = ang_acc*cos(theta);
    x_com_vel = x_com_vel + x_com_acc*del_t;
    x_com_curr = xcom_prev+x_com_vel*del_t+0.5*x_com_acc*(del_t^2);

%     y_com_acc = -ang_acc*sin(theta);
%     y_com_vel = y_com_vel + y_com_acc*del_t;
%     y_com_curr = ycom_prev+y_com_vel*del_t+0.5*y_com_acc*(del_t^2);
%     
    y_com_curr = ycom_prev;
    %COM
    %P = line([sin(theta) sin(theta)], [cos(theta) cos(theta)]);
    %O_circ = viscircles([0 0],0.01);
    leg1 = line([xcop x_com_curr],[0 y_com_curr]);
    leg2 = line([xcop_prev x_com_curr],[0 y_com_curr]);
   % leg = line([0 x_com_curr],[0 y_com_curr]);
    xcom_prev=x_com_curr;
    ycom_prev = y_com_curr;

    pause(del_t);

    
        %delete(P);
        %delete(o_circ);
        delete(leg1);
        delete(leg2);
   end
%end




%fanimator(@fplot,x_pos,y_pos,'ko','MarkerFaceColor','k','AnimationRange',[0 10]);
% hold on;
% assignin('base',"x_pos",x_pos)
% assignin('base',"y_pos",y_pos)
% fanimator(@plotPendulum,'AnimationRange',[0.01 10],'FrameRate',100);
% %fanimator(@(t) ,'AnimationRange',[0.01 10],'FrameRate',100);
% fanimator(@(t) text(-0.3,0.3,"Timer: "+num2str(t*100,2)+" s"),'AnimationRange',[0.01 10],'FrameRate',100);
% hold off

end
% function c=plotPendulum(t)
% %x = sym('x');
% x_pos = evalin('base','x_pos');
% y_pos = evalin('base','y_pos');
% tSize = size(x_pos,1);
% %t = linspace(1,tSize,1)
% p  = eval('t*100')
% x_pos1 = @(p) x_pos(t);
% y_pos1 = @(p) y_pos(t);
% c = fplot(x_pos1,y_pos1)
% %p  = eval(t*100)
% %plot(x_pos(p),y_pos(p),'k-')
% 
% end
% % 