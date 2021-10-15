% velocity motion model based inverse kinematics
% (c) Mohammad Ali rizvi

clear;clc; close all;
T =0.01;
ITER = 1000;
L = 0.5; %robot width
r = 0.25; %wheel radius

t=linspace(-pi,pi,ITER); 
xt = 8*sin(t).^3; yt = 8*sin((2*t)).^3;
%plot(x,y)

N=0:ITER;
x = 8*(sin(N*T)).^3;
y = 8*(sin(2*N*T)).^3;
%figure;
plot(x,y, 'linewidth', 3)
title('Original Discrete Trajectory')
figure;

% initializing
phiN = zeros(1, ITER); w = zeros(1, ITER); v = zeros(1, ITER);
v_r = zeros(1, ITER); v_l = zeros(1, ITER); w_l = zeros(1, ITER); 
w_r = zeros(1, ITER);

for n=2:length(N)-1
    phiN(n) = atan2(y(n) - y(n-1), x(n) - x(n-1));
    
    meu = 0.5*( (sin(phiN(n))*(y(n+1)-y(n))+cos(phiN(n))*(x(n+1)-x(n))) ...
        / (cos(phiN(n))*(y(n+1)-y(n))-sin(phiN(n))*(x(n+1)-x(n))) );
    
    xm = 0.5*(x(n)+x(n+1));
    ym = 0.5*(y(n)+y(n+1));
    
    xs = xm - (meu/2)*(y(n+1)-y(n));
    ys = ym + (meu/2)*(x(n+1)-x(n));

    R = sqrt((x(n)-xs)^2+(y(n)-ys)^2);
    
    theta_1 = atan2(y(n)-ys,x(n)-xs);
    theta_2 = atan2(y(n+1)-ys,x(n+1)-xs);

    delta_phi_n = wrapToPi(theta_1 - theta_2);
    
    w(n) = delta_phi_n / T;
    
    v(n) = R*abs(w(n));
    
    v_r(n) = ( (R + 0.5*L).*w(n)); % right wheel linear velocity
    w_r(n) = v_r(n)/ r; % right wheel angular velocity

    v_l(n) = ( (R - 0.5*L).*w(n)); % left wheel linear velocity
    w_l(n) = v_l(n)/r; % left wheel angular velocity
    
end 

subplot 211
plot(v,'linewidth',2); legend('v_n'); title('linear velocity')
subplot 212
plot(w,'linewidth',2); legend('\omega_n'); title('anguar velocity')
figure;

% R_t = v./w; 
% 
% v_r = ( (R_t + 0.5*L).*w); % right wheel linear velocity
% w_r = v_r ./ r; % right wheel angular velocity
% 
% v_l = ( (R_t - 0.5*L).*w); % left wheel linear velocity
% w_l = v_l./r; % left wheel angular velocity

subplot 121
plot(w_r,'linewidth',2); legend('\omega_r'); title('w_r')
subplot 122
plot(w_l,'linewidth',2); legend('\omega_l'); title('w_l')
% subplot 223
% plot(v_r); title('v_r')
% subplot 224
% plot(v_l); title('v_l')

% %plotting initializations: 
% axis tight manual % this ensures that getframe() returns a consistent size
% filename = 'testAnimated.gif';
% h = figure; 

x_f = zeros(1, ITER); y_f = zeros(1, ITER); phi_f = zeros(1, ITER); 
for n = 1:ITER-1
%     x_f(n+1) = x_f(n) + (v(n)/w(n))*(-sin(phiN(n))+sin(phiN(n)+w(n)*T));
%     y_f(n+1) = y_f(n) + (v(n)/w(n))*(-cos(phiN(n))+cos(phiN(n)+w(n)*T));
    
    x_f(n+1) = x_f(n) + v(n)*T*cos(phiN(n));
    y_f(n+1) = y_f(n) + v(n)*T*sin(phiN(n));
    
    %phiN(n+1) = phiN(n)+T*w(n);
    
%     %create GIF
%     frame = getframe(h); 
%     im = frame2im(frame); 
%     [imind,cm] = rgb2ind(im,256); 
%     % Write to the GIF File 
%     if n == 2
%       imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
%     else 
%       imwrite(imind,cm,filename,'gif','WriteMode','append'); 
%     end 

end
figure;
hold on
plot(x_f,y_f,'linewidth',1.5)
plot(x,y,'linewidth',2)
legend('Recovered','True')
xlabel('x'); ylabel('y');
title('Recovered trajectory')

% figure()
% hold on 
%  
% plot(x_f, y_f, 'linewidth', 6); 
% plot(x,y,'linewidth',2);
% legend('Calculated Path', 'Original Path')
% hold off
% 
% print -deps figures/OutputFig
