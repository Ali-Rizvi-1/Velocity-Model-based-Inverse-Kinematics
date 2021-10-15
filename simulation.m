% velocity motion model based inverse kinematics
% (c) 

clear all;clc;
T =0.1;
ITER = 1e3;
L = 0.5; %robot width
r = 0.25; %wheel radius

t=linspace(-pi,pi,ITER); 
xt = 8*sin(t).^3; yt = 8*sin((2*t)).^3;
%plot(x,y)

N=0:1:ITER;
x = 8*(sin(N*T)).^3;
y = 8*(sin(2*N*T)).^3;
%figure;
plot(x,y)
figure;

% initializing
phiN = zeros(1, ITER); w = zeros(1, ITER); v = zeros(1, ITER);

for n=2:length(N)-1
    phiN(n) = atan2(y(n) - y(n-1), x(n) - x(n-1));
    
    meu = 0.5*( (sin(phiN(n))*(y(n+1)-y(n))+cos(phiN(n))*(x(n+1)-x(n))) ...
        / (cos(phiN(n))*(y(n+1)-y(n))-sin(phiN(n))*(x(n+1)-x(n))) );
    
    xm = 0.5*(x(n)+x(n+1));
    ym = 0.5*(y(n)+y(n+1));
    
    xs = xm + meu*(y(n)-y(n+1));
    ys = ym + meu*(x(n+1)-x(n));

    R = sqrt((x(n)-xs)^2+(y(n)-ys)^2);
    
    theta_1 = atan2(y(n)-ys,x(n)-xs);
    theta_2 = atan2(y(n+1)-ys,x(n+1)-xs);

    delta_phi_n = wrapToPi(theta_1 - theta_2);
    
    w(n) = delta_phi_n / T;
    
    v(n) = R*w(n);
    
end 
R_t = v./w; 

v_r = ( (R_t + 0.5*L).*w); % right wheel linear velocity
w_r = v_r ./ r; % right wheel angular velocity

v_l = ( (R_t - 0.5*L).*w); % left wheel linear velocity
w_l = v_l./r; % left wheel angular velocity

subplot 221
plot(w_r); title('w_r')
subplot 222
plot(w_l); title('w_l')
subplot 223
plot(v_r); title('v_r')
subplot 224
plot(v_l); title('v_l')


x_f = zeros(1, ITER); y_f = zeros(1, ITER); phi_f = zeros(1, ITER); 
for n = 2:ITER-1
    x_f(n+1) = x_f(n) + (v(n)/w(n))*(-sin(phiN(n))+sin(phiN(n)+w(n)*T));
    y_f(n+1) = y_f(n) + (v(n)/w(n))*(-cos(phiN(n))+cos(phiN(n)+w(n)*T));
end
figure;
plot(x_f,y_f)