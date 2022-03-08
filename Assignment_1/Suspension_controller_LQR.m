
function Pipelined_control()

clc;
clear all;
format short;

%% suspension system model
m1 = 2500;
m2 = 320;
k1 = 80000;
k2 = 500000;
b1 = 350;
b2 = 15020;

A=[0                 1   0                                              0
  -(b1*b2)/(m1*m2)   0   ((b1/m1)*((b1/m1)+(b1/m2)+(b2/m2)))-(k1/m1)   -(b1/m1)
   b2/m2             0  -((b1/m1)+(b1/m2)+(b2/m2))                      1
   k2/m2             0  -((k1/m1)+(k1/m2)+(k2/m2))                      0];

B=[0;
   1/m1;
   0;
   (1/m1)+(1/m2)];

C=[0 0 1 0];

sysc = ss(A,B,C,0);

h = 0.1;
tau = h*0.9;

%% discretization 
sysd = c2d(sysc, h); phi = sysd.a; Gamma = sysd.b; C = sysd.c;
Gamma0 = inv(A)*(expm(A*(h-tau))-expm(A*0))*B;
Gamma1 = inv(A)*(expm(A*h)-expm(A*(h-tau)))*B;


phi_aug = [phi  Gamma1;zeros(1,5)];
Gamma_aug = [Gamma0; 1];
C_aug = [C 0];

%% Desired closed-loop poles and pole placement
% alpha = [0.95 0.98 0.98 0.98 0.9];
% K = -acker(phi_aug,Gamma_aug,alpha)

%% LQR design
[X,L,G] = dare(phi_aug, Gamma_aug, [1 0 0 0 0; 0 1 0 0 0; 0 0 10*10^13 0 0; 0 0 0 1 0;0 0 0 0 1], 1);
K = -G;

%% feedforward gain
F = 1/(C_aug*inv(eye(5)-phi_aug-Gamma_aug*K)*Gamma_aug);

 
x0 = [0;0;0;0;0];
time(1) = 0;
for i=1:1500
   if i==1
       time(1) = 0;
   else 
       time(i) = time(i-1) + h;
       
   end
 
    if i>500 & i<1000
        Road(i) = -0.02;
    else 
        Road(i) = 0;
    end 
%     Road(i) = 0.002*sin(0.005*i);
    y(i) = C_aug*x0;
    u(i) = K*x0 + F*Road(i);
    x_1 = phi_aug*x0 + Gamma_aug*u(i);
    x0 = x_1;
   
end
max(u)
plot(time, y,'b', time, Road,'r')


