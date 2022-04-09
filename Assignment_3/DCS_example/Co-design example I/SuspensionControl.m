
function CruiseControl_Design()

clc;
clear all;
format short;

A = [ 0       1       0    0;
     -8      -4       8    4;
      0       0       0    1;
     80      40     -160  -60];
  
B = [0; 80; 20;-1120];
C = [1 0 0 0];

sysc = ss(A,B,C,0);

T_period = 0.02;
delay = 0.0151;
sysd = c2d(sysc, T_period); phi = sysd.a; Gamma = sysd.b; C = sysd.c;

phi
% eye(2) + A*T_period
Gamma0 = inv(A)*(expm(A*(T_period-delay))-expm(A*0))*B
% (T_period - delay)*B
Gamma1 = inv(A)*(expm(A*T_period)-expm(A*(T_period-delay)))*B
% (delay)*B

phi_aug = [phi  Gamma1;zeros(1,5)]
Gamma_aug = [Gamma0; 1]
C_aug = [C 0]

gamma = [Gamma_aug phi_aug*Gamma_aug phi_aug^2*Gamma_aug  phi_aug^3*Gamma_aug phi_aug^4*Gamma_aug]
det(gamma)
if det(gamma) == 0
    disp('Uncontrollable')
else
    disp('Controllable');
end


H = (phi_aug - eye(5)*0.7)*(phi_aug + eye(5)*0.7)*(phi_aug - eye(5)*0.90)*(phi_aug - eye(5)*0.92)*(phi_aug - eye(5)*0.92)
K = - [0 0 0 0 1]*inv(gamma)*H
Acl = (phi_aug + Gamma_aug*K)
F = 1/(C_aug*inv(eye(5)-Acl)*Gamma_aug)
abs(eigs(Acl))

x1(2) = 0; x1(1) = 0;
x2(2) = 2; x2(1) = 2.0;
x3(2) = 1.0; x3(1) = 1.0;
x4(2) = 2.0; x4(1) = 2.0;
input(2) = 0; input(1) = 0;
time(2) = T_period; time(1) = 0;
i = 2;
r = 3.0;
for i=2:4/T_period
    
    u = K*[x1(i);x2(i);x3(i);;x4(i);input(i-1)] + r*F;
   
    xkp1 = phi*[x1(i);x2(i);x3(i);x4(i)]+ Gamma1*input(i-1) + Gamma0*u;
    x1(i+1) = xkp1(1);
    x2(i+1) = xkp1(2);
    x3(i+1) = xkp1(3);
    x4(i+1) = xkp1(4);
    input(i) = u;    
    time(i+1) = time(i) + T_period;   
end

plot(time, x1, 'r');
% hold;
% figure
% plot(time(1:length(input)), input, 'r');
max(abs(input))
