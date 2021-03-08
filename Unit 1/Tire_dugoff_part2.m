
close all;
clear all;
clc;


m = 2000;
g = 9.8;
mu = 0.7;
Fz = m*g/4;
C_s = 3e5;
C_alpha = 1.5e5;

%si=[0:0.1:1];

%alpha=0;


for si=0.1:0.1:0.5
for alpha=0:0.05:10
    
S = (mu*Fz*(1-si))/(2*(((C_s^2)*(si^2))+((C_alpha^2)*(tand(alpha))^2))^0.5);



if S>1
    fs = 1;
    
else
    fs = S*(2-S);
    
end   
%F_t = (C_s*si*fs)/(1-si);
F_s = (C_alpha*tand(alpha)*fs)/(1-si);
figure(1);
plot(alpha,F_s,'.');
hold on;

end
end
% F_s = (C_alpha*(tan(alpha))*fs)/(1-si);

%Longitudinal force vs Longitudinal slip



% xlabel('long. slip %');
% ylabel('F_x (N)');
% title('Longitudinal Tire Force vs Longitudinal Slip');
% grid on;
% hold on;