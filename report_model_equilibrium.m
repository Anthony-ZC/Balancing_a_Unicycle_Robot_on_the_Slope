clear;
g = 9.8;
M_p = 15.63;
M_w = 31.28;
I_p = 3.55;
I_w = 1.40;
R_w = 0.31;
L = 0.155;
n = 19.56;
k_T = 0.05;
R = 1.2;
k_e = 0.0534;

theta = 0.1;
equilibrium = asin((M_w+M_p)*R_w/(M_p*L)*sin(theta))-theta;

P1 = M_w*R_w^2 + M_p*R_w^2 + I_w;
P2 = M_p*R_w*L*cos(equilibrium+theta);
P3 = M_p*L^2 + I_p;
P4 = M_p*L*g*cos(equilibrium+theta);
P5 = (n*k_T)/R;
P6 = (M_w + M_p)*R_w;

A = [0 1 0 0;
    (P1*P4)/(P1*P3-P2^2) 0 0 ((P1+P2)*P5*n*k_e)/(P1*P3-P2^2);
    0 0 0 1;
    -(P2*P4)/(P1*P3-P2^2) 0 0 -((P2+P3)*P5*n*k_e)/(P1*P3-P2^2)];
eig(A)
B2 = [0;
    -(P1+P2)*P5/(P1*P3-P2^2);
    0;
    (P2+P3)*P5/(P1*P3-P2^2)];

B1=B2;

C1 = [1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];

n = size(A,1);
m = size(B2,2);
p_z = size(C1,1);

D11 = zeros(p_z,1);
D12 = zeros(p_z,1);

% sys_ol= ss(A,B2,C1,0);
% 
% x0 = [0.1 ; -0.1; 0; 0.1];
% t = 0:0.01:30;
% w = [zeros(1,length(t))];
% [y, t, x] = lsim(sys_ol, w, t,x0);
% 
% figure('Position', [100, 100, 600, 600]); 
% sgtitle('Open-loop Simulation')
% state_variables = ["\alpha","\alpha_{dot}","\beta","\beta_{dot}"];
% for i=1:4
%     subplot(4,1,i);
%     plot(t, x(:,i), 'b','LineWidth',1);
%     legend( sprintf('%s', state_variables(i)),'FontSize',10);
%     % title(sprintf('Trajectories of state variable %s',state_variables(i)),'FontSize',12);
%     grid on;
% end
% xlabel('Time (seconds)')