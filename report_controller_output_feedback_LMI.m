clear;
close;
cvx_clear;
run report_model_equilibrium.m
% run report_model_negative_theta.m
C2 = [1 0 0 0;
    0 0 1 0;
    0 0 0 1];
p_y = size(C2,1);
D21 = zeros(p_y,1);
D22 = zeros(p_y,1);

cvx_begin sdp
    variable W(n,n) symmetric
    variable V(p_y,n)
    
    subject to
        W >= 1e-6*eye(n);
        A'*W + W*A +C2'*V + V'*C2 + 4*W <= -1e-6*eye(n);
cvx_end
L_gain = W\V';

cvx_begin sdp
    variable P(n,n) symmetric
    variable Z(m,n)
    
    subject to
        P >= 1e-6*eye(n);
        A*P + P*A' + B2*Z + Z'*B2' <= -1e-6*eye(n);
cvx_end
K_gain = Z/P;

AK = A+L_gain*C2+B2*K_gain;
BK = -L_gain;
CK = K_gain;
DK = zeros(m,p_y);

Q = inv(eye(m)-DK*D22);
Assistant_matrix = [eye(m) -DK;-D22 eye(p_y)]\[zeros(m,n) CK;C2 zeros(p_y,n)];

A_cl = blkdiag(A,AK) +blkdiag(B2,BK)*Assistant_matrix;
B_cl = [B1+B2*DK*Q*D21; BK*Q*D21];
C_cl = [C1 zeros(p_z,n)] + [D12 zeros(p_z,p_y)]*Assistant_matrix;
D_cl = D11+D12*DK*Q*D21;

sys_cl= ss(A_cl,B_cl,C_cl,D_cl);

x0 = [0.1 ; -0.1; 0; 0.1; 0; 0; 0; 0];
t = 0:0.01:30;
w = [zeros(1,length(t))];
w = randn(1, length(t));
w= sin(t);
[y, t, x] = lsim(sys_cl, w, t,x0);

figure('Position', [100, 100, 600, 600]); 
sgtitle('Output Feedback Control through Lyapunov Equation')
state_variables = ["\alpha","\alpha_{dot}","\beta","\beta_{dot}"];
for i=1:4
    subplot(4,1,i);
    plot(t, x(:,i), 'b', t, x(:,i+n), 'r--','LineWidth',1);
    legend( sprintf('%s', state_variables(i)), sprintf('%s^{hat}', state_variables(i)),'FontSize',10);
    % title(sprintf('Trajectories of real & estimated state variable %s',state_variables(i)),'FontSize',12);
    grid on;
end
xlabel('Time (seconds)')
