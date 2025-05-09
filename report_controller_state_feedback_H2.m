clear;
close;
cvx_clear;
run report_model_equilibrium.m

C2 = [1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];
p_y = size(C2,1);
D21 = zeros(p_y,1);
D22 = zeros(p_y,1);

rank(ctrb(A,B2))
rank(obsv(A,C2))

cvx_begin sdp
    variable P(n,n) symmetric
    variable Z(m,n)
    variable G(p_z,p_z) symmetric

    minimize(trace(G))
    subject to
        P >= 1e-6*eye(n);
        A*P + B2*Z + P*A' +Z'*B2' + B1*B1' + P <= -1e-6*eye(n);
        [G (C1*P+D12*Z)';C1*P+D12*Z  P] >= 1e-6*eye(p_z+n);
cvx_end
K_gain = Z/P;
trace(G)
sqrt(trace(G))

% e = eig(A+B2*K_gain)'
% e(1)
% e(2)
% e(3)
% e(4)
AK = zeros(n,n);
BK = zeros(n,p_y);
CK = zeros(m,n);
DK = K_gain;

Q = inv(eye(m)-DK*D22);
Assistant_matrix = [eye(m) -DK;-D22 eye(p_y)]\[zeros(m,n) CK;C2 zeros(p_y,n)];

A_cl = blkdiag(A,AK) +blkdiag(B2,BK)*Assistant_matrix;
B_cl = [B1+B2*DK*Q*D21; BK*Q*D21];
C_cl = [C1 zeros(p_z,n)] + [D12 zeros(p_z,p_y)]*Assistant_matrix;
D_cl = D11+D12*DK*Q*D21;

sys_cl= ss(A_cl,B_cl,C_cl,D_cl);
% sigma(sys_cl)

x0 = [0.1 ; -0.1; 0; 0.1; 0; 0; 0; 0];
t = 0:0.01:30;
w = [zeros(1,length(t))];
w = randn(1, length(t));
[y, t, x] = lsim(sys_cl, w, t,x0);

figure('Position', [100, 100, 600, 600]); 
sgtitle('State Feedback Control through H_2 Optimal Control')
state_variables = ["\alpha","\alpha_{dot}","\beta","\beta_{dot}"];
for i=1:4
    subplot(4,1,i);
    plot(t, x(:,i), 'b','LineWidth',1);
    legend( sprintf('%s', state_variables(i)),'FontSize',10);
    % title(sprintf('Trajectories of state variable %s',state_variables(i)),'FontSize',12);
    grid on;
end
xlabel('Time (seconds)')
