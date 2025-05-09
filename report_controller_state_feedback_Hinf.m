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
    variable Y(n,n) symmetric
    variable Z(m,n)
    variable gamm 

    minimize(gamm)

    subject to
        Y >= 1e-6*eye(n);
        [Y*A'+A*Y+Z'*B2'+B2*Z B1 Y*C1'+Z'*D12'; 
            B1' -gamm*eye(m) D11'; 
            C1*Y+D12*Z D11 -gamm*eye(p_z)] <= -1e-6*eye(n+m+p_z);
cvx_end
K_gain=Z/Y;
disp(gamm)

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

x0 = [0.1 ; -0.1; 0; 0.1; 0; 0; 0; 0];
t = 0:0.01:20;
w = [zeros(1,length(t))];
w = sin(t);
[y, t, x] = lsim(sys_cl, w, t,x0);

figure('Position', [100, 100, 600, 600]); 
sgtitle('State Feedback Control through H_{\infty} Optimal Control')
state_variables = ["\alpha","\alpha_{dot}","\beta","\beta_{dot}"];
for i=1:4
    subplot(4,1,i);
    plot(t, x(:,i), 'b','LineWidth',1);
    legend( sprintf('%s', state_variables(i)),'FontSize',10);
    % title(sprintf('Trajectories of state variable %s',state_variables(i)),'FontSize',12);
    grid on;
end
xlabel('Time (seconds)')

