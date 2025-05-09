clear;
close;
cvx_clear;
run report_model_equilibrium.m

C2 = [1 0 0 0;
    0 0 1 0;
    0 0 0 1];
p_y = size(C2,1);
D21 = zeros(p_y,1);
D22 = zeros(p_y,1);

cvx_begin sdp
    variable gamm
    variable Y1(n,n) symmetric
    variable X1(n,n) symmetric
    variable An(n,n)
    variable Bn(n,p_y)
    variable Cn(m,n)
    variable Dn(m,p_y)

    minimize(gamm)

    M11 = A*Y1+Y1*A'+B2*Cn+Cn'*B2';
    M21 = A'+An+(B2*Dn*C2)';
    M22 = X1*A+A'*X1+Bn*C2+C2'*Bn';
    M31 = (B1+B2*Dn*D21)';
    M32 = (X1*B1+Bn*D21)';
    M41 = C1*Y1+D12*Cn;
    M42 = C1+D12*Dn*C2;
    M43 = D11+D12*Dn*D21;

    gamm>=2.2*1e-4;
    [M11 M21' M31' M41';
     M21 M22 M32' M42';
     M31 M32 -gamm*eye(m) M43';
     M41 M42 M43 -gamm*eye(p_z)] <= -1e-6*eye(n+n+m+p_z);
    
    [Y1 eye(n);
     eye(n) X1] >= 1e-6*eye(n+n);

cvx_end
disp(gamm)

Y2 = eye(n);
X2 = eye(n)-X1*Y1;

M_K2 = [X2 X1*B2; zeros(m,n) eye(m)]\([An-X1*A*Y1 Bn;Cn Dn])/[Y2' zeros(n,p_y);C2*Y1 eye(p_y)];

AK2 = M_K2(1:n,1:n);
BK2 = M_K2(1:n,n+1:end);
CK2 = M_K2(n+1:end,1:n);
DK2 = M_K2(n+1:end,n+1:end);

DK = (eye(m)+DK2*D22)\DK2;
BK = BK2*(eye(p_y)-D22*DK);
CK = (eye(m)-DK*D22)*CK2;
AK = AK2 - BK/(eye(p_y)-D22*DK)*D22*CK;

Q = inv(eye(m)-DK*D22);
Assistant_matrix = [eye(m) -DK;-D22 eye(p_y)]\[zeros(m,n) CK;C2 zeros(p_y,n)];

A_cl = blkdiag(A,AK) +blkdiag(B2,BK)*Assistant_matrix;
B_cl = [B1+B2*DK*Q*D21; BK*Q*D21];
C_cl = [C1 zeros(p_z,n)] + [D12 zeros(p_z,p_y)]*Assistant_matrix;
D_cl = D11+D12*DK*Q*D21;

sys_cl= ss(A_cl,B_cl,C_cl,D_cl);

x0 = [0.1 ; -0.1; 0; 0.1; 0; 0; 0; 0];
t = 0:0.01:80;
w = [zeros(1,length(t))];
w = sin(t);
[y, t, x] = lsim(sys_cl, w, t,x0);

figure('Position', [100, 100, 600, 600]); 
sgtitle('Output Feedback Control through H_{\infty} Optimal Control')
state_variables = ["\alpha","\alpha_{dot}","\beta","\beta_{dot}"];
for i=1:4
    subplot(4,1,i);
    plot(t, x(:,i), 'b', t, x(:,i+n), 'r--','LineWidth',1);
    legend( sprintf('%s', state_variables(i)), sprintf('%s^{hat}', state_variables(i)),'FontSize',10);
    % title(sprintf('Trajectories of real & estimated state variable %s',state_variables(i)),'FontSize',12);
    grid on;
    ylim([min(x(:,i)),max(x(:,i))])
end
xlabel('Time (seconds)')

figure('Position', [100, 100, 600, 600]); 
sgtitle('Output Feedback Control through H_{\infty} Optimal Control')
state_variables = ["\alpha","\alpha_{dot}","\beta","\beta_{dot}"];
for i=1:4
    subplot(4,1,i);
    plot(t, x(:,i), 'b', t, x(:,i+n), 'r--','LineWidth',1);
    legend( sprintf('%s', state_variables(i)), sprintf('%s^{hat}', state_variables(i)),'FontSize',10);
    % title(sprintf('Trajectories of real & estimated state variable %s',state_variables(i)),'FontSize',12);
    grid on;
    ylim([-1,1])
end
xlabel('Time (seconds)')

