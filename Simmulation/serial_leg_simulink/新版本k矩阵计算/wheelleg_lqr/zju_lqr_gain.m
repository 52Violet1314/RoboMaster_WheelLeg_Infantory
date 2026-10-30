function result = zju_lqr_gain(Qdiag,Rdiag,leg_length)
%ZJU_LQR_GAIN 用统一等效腿模型的平衡点线性模型求连续 LQR 增益 K。
%
%   result = zju_lqr_gain()
%   result = zju_lqr_gain(Qdiag,Rdiag)
%
% 代价函数：
% J = integral_0^inf(delta_X'*Q*delta_X + delta_U'*R*delta_U) dt
% delta_U = -K*delta_X，故实际力矩为 U=U*-K*(X-X*)。
%
% 状态/输入顺序与当前统一模型的十阶状态空间完全一致。
% 默认 Q 为当前实车调参值；传入两个对角向量即可调参。

if nargin < 1 || isempty(Qdiag)
    % [X, dX, yaw, dyaw, thetaL, dthetaL, thetaR, dthetaR, pitch, dpitch]
    % 当前调参：X=65，dX=4，dyaw=2，thetaL/R=35000，dthetaL/R=9，pitch=45000，dpitch=2。
    Qdiag = [65 4 400 2 35000 9 35000 9 45000 2];
end
if nargin < 2 || isempty(Rdiag)
    % 输入顺序：[Fw_L, Fw_R, Tp_L, Tp_R]。
    Rdiag = [10 10 1 1];
end
if nargin < 3 || isempty(leg_length)
    leg_length = 0.20;
end
Qdiag = Qdiag(:);
Rdiag = Rdiag(:);
assert(numel(Qdiag)==10 && all(isfinite(Qdiag)) && all(Qdiag>=0), ...
    'Qdiag 必须是 10 个非负有限数。');
assert(numel(Rdiag)==4 && all(isfinite(Rdiag)) && all(Rdiag>0), ...
    'Rdiag 必须是 4 个正有限数。');

p = zju_parameters_template(leg_length);
equilibrium = zju_find_static_equilibrium(p);
A = equilibrium.A;
B = equilibrium.B;
Q = diag(Qdiag);
R = diag(Rdiag);

% 连续代数 Riccati 方程：A''P+PA-PBR^-1B''P+Q=0。
% icare 的第二输出就是使 delta_U=-K*delta_X 的反馈增益。
[P,K,closedLoopPoles] = icare(A,B,Q,R,[],[],[]);
Acl = A-B*K;
careResidual = A'*P + P*A - P*B*(R\(B'*P)) + Q;

result = struct( ...
    'p',p, 'leg_length',leg_length, 'equilibrium',equilibrium, 'A',A, 'B',B, ...
    'Qdiag',Qdiag, 'Rdiag',Rdiag, 'Q',Q, 'R',R, ...
    'P',P, 'K',K, 'Acl',Acl, ...
    'open_loop_poles',eig(A), 'closed_loop_poles',closedLoopPoles, ...
    'care_residual_norm',norm(careResidual,'fro'), ...
    'control_law','U = U_star - K*(X-X_star)', ...
    'state_names',{equilibrium.state_names}, ...
    'input_names',{equilibrium.input_names});
end
