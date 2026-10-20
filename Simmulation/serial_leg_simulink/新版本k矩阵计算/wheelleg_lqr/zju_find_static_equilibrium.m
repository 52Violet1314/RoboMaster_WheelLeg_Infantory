function equilibrium = zju_find_static_equilibrium(p)
%ZJU_FIND_STATIC_EQUILIBRIUM 基于既有五阶非线性方程寻找静态平衡参考点。
% 本函数不改动 PDF 的动力学、消元或五阶非线性方程；只求解
%   Mred(q)*0 = Q_tau(U)+G(q)+C(q,0) = 0。
%
% 决策变量 z=[theta_l_l; theta_l_r; theta_b;
%              Twl_l; Twl_r; Tbl_l; Tbl_r]。
% 五个静态方程而有七个决策变量，因此平衡点不是唯一的。为使每次
% 换参数后的参考姿态可重复，另规定“车体竖直、两侧轮电机力矩相同”，
% 使七个未知量恰有七个方程。左右腿角和两侧髋关节力矩仍允许不同，
% 因而左右不对称的实车参数也可直接使用。
%
% 角度取正与旧 get_K_jiao_LQR.m 的控制接口保持一致：
% theta_l_l/theta_l_r 为二维腿模型的前摆正；phi 为车体左转正。
% 返回值 equilibrium 同时包含：
%   q,dq,U,X       平衡点的广义坐标、速度、输入和十阶状态；
%   residual       五条静态方程残差；
%   A,B            在 (X,U) 处由同一非线性方程数值线性化的局部模型：
%                    d(delta_X)/dt=A*delta_X+B*delta_U。

% 初始点与上下界只描述允许搜索的站立范围，不是动力学模型参数。
z0 = [0.10; 0.10; 0; 0; 0; 0; 0];
lb = [-0.70; -0.70; -0.50; -10; -10; -10; -10];
ub = [ 0.70;  0.70;  0.50;  10;  10;  10;  10];

% 六、七项是选点条件：theta_b=0、Twl_l=Twl_r。
% 它们不是新增动力学方程，不改变原五条静态方程。
residualForSearch = @(z) [staticResidual(z,p); z(3); z(4)-z(5)];
options = optimoptions('lsqnonlin', 'Display', 'off', ...
    'FunctionTolerance', 1e-14, 'OptimalityTolerance', 1e-12, ...
    'StepTolerance', 1e-13, 'MaxIterations', 300, 'MaxFunctionEvaluations', 5000);
[z,~,~,exitflag,output] = lsqnonlin(residualForSearch,z0,lb,ub,options);

residual = staticResidual(z,p);
residualNorm = norm(residual);
assert(exitflag > 0 && residualNorm < 1e-8, ...
    '未找到满足五条静态方程的平衡点：exitflag=%d，残差范数=%.3e。', ...
    exitflag,residualNorm);

q = [0; 0; z(1:3)];
dq = zeros(5,1);
U = z(4:7);
X = [0; 0; 0; 0; z(1); 0; z(2); 0; z(3); 0];
[A,B,xDot] = localLinearization(p,X,U);

equilibrium = struct( ...
    'q',q, 'dq',dq, 'ddq',zeros(5,1), 'U',U, 'X',X, ...
    'residual',residual, 'residual_norm',residualNorm, ...
    'exitflag',exitflag, 'solver_output',output, ...
    'selection_rule','Exact static equilibrium with theta_b=0 and Twl_l=Twl_r.', ...
    'linearization_method','Central numerical derivative of the unchanged nonlinear five-equation model at (X*,U*).', ...
    'A',A, 'B',B, 'x_dot_at_equilibrium',xDot, ...
    'open_loop_eigenvalues',eig(A), ...
    'state_names',{{'s','ds','phi','dphi','theta_l_l','dtheta_l_l','theta_l_r','dtheta_l_r','theta_b','dtheta_b'}}, ...
    'input_names',{{'Twl_l','Twl_r','Tbl_l','Tbl_r'}});
end

function ceq = staticResidual(z,p)
q = [0; 0; z(1:3)];
nonlinear = zju_nonlinear_eliminate(p,q,zeros(5,1),z(4:7));
ceq = nonlinear.residual(zeros(5,1));
end

function [A,B,xDot0] = localLinearization(p,X,U)
% 仅为“以找到的平衡点为原点”的局部模型计算数值导数。
% 这里对已有非线性模型求导，不改变它的动力学或消元关系。
n = 10;
m = 4;
hX = p.linearization_step;
hU = p.linearization_step;
xDot0 = stateDynamics(p,X,U);
A = zeros(n,n);
B = zeros(n,m);
for k = 1:n
    dx = zeros(n,1); dx(k) = hX;
    A(:,k) = (stateDynamics(p,X+dx,U)-stateDynamics(p,X-dx,U))/(2*hX);
end
for k = 1:m
    du = zeros(m,1); du(k) = hU;
    B(:,k) = (stateDynamics(p,X,U+du)-stateDynamics(p,X,U-du))/(2*hU);
end
end

function xDot = stateDynamics(p,X,U)
% 将十阶状态换回文档的五个广义角，再调用原有非线性消元方程。
[q,dq] = stateToGeneralizedCoordinates(p,X);
nonlinear = zju_nonlinear_eliminate(p,q,dq,U);
ddq = nonlinear.ddq;

tl = q(3); tr = q(4);
dtl = dq(3); dtr = dq(4);
sdd = p.R_w/2*(ddq(1)+ddq(2));
phidd = -p.R_w/(2*p.R_l)*(ddq(1)-ddq(2)) ...
    -p.l_l/(2*p.R_l)*cos(tl)*ddq(3) ...
    +p.l_r/(2*p.R_l)*cos(tr)*ddq(4) ...
    +p.l_l/(2*p.R_l)*sin(tl)*dtl^2 ...
    -p.l_r/(2*p.R_l)*sin(tr)*dtr^2;
xDot = [X(2); sdd; X(4); phidd; X(6); ddq(3); ...
        X(8); ddq(4); X(10); ddq(5)];
end

function [q,dq] = stateToGeneralizedCoordinates(p,X)
% 文档 Eq.(7)、(20) 的反变换。s、phi 是整体位置/航向自由度，
% 其零值不影响静态受力；该变换只用于把局部十阶状态传回五阶模型。
tl = X(5); tr = X(7);
dtl = X(6); dtr = X(8);
wheelSum = 2*X(1)/p.R_w;
wheelDiff = -2*p.R_l/p.R_w*(X(3) ...
    +p.l_l/(2*p.R_l)*sin(tl)-p.l_r/(2*p.R_l)*sin(tr));
wheelRateSum = 2*X(2)/p.R_w;
wheelRateDiff = -2*p.R_l/p.R_w*(X(4) ...
    +p.l_l/(2*p.R_l)*cos(tl)*dtl-p.l_r/(2*p.R_l)*cos(tr)*dtr);
q = [0.5*(wheelSum+wheelDiff); 0.5*(wheelSum-wheelDiff); tl; tr; X(9)];
dq = [0.5*(wheelRateSum+wheelRateDiff); ...
      0.5*(wheelRateSum-wheelRateDiff); dtl; dtr; X(10)];
end
