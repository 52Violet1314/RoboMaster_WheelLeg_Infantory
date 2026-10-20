function out = zju_symbolic_eliminate()
%ZJU_SYMBOLIC_ELIMINATE 由文档 Eq.(21)-(31) 符号消元，不代入实车数值。
% 输出：Mred*ddq = Btau*U + G + C。

syms theta_w_l theta_w_r theta_l_l theta_l_r theta_b real
syms dtheta_w_l dtheta_w_r dtheta_l_l dtheta_l_r dtheta_b real
syms ddtheta_w_l ddtheta_w_r ddtheta_l_l ddtheta_l_r ddtheta_b real
syms T_w_l T_w_r T_b_l T_b_r real
syms f_l f_r F_ws_l F_ws_r F_wh_l F_wh_r F_bs_l F_bs_r F_bh_l F_bh_r real
syms R_w R_l l_l l_r l_w_l l_w_r l_b_l l_b_r l_c phi_c real
syms m_w m_l m_r m_b I_w I_l I_r I_b I_zz g real

q = [theta_w_l;theta_w_r;theta_l_l;theta_l_r;theta_b];
dq = [dtheta_w_l;dtheta_w_r;dtheta_l_l;dtheta_l_r;dtheta_b];
ddq = [ddtheta_w_l;ddtheta_w_r;ddtheta_l_l;ddtheta_l_r;ddtheta_b];
U = [T_w_l;T_w_r;T_b_l;T_b_r];
zint = [f_l;f_r;F_ws_l;F_ws_r;F_wh_l;F_wh_r;F_bs_l;F_bs_r;F_bh_l;F_bh_r];
z = [ddq;zint];

% 文档 Eq.(8)-(9)、(12)-(13) 的几何辅助角。
alpha_l = theta_l_l-acos((l_w_l^2+l_l^2-l_b_l^2)/(2*l_w_l*l_l));
alpha_r = theta_l_r-acos((l_w_r^2+l_r^2-l_b_r^2)/(2*l_w_r*l_r));
beta_l  = theta_l_l+acos((l_b_l^2+l_l^2-l_w_l^2)/(2*l_b_l*l_l));
beta_r  = theta_l_r+acos((l_b_r^2+l_r^2-l_w_r^2)/(2*l_b_r*l_r));
beta_c  = theta_b+phi_c;

% 文档 Eq.(7)、(10)-(11)、(14)-(18) 的二阶导数。
phidd = -R_w/(2*R_l)*(ddtheta_w_l-ddtheta_w_r) ...
    -l_l/(2*R_l)*cos(theta_l_l)*ddtheta_l_l+l_r/(2*R_l)*cos(theta_l_r)*ddtheta_l_r ...
    +l_l/(2*R_l)*sin(theta_l_l)*dtheta_l_l^2-l_r/(2*R_l)*sin(theta_l_r)*dtheta_l_r^2;
Sbdd = R_w/2*(ddtheta_w_l+ddtheta_w_r) ...
    +l_l/2*cos(theta_l_l)*ddtheta_l_l+l_r/2*cos(theta_l_r)*ddtheta_l_r+l_c*cos(beta_c)*ddtheta_b ...
    -l_l/2*sin(theta_l_l)*dtheta_l_l^2-l_r/2*sin(theta_l_r)*dtheta_l_r^2-l_c*sin(beta_c)*dtheta_b^2;
hbdd = -l_l/2*(sin(theta_l_l)*ddtheta_l_l+cos(theta_l_l)*dtheta_l_l^2) ...
    -l_r/2*(sin(theta_l_r)*ddtheta_l_r+cos(theta_l_r)*dtheta_l_r^2) ...
    -l_c*(sin(beta_c)*ddtheta_b+cos(beta_c)*dtheta_b^2);
Slldd = R_w*ddtheta_w_l+l_w_l*(cos(alpha_l)*ddtheta_l_l-sin(alpha_l)*dtheta_l_l^2);
Slrdd = R_w*ddtheta_w_r+l_w_r*(cos(alpha_r)*ddtheta_l_r-sin(alpha_r)*dtheta_l_r^2);
hlldd = hbdd+l_b_l*(sin(beta_l)*ddtheta_l_l+cos(beta_l)*dtheta_l_l^2) ...
    +l_c*(sin(beta_c)*ddtheta_b+cos(beta_c)*dtheta_b^2);
hlrdd = hbdd+l_b_r*(sin(beta_r)*ddtheta_l_r+cos(beta_r)*dtheta_l_r^2) ...
    +l_c*(sin(beta_c)*ddtheta_b+cos(beta_c)*dtheta_b^2);

% 文档 Eq.(21)-(31)：将等式统一写为 e=0。
e1 = m_w*R_w*ddtheta_w_l-f_l+F_ws_l;
e2 = I_w*ddtheta_w_l-T_w_l+f_l*R_w;
e3 = m_w*R_w*ddtheta_w_r-f_r+F_ws_r;
e4 = I_w*ddtheta_w_r-T_w_r+f_r*R_w;
e5 = m_l*Slldd-F_ws_l+F_bs_l;
e6 = m_l*hlldd-F_wh_l+F_bh_l+m_l*g;
e7 = I_l*ddtheta_l_l-F_wh_l*l_w_l*sin(alpha_l)+F_ws_l*l_w_l*cos(alpha_l) ...
    - F_bh_l*l_b_l*sin(beta_l)+F_bs_l*l_b_l*cos(beta_l)+T_w_l-T_b_l;
e8 = m_r*Slrdd-F_ws_r+F_bs_r;
e9 = m_r*hlrdd-F_wh_r+F_bh_r+m_r*g;
e10 = I_r*ddtheta_l_r-F_wh_r*l_w_r*sin(alpha_r)+F_ws_r*l_w_r*cos(alpha_r) ...
    - F_bh_r*l_b_r*sin(beta_r)+F_bs_r*l_b_r*cos(beta_r)+T_w_r-T_b_r;
e11 = m_b*Sbdd-F_bs_l-F_bs_r;
e12 = m_b*hbdd-F_bh_l-F_bh_r+m_b*g;
e13 = I_b*ddtheta_b+T_b_l+T_b_r+(F_bs_l+F_bs_r)*l_c*cos(beta_c) ...
    -(F_bh_l+F_bh_r)*l_c*sin(beta_c);
e14 = I_zz*phidd+(f_l-f_r)*R_l;
e15 = F_wh_l-F_wh_r;
e = [e1;e2;e3;e4;e5;e6;e7;e8;e9;e10;e11;e12;e13;e14;e15];

Mraw = jacobian(e,z);
craw = subs(e,z,zeros(size(z)));
Mdd = Mraw(:,1:5);
Mint = Mraw(:,6:end);
% 严格按文档的“轮、腿、机体方程联立解十个内力，再代回”处理。
% 用 Eq.(21)-(24)、Eq.(28)、Eq.(31) 解十个内部力；代回后保留的五行依次是
% 左腿转动 Eq.(25)、右腿转动 Eq.(26)、机体水平 Eq.(27)、机体俯仰 Eq.(29)、偏航 Eq.(30)。
% 此固定物理消元顺序在左右完全对称的直立零位也非奇异，不依赖任意 null() 基底。
solveRows = [1 2 3 4 5 6 8 9 12 15];
keepRows = [7 10 11 13 14];
MintSolve = Mint(solveRows,:);
assert(isequal(size(MintSolve),[10 10]),'内部力消元矩阵尺寸异常。');
N = [-MintSolve.'\Mint(keepRows,:).'; eye(5)];
Mred = simplify(Mdd(keepRows,:)-Mint(keepRows,:)*(MintSolve\Mdd(solveRows,:)),'Steps',20);
rred = simplify(-craw(keepRows)+Mint(keepRows,:)*(MintSolve\craw(solveRows)),'Steps',20);
assert(isequal(size(N),[15 5]),'符号消元秩异常：N 为 %dx%d。',size(N,1),size(N,2));
G = simplify(subs(rred,[dq;U],zeros(9,1)),'Steps',20);
Btau = simplify(jacobian(rred,U),'Steps',20);
C = simplify(rred-Btau*U-G,'Steps',20);

out = struct('q',q,'dq',dq,'ddq',ddq,'U',U,'parameters', ...
    [R_w;R_l;l_l;l_r;l_w_l;l_w_r;l_b_l;l_b_r;l_c;phi_c;m_w;m_l;m_r;m_b;I_w;I_l;I_r;I_b;I_zz;g], ...
    'Mred',Mred,'Btau',Btau,'G',G,'C',C,'N',N, ...
    'elimination_rows',struct('solveRows',solveRows,'keepRows',keepRows, ...
        'description','Solve Eq.(21)-(24), Eq.(28), Eq.(31) for internal forces; substitute into Eq.(25)-(27), Eq.(29)-(30).'), ...
    'raw_equations',e, ...
    'raw_M',Mraw,'raw_c',craw);
end
