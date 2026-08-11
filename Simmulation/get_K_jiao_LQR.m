function K = get_K_jiao_LQR(leg_length)
% v1：这份LQR程序是参考我之前写的哈工程LQR程序以及小周写的AB矩阵求解器优化后写出来的，感谢周神（2024/05/07）
% v2：添加了可以专门调试定腿长的功能（2024/05/08）
% v3：优化部分注释，添加单位说明（2024/05/15）
% v4: 优化了输出，输出矩阵K的系数可以真正的复制到C里（2024/05/16）

% 以下所有变量含义参考2023上交轮腿电控开源（https://bbs.robomaster.com/forum.php?mod=viewthread&tid=22756）所使用符号含义

%%%%%%%%%%%%%%%%%%%%%%%%%Step 0：重置程序，定义变量%%%%%%%%%%%%%%%%%%%%%%%%%

% 定义机器人机体参数
syms R_w                % 驱动轮半径
syms R_l                % 驱动轮轮距/2
syms l_l l_r            % 左右腿长
syms l_wl l_wr          % 驱动轮质心到左右腿部质心距离
syms l_bl l_br          % 机体质心到左右腿部质心距离
syms l_c                % 机体质心到腿部关节中心点距离
syms m_w m_l m_b        % 驱动轮质量 腿部质量 机体质量
syms I_w                % 驱动轮转动惯量           (自然坐标系法向)
syms I_ll I_lr          % 驱动轮左右腿部转动惯量    (自然坐标系法向，实际上会变化)
syms I_b                % 机体转动惯量             (自然坐标系法向)
syms I_z                % 机器人z轴转动惯量        (简化为常量)

% 定义其他独立变量并补充其导数
syms theta_wl   theta_wr   % 左右驱动轮转角
syms dtheta_wl  dtheta_wr 
syms ddtheta_wl ddtheta_wr ddtheta_ll ddtheta_lr ddtheta_b

% 定义状态向量
syms s ds phi dphi theta_ll dtheta_ll theta_lr dtheta_lr theta_b dtheta_b

% 定义控制向量
syms T_wl T_wr T_bl T_br

% 输入物理参数：重力加速度
syms g



%%%%%%%%%%%%%%%%%%%%%%%Step 1：解方程，求控制矩阵A，B%%%%%%%%%%%%%%%%%%%%%%%

% 通过原文方程组(3.11)-(3.15)，求出ddtheta_wl,ddtheta_wr,ddtheta_ll,ddtheta_lr,ddtheta_b表达式
eqn1 = (I_w*l_l/R_w+m_w*R_w*l_l+m_l*R_w*l_bl)*ddtheta_wl+(m_l*l_wl*l_bl-I_ll)*ddtheta_ll+(m_l*l_wl+m_b*l_l/2)*g*theta_ll+T_bl-T_wl*(1+l_l/R_w)==0;
eqn2 = (I_w*l_r/R_w+m_w*R_w*l_r+m_l*R_w*l_br)*ddtheta_wr+(m_l*l_wr*l_br-I_lr)*ddtheta_lr+(m_l*l_wr+m_b*l_r/2)*g*theta_lr+T_br-T_wr*(1+l_r/R_w)==0;
eqn3 = -(m_w*R_w*R_w+I_w+m_l*R_w*R_w+m_b*R_w*R_w/2)*ddtheta_wl-(m_w*R_w*R_w+I_w+m_l*R_w*R_w+m_b*R_w*R_w/2)*ddtheta_wr-(m_l*R_w*l_wl+m_b*R_w*l_l/2)*ddtheta_ll-(m_l*R_w*l_wr+m_b*R_w*l_r/2)*ddtheta_lr+T_wl+T_wr==0;
eqn4 = (m_w*R_w*l_c+I_w*l_c/R_w+m_l*R_w*l_c)*ddtheta_wl+(m_w*R_w*l_c+I_w*l_c/R_w+m_l*R_w*l_c)*ddtheta_wr     +m_l*l_wl*l_c*ddtheta_ll      +m_l*l_wr*l_c*ddtheta_lr-I_b*ddtheta_b+m_b*g*l_c*theta_b    -(T_wl+T_wr)*l_c/R_w-(T_bl+T_br)==0;
eqn5 = ((I_z*R_w)/(2*R_l)+I_w*R_l/R_w)*ddtheta_wl-((I_z*R_w)/(2*R_l)+I_w*R_l/R_w)*ddtheta_wr+(I_z*l_l)/(2*R_l)*ddtheta_ll-(I_z*l_r)/(2*R_l)*ddtheta_lr-T_wl*R_l/R_w+T_wr*R_l/R_w==0;
[ddtheta_wl,ddtheta_wr,ddtheta_ll,ddtheta_lr,ddtheta_b] = solve(eqn1,eqn2,eqn3,eqn4,eqn5,ddtheta_wl,ddtheta_wr,ddtheta_ll,ddtheta_lr,ddtheta_b);


% 通过计算雅可比矩阵的方法得出控制矩阵A，B所需要的全部偏导数
J_A = jacobian([ddtheta_wl,ddtheta_wr,ddtheta_ll,ddtheta_lr,ddtheta_b],[theta_ll,theta_lr,theta_b]);
J_B = jacobian([ddtheta_wl,ddtheta_wr,ddtheta_ll,ddtheta_lr,ddtheta_b],[T_wl,T_wr,T_bl,T_br]);

% 定义矩阵A，B，将指定位置的数值根据上述偏导数计算出来并填入
A = sym('A',[10 10]);
B = sym('B',[10 4]);

% 填入A数据：a25,a27,a29,a45,a47,a49,a65,a67,a69,a85,a87,a89,a105,a107,a109
for p = 5:2:9
    A_index = (p - 3)/2;
    A(2,p) = R_w*(J_A(1,A_index) + J_A(2,A_index))/2;
    A(4,p) = (R_w*(- J_A(1,A_index) + J_A(2,A_index)))/(2*R_l) - (l_l*J_A(3,A_index))/(2*R_l) + (l_r*J_A(4,A_index))/(2*R_l);
    for q = 6:2:10
        A(q,p) = J_A(q/2,A_index);
    end
end

% A的以下数值为1：a12,a34,a56,a78,a910，其余数值为0
for r = 1:10
    if rem(r,2) == 0
        A(r,1) = 0; A(r,2) = 0; A(r,3) = 0; A(r,4) = 0; A(r,6) = 0; A(r,8) = 0; A(r,10) = 0;
    else
        A(r,:) = zeros(1,10);
        A(r,r+1) = 1;
    end
end

% 填入B数据：b21,b22,b23,b24,b41,b42,b43,b44,b61,b62,b63,b64,b81,b82,b83,b84,b101,b102,b103,b104,
for h = 1:4
    B(2,h) = R_w*(J_B(1,h) + J_B(2,h))/2;
    B(4,h) = (R_w*(- J_B(1,h) + J_B(2,h)))/(2*R_l) - (l_l*J_B(3,h))/(2*R_l) + (l_r*J_B(4,h))/(2*R_l);
    for f = 6:2:10
        B(f,h) = J_B(f/2,h);
    end
end

% B的其余数值为0
for e = 1:2:9
    B(e,:) = zeros(1,4);
end



%%%%%%%%%%%%%%%%%%%%%Step 2：输入参数（可以修改的部分）%%%%%%%%%%%%%%%%%%%%%

% 物理参数赋值（唯一此处不可改变！），后面的数据通过增加后缀_ac区分模型符号和实际数据
g_ac = 9.81;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                      此处可以输入机器人机体基本参数                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%机器人机体与轮部参数%%%%%%%%%%%%%%%%%%%%%%%%%%%%

R_w_ac = 0.0525;                           % 驱动轮半径                  （单位：m）
R_l_ac = 0.20;                            % 两个驱动轮之间距离/2         （单位：m）
l_c_ac = -0.03;                            % 机体质心到腿部关节中心点距离  （单位：m）
m_w_ac = 0.3; m_l_ac = 0.8; m_b_ac = 15.75;    % 驱动轮质量 腿部质量 机体质量  （单位：kg）
I_w_ac = 0.5*m_w_ac*R_w_ac^2;                            % 驱动轮转动惯量               （单位：kg m^2）
I_b_ac = m_b_ac*(0.485^2+0.152^2)/12.0;                            % 机体转动惯量(自然坐标系法向)  （单位：kg m^2）
I_z_ac = 0.226;                            % 机器人z轴转动惯量            （单位：kg m^2）

%%%%%%%%%%%%%%%%%%%%%%机器人腿部参数（定腿长调试用）%%%%%%%%%%%%%%%%%%%%%%%%

% 如果需要使用此部分，请去掉120-127行以及215-218行注释，然后将224行之后的所有代码注释掉
% 或者点击左侧数字"224"让程序只运行行之前的内容并停止

l_l_ac = leg_length;        % 左腿摆杆长度                      （左腿对应数据）  （单位：m）
l_wl_ac = leg_length/2;       % 左驱动轮质心到左腿摆杆质心距离                      （单位：m）
l_bl_ac = leg_length/2;       % 机体转轴到左腿摆杆质心距离                          （单位：m）
I_ll_ac = m_l_ac*((l_wl_ac+l_bl_ac)^2+0.048^2)/12.0;      % 左腿摆杆转动惯量                                   （单位：kg m^2）
l_r_ac = leg_length;        % 右腿摆杆长度                      （右腿对应数据）  （单位：m）
l_wr_ac = leg_length/2;       % 右驱动轮质心到右腿摆杆质心距离                      （单位：m）
l_br_ac = leg_length/2;       % 机体转轴到右腿摆杆质心距离                          （单位：m）
I_lr_ac = m_l_ac*((l_wr_ac+l_br_ac)^2+0.048^2)/12.0;      % 右腿摆杆转动惯量                                   （单位：kg m^2）

% 机体转轴定义参考哈工程开源（https://zhuanlan.zhihu.com/p/563048952），是左右
% 两侧两个关节电机之间的中间点相连所形成的轴
% （如果目的是小板凳，考虑使左右腿相关数据一致）

%%%%%%%%%%%%%%%%%%%%%%%%%%%机器人腿部参数数据集%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 根据不同腿长长度，先针对左腿测量出对应的l_wl，l_bl，和I_ll
% 通过以下方式记录数据: 矩阵分4列，
% 第一列为左腿腿长范围区间中所有小数点精度0.01的长度，例如：0.09，0.18，单位：m
% 第二列为l_wl，单位：m
% 第三列为l_bl，单位：m
% 第四列为I_ll，单位：kg m^2
% （注意单位别搞错！）
% 行数根据L_0范围区间（，单位cm时）的整数数量进行调整

Leg_data_l = [0.11,  0.0480,  0.0620,  0.01819599;
              0.12,  0.0470,  0.0730,  0.01862845;
              0.13,  0.0476,  0.0824,  0.01898641;
              0.14,  0.0480,  0.0920,  0.01931342;
              0.15,  0.0490,  0.1010,  0.01962521;
              0.16,  0.0500,  0.1100,  0.01993092;
              0.17,  0.0510,  0.1190,  0.02023626;
              0.18,  0.0525,  0.1275,  0.02054500;
              0.19,  0.0539,  0.1361,  0.02085969;
              0.20,  0.0554,  0.1446,  0.02118212;
              0.21,  0.0570,  0.1530,  0.02151357;
              0.22,  0.0586,  0.1614,  0.02185496;
              0.23,  0.0600,  0.1700,  0.02220695;
              0.24,  0.0621,  0.1779,  0.02256999;
              0.25,  0.0639,  0.1861,  0.02294442;
              0.26,  0.0657,  0.1943,  0.02333041;
              0.27,  0.0676,  0.2024,  0.02372806;
              0.28,  0.0700,  0.2100,  0.02413735;
              0.29,  0.0713,  0.2187,  0.02455817;
              0.30,  0.0733,  0.2267,  0.02499030];
% 以上数据应通过实际测量或sw图纸获得

% 由于左右腿部数据通常完全相同，我们通过复制的方式直接定义右腿的全部数据集
% 矩阵分4列，第一列为右腿腿长范围区间中（，单位cm时）的整数腿长l_r*0.01，第二列为l_wr，第三列为l_br，第四列为I_lr）
Leg_data_r = Leg_data_l;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                           此处可以输入QR矩阵                           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 矩阵Q中，以下列分别对应：
%        s     ds     phi     dphi     theta_ll dtheta_ll theta_lr dtheta_lr theta_b dtheta_b
lqr_Q = [20,    0,     0,      0,       0,       0,        0,       0,        0,      0;
         0,    1,     0,      0,       0,       0,        0,       0,        0,      0;
         0,    0,     400,      0,       0,       0,        0,       0,        0,      0;
         0,    0,     0,     1,       0,       0,        0,       0,        0,       0;
         0,    0,     0,      0,       40000,       0,        0,       0,        0,      0;
         0,    0,     0,      0,       0,       10,        0,       0,        0,      0;
         0,    0,     0,      0,       0,       0,       40000,       0,        0,      0;
         0,    0,     0,      0,       0,       0,        0,       10,        0,      0;
         0,    0,     0,      0,       0,       0,        0,       0,        30000,      0;
         0,    0,     0,      0,       0,       0,        0,       0,        0,      1];
% 其中：
% s       : 自然坐标系下机器人水平方向移动距离，单位：m，ds为其导数
% phi     ：机器人水平方向移动时yaw偏航角度，dphi为其导数
% theta_ll：左腿摆杆与竖直方向（自然坐标系z轴）夹角，dtheta_ll为其导数
% theta_lr：右腿摆杆与竖直方向（自然坐标系z轴）夹角，dtheta_lr为其导数
% theta_b ：机体与自然坐标系水平夹角，dtheta_b为其导数

% 矩阵中，以下列分别对应：
%        T_wl    T_wr     T_bl     T_br
lqr_R = [10,      0,       0,       0;
         0,      10,       0,       0;
         0,      0,       1,       0;
         0,      0,       0,       1];
% 其中：
% T_wl: 左侧驱动轮输出力矩
% T_wr：右侧驱动轮输出力矩
% T_bl：左侧髋关节输出力矩
% T_br：右腿髋关节输出力矩
% 单位皆为Nm

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%Step 2.5：求解矩阵（定腿长调试用）%%%%%%%%%%%%%%%%%%%%%

% 如果需要使用此部分，请去掉120-127行以及215-218行注释，然后将224行之后的所有代码注释掉，
% 或者点击左侧数字"224"让程序只运行行之前的内容并停止
K = get_K_from_LQR(R_w,R_l,l_l,l_r,l_wl,l_wr,l_bl,l_br,l_c,m_w,m_l,m_b,I_w,I_ll,I_lr,I_b,I_z,g, ...
            R_w_ac,R_l_ac,l_l_ac,l_r_ac,l_wl_ac,l_wr_ac,l_bl_ac,l_br_ac, ...
            l_c_ac,m_w_ac,m_l_ac,m_b_ac,I_w_ac,I_ll_ac,I_lr_ac,I_b_ac,I_z_ac,g_ac, ...
            A,B,lqr_Q,lqr_R)
%K = sprintf([strjoin(repmat({'%.5g'},1,size(K,2)),',  ') '\n'], K.')

end

function K = get_K_from_LQR(R_w,R_l,l_l,l_r,l_wl,l_wr,l_bl,l_br,l_c,m_w,m_l,m_b,I_w,I_ll,I_lr,I_b,I_z,g, ...
            R_w_ac,R_l_ac,l_l_ac,l_r_ac,l_wl_ac,l_wr_ac,l_bl_ac,l_br_ac, ...
            l_c_ac,m_w_ac,m_l_ac,m_b_ac,I_w_ac,I_ll_ac,I_lr_ac,I_b_ac,I_z_ac,g_ac, ...
            A,B,lqr_Q,lqr_R)
    % 基于机体以及物理参数，获得控制矩阵A，B的全部数值
    A_ac = subs(A,[R_w R_l l_l l_r l_wl l_wr l_bl l_br l_c m_w m_l m_b I_w I_ll I_lr I_b I_z g], ...
        [R_w_ac R_l_ac l_l_ac l_r_ac l_wl_ac l_wr_ac l_bl_ac l_br_ac l_c_ac ...
        m_w_ac m_l_ac m_b_ac I_w_ac I_ll_ac I_lr_ac I_b_ac I_z_ac g_ac]);
    B_ac = subs(B,[R_w R_l l_l l_r l_wl l_wr l_bl l_br l_c m_w m_l m_b I_w I_ll I_lr I_b I_z g], ...
        [R_w_ac R_l_ac l_l_ac l_r_ac l_wl_ac l_wr_ac l_bl_ac l_br_ac l_c_ac ...
        m_w_ac m_l_ac m_b_ac I_w_ac I_ll_ac I_lr_ac I_b_ac I_z_ac g_ac]);

    % 根据以上信息和提供的矩阵Q和R求解Riccati方程，获得增益矩阵K
    % P为Riccati方程的解，矩阵L可以无视
    [P,K,L_k] = icare(A_ac,B_ac,lqr_Q,lqr_R,[],[],[]);
end
