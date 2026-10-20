function out = zju_nonlinear_eliminate(p,q,dq,U)
%ZJU_NONLINEAR_ELIMINATE 文档 Eq.(21)-(31) 的数值消元。
% q=[theta_w_l theta_w_r theta_l_l theta_l_r theta_b]^T。
% 内部力 zint=[f_l f_r Fws_l Fws_r Fwh_l Fwh_r Fbs_l Fbs_r Fbh_l Fbh_r]^T
% 被消去后得到 Mred(q)*ddq = rred(q,dq,U)，保留全部速度非线性项。
q=q(:); dq=dq(:); U=U(:); assert(numel(q)==5 && numel(dq)==5 && numel(U)==4);
Ucmd=U; % 嵌套的 reduce_at 会临时改变 U，保留原始四维实际输入。
% 消元后的右端按物理来源拆开：
% r = Q_tau(U) + G(q) + C(q,dq)。
% G 同时包含重力和姿态几何项；C 是速度二次耦合项。
[Mred,rred,Mall,call,N] = reduce_at(dq,Ucmd);
[~,Gred] = reduce_at(zeros(5,1),zeros(4,1));
[~,r_no_input] = reduce_at(dq,zeros(4,1));
Cred = r_no_input-Gred;
Btau=zeros(5,4);
for kk=1:4
    Uk=zeros(4,1); Uk(kk)=1;
    [~,rk]=reduce_at(zeros(5,1),Uk);
    Btau(:,kk)=rk-Gred;
end
assert(isequal(size(Btau),[5 4]),'B_tau 的尺寸应为 5x4。');
assert(isequal(size(Ucmd),[4 1]),'输入扭矩 U 的尺寸应为 4x1。');
Qtau=Btau*Ucmd;
z=Mall\(-call);
% raw_M/raw_c 是 Eq.(21)-(31) 中 [ddq; internal_forces] 的 15x15 系数和常数项。
out=struct('Mred',Mred,'rred',rred,'ddq',z(1:5),'internal_forces',z(6:end), ...
    'B_tau',Btau,'Q_tau',Qtau,'G_q',Gred,'C_qdq',Cred,'elimination_basis',N, ...
    'raw_M',Mall,'raw_c',call,'q',q,'dq',dq,'U',Ucmd,'p',p, ...
    'residual',@(ddq)Mred*ddq(:)-rred);

    function [Mr,rr,Mfull,cfull,Nlocal]=reduce_at(dq_eval,U_eval)
        % 此函数用同一组 q，在指定速度/输入下消元；只用于分解右端项。
        dq_saved=dq; U_saved=U;
        dq=dq_eval; U=U_eval;
        z0=zeros(15,1); cfull=residual(z0); Mfull=zeros(15);
        for jcol=1:15
            ek=zeros(15,1); ek(jcol)=1;
            Mfull(:,jcol)=residual(ek)-cfull;
        end
        Mdd=Mfull(:,1:5); Mint=Mfull(:,6:end);
        % 与文档“联立轮、腿、机体方程求内力，再代回”的顺序一致：
        % Eq.(21)-(24)、Eq.(28)、Eq.(31) 解十个内力；保留 Eq.(25)-(27)、Eq.(29)-(30)。
        % 该选行在左右完全对称的直立零位也非奇异。
        solveRows=[1 2 3 4 5 6 8 9 12 15];
        keepRows=[7 10 11 13 14];
        MintSolve=Mint(solveRows,:);
        assert(rcond(MintSolve)>1e-12,'内部力消元矩阵病态或奇异。');
        Nlocal=[-(MintSolve.'\Mint(keepRows,:).'); eye(5)];
        Mr=Mdd(keepRows,:)-Mint(keepRows,:)*(MintSolve\Mdd(solveRows,:));
        rr=-cfull(keepRows)+Mint(keepRows,:)*(MintSolve\cfull(solveRows));
        dq=dq_saved; U=U_saved;
    end

    function e=residual(z)
        ddq=z(1:5); zi=z(6:end);
        fl=zi(1); fr=zi(2); Fwsl=zi(3); Fwsr=zi(4); Fwhl=zi(5); Fwhr=zi(6);
        Fbsl=zi(7); Fbsr=zi(8); Fbhl=zi(9); Fbhr=zi(10);
        [~,phidd,Sbdd,hbdd,Slldd,hlldd,Slrdd,hlrdd]=kin(ddq);
        twl=U(1); twr=U(2); tbl=U(3); tbr=U(4);
        tl=q(3); tr=q(4); tb=q(5); Rw=p.R_w; g=p.g;
        al=tl-acos((p.l_w_l^2+p.l_l^2-p.l_b_l^2)/(2*p.l_w_l*p.l_l));
        ar=tr-acos((p.l_w_r^2+p.l_r^2-p.l_b_r^2)/(2*p.l_w_r*p.l_r));
        bl=tl+acos((p.l_b_l^2+p.l_l^2-p.l_w_l^2)/(2*p.l_b_l*p.l_l));
        br=tr+acos((p.l_b_r^2+p.l_r^2-p.l_w_r^2)/(2*p.l_b_r*p.l_r));
        e=[p.m_w*Rw*ddq(1)-fl+Fwsl; p.I_w*ddq(1)-twl+fl*Rw;
           p.m_w*Rw*ddq(2)-fr+Fwsr; p.I_w*ddq(2)-twr+fr*Rw;
           p.m_l*Slldd-Fwsl+Fbsl; p.m_l*hlldd-Fwhl+Fbhl+p.m_l*g;
           % Eq.(25)-(26): 将右端的腿-机体力矩移至残差左端，二者必须变号。
           p.I_l*ddq(3)-Fwhl*p.l_w_l*sin(al)+Fwsl*p.l_w_l*cos(al)-Fbhl*p.l_b_l*sin(bl)+Fbsl*p.l_b_l*cos(bl)+twl-tbl;
           p.m_r*Slrdd-Fwsr+Fbsr; p.m_r*hlrdd-Fwhr+Fbhr+p.m_r*g;
           % 右腿同理，对应原文“l -> r”的对称方程。
           p.I_r*ddq(4)-Fwhr*p.l_w_r*sin(ar)+Fwsr*p.l_w_r*cos(ar)-Fbhr*p.l_b_r*sin(br)+Fbsr*p.l_b_r*cos(br)+twr-tbr;
           p.m_b*Sbdd-Fbsl-Fbsr; p.m_b*hbdd-Fbhl-Fbhr+p.m_b*g;
           p.I_b*ddq(5)+tbl+tbr+(Fbsl+Fbsr)*p.l_c*cos(tb+p.phi_c)-(Fbhl+Fbhr)*p.l_c*sin(tb+p.phi_c);
           p.I_zz*phidd+(fl-fr)*p.R_l; Fwhl-Fwhr];
    end
    function [sdd,phidd,Sbdd,hbdd,Slldd,hlldd,Slrdd,hlrdd]=kin(ddq)
        % 文档 Eq.(7)、(10)-(11)、(14)-(18) 的解析二阶导数。
        tl=q(3); tr=q(4); tb=q(5);
        dtl=dq(3); dtr=dq(4); dtb=dq(5);
        al=tl-acos((p.l_w_l^2+p.l_l^2-p.l_b_l^2)/(2*p.l_w_l*p.l_l));
        ar=tr-acos((p.l_w_r^2+p.l_r^2-p.l_b_r^2)/(2*p.l_w_r*p.l_r));
        bl=tl+acos((p.l_b_l^2+p.l_l^2-p.l_w_l^2)/(2*p.l_b_l*p.l_l));
        br=tr+acos((p.l_b_r^2+p.l_r^2-p.l_w_r^2)/(2*p.l_b_r*p.l_r));
        bc=tb+p.phi_c;
        sdd=p.R_w/2*(ddq(1)+ddq(2));
        phidd=-p.R_w/(2*p.R_l)*(ddq(1)-ddq(2)) ...
            -p.l_l/(2*p.R_l)*cos(tl)*ddq(3)+p.l_r/(2*p.R_l)*cos(tr)*ddq(4) ...
            +p.l_l/(2*p.R_l)*sin(tl)*dtl^2-p.l_r/(2*p.R_l)*sin(tr)*dtr^2;
        Sbdd=p.R_w/2*(ddq(1)+ddq(2)) ...
            +p.l_l/2*cos(tl)*ddq(3)+p.l_r/2*cos(tr)*ddq(4)+p.l_c*cos(bc)*ddq(5) ...
            -p.l_l/2*sin(tl)*dtl^2-p.l_r/2*sin(tr)*dtr^2-p.l_c*sin(bc)*dtb^2;
        hbdd=-p.l_l/2*(sin(tl)*ddq(3)+cos(tl)*dtl^2) ...
            -p.l_r/2*(sin(tr)*ddq(4)+cos(tr)*dtr^2) ...
            -p.l_c*(sin(bc)*ddq(5)+cos(bc)*dtb^2);
        Slldd=p.R_w*ddq(1)+p.l_w_l*(cos(al)*ddq(3)-sin(al)*dtl^2);
        Slrdd=p.R_w*ddq(2)+p.l_w_r*(cos(ar)*ddq(4)-sin(ar)*dtr^2);
        hlldd=hbdd+p.l_b_l*(sin(bl)*ddq(3)+cos(bl)*dtl^2) ...
            +p.l_c*(sin(bc)*ddq(5)+cos(bc)*dtb^2);
        hlrdd=hbdd+p.l_b_r*(sin(br)*ddq(4)+cos(br)*dtr^2) ...
            +p.l_c*(sin(bc)*ddq(5)+cos(bc)*dtb^2);
    end
end
