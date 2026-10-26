function result = wheelleg_lqr(command, leg_length)
%WHEELLEG_LQR Unified report and LQR entry for the equivalent straight-leg model.
%
%   result = wheelleg_lqr('solve',    0.20);  % calculate only
%   wheelleg_lqr('parameters',        0.20);  % all physical parameters
%   wheelleg_lqr('equilibrium',       0.20);  % X*, U*, angles and inputs
%   wheelleg_lqr('symbolic',          0.20);  % full five symbolic equations
%   wheelleg_lqr('five',              0.20);  % five numerical reduced equations
%   wheelleg_lqr('state',             0.20);  % full 10th-order state equations
%   wheelleg_lqr('k',                 0.20);  % Q, R, P, K and poles
%   wheelleg_lqr('checks',            0.20);  % residuals and validity checks
%   wheelleg_lqr('all',               0.20);  % print every report above
%
% Control law for deployment is U = U_star - K*(X-X_star).

if nargin < 1 || isempty(command), command = 'konly'; end
if nargin < 2 || isempty(leg_length), leg_length = 0.20; end
command = lower(char(command));

if strcmp(command,'help')
    printHelp(); result = []; return;
end

needsResult = ~strcmp(command,'symbolic');
if needsResult
    result = zju_lqr_gain([],[],leg_length);
else
    result = [];
end

switch command
    case 'konly'
        fprintf('K (leg_length=%.6f m), rows [Twl_l Twl_r Tbl_l Tbl_r]:\n',leg_length);
        disp(result.K);
    case {'solve','result'}
        % Numerical result is returned without terminal output.
    case {'parameters','params'}
        printParameters(result.p);
    case {'equilibrium','eq'}
        printParameters(result.p);
        printEquilibrium(result.equilibrium);
    case {'symbolic','five-symbolic'}
        printSymbolDictionary();
        printSymbolicReport();
    case {'five','five-numeric','nonlinear'}
        printParameters(result.p);
        printFiveEquations(result);
    case {'state','state-space','linear'}
        printParameters(result.p);
        printEquilibrium(result.equilibrium);
        printStateEquations(result);
        printStateMatrices(result);
    case {'k','lqr'}
        printParameters(result.p);
        printEquilibrium(result.equilibrium);
        printLqr(result);
    case {'checks','check','validate'}
        printChecks(result);
    case 'all'
        printParameters(result.p);
        printEquilibrium(result.equilibrium);
        printSymbolDictionary();
        printSymbolicReport();
        printFiveEquations(result);
        printStateEquations(result);
        printStateMatrices(result);
        printLqr(result);
        printChecks(result);
    otherwise
        error('Unknown command "%s". Run wheelleg_lqr(''help'').',command);
end
end

function printHelp()
fprintf(['\nUnified equivalent-straight-leg LQR model\n', ...
 'Running wheelleg_lqr with no input prints K for the default 0.20 m leg.\n', ...
 'Commands: parameters | equilibrium | symbolic | five | state | k | checks | all | solve\n', ...
 'Example: wheelleg_lqr(''all'',0.20)\n', ...
 'All lengths use m, angles rad, mass kg, inertia kg*m^2, torque N*m.\n\n']);
end

function printSymbolicReport()
formulaFile = fullfile(fileparts(mfilename('fullpath')),'zju_非线性符号方程.md');
assert(isfile(formulaFile),'Missing symbolic formula document: %s',formulaFile);
fprintf('\n============= Full symbolic five-equation report =============\n');
fprintf('Formula file: %s\n',formulaFile);
fprintf('%s\n',fileread(formulaFile));
fprintf('================================================================\n');
end

function printParameters(p)
rows = {
 'g',p.g,'m/s^2','gravity';
 'R_w',p.R_w,'m','wheel radius';
 'R_l',p.R_l,'m','half wheel track';
 'm_w',p.m_w,'kg','single wheel equivalent mass';
 'I_w',p.I_w,'kg*m^2','single wheel axial inertia';
 'm_b',p.m_b,'kg','body equivalent mass';
 'I_b',p.I_b,'kg*m^2','body pitch inertia about its CoM';
 'I_zz',p.I_zz,'kg*m^2','whole robot yaw inertia';
 'l_c',p.l_c,'m','hip-midpoint to body-CoM signed sagittal distance';
 'phi_c',p.phi_c,'rad','hip-to-CoM angle from body +z, in pitch plane';
 'm_l',p.m_l,'kg','left equivalent-leg mass';
 'm_r',p.m_r,'kg','right equivalent-leg mass';
 'I_l',p.I_l,'kg*m^2','left leg inertia about its CoM';
 'I_r',p.I_r,'kg*m^2','right leg inertia about its CoM';
 'l_l',p.l_l,'m','left virtual-leg length';
 'l_r',p.l_r,'m','right virtual-leg length';
 'l_w_l',p.l_w_l,'m','left wheel centre to leg CoM';
 'l_w_r',p.l_w_r,'m','right wheel centre to leg CoM';
 'l_b_l',p.l_b_l,'m','left hip to leg CoM';
 'l_b_r',p.l_b_r,'m','right hip to leg CoM';
 'Ts',p.Ts,'s','controller sample period';
 'linearization_step',p.linearization_step,'-','central-difference step'};
fprintf('\n============= Physical parameter table =============\n');
fprintf('%-20s %-18s %-12s %s\n','symbol','value','unit','meaning');
for k=1:size(rows,1)
    fprintf('%-20s %-18.12g %-12s %s\n',rows{k,1},rows{k,2},rows{k,3},rows{k,4});
end
fprintf('======================================================\n');
fprintf('Coordinate/sign convention inherited from get_K_jiao_LQR.m:\n');
fprintf('  body: %s; yaw +: %s\n',p.coordinate_convention.body_axes,p.coordinate_convention.yaw_positive);
fprintf('  leg plane: %s; leg angle +: %s\n', ...
    p.coordinate_convention.leg_plane_axes,p.coordinate_convention.leg_angle_positive);
end

function printSymbolDictionary()
fprintf('\n============= Symbol dictionary for five reduced equations =============\n');
fprintf('q=[theta_w_l theta_w_r theta_l_l theta_l_r theta_b]^T\n');
fprintf('ddq=[ddtheta_w_l ddtheta_w_r ddtheta_l_l ddtheta_l_r ddtheta_b]^T\n');
fprintf('U=[Twl_l Twl_r Tbl_l Tbl_r]^T\n');
fprintf('Mred(q)*ddq = Btau(q)*U + G(q) + C(q,dq)\n');
fprintf('Mred: 5x5 generalized inertia matrix; Btau: 5x4 torque mapping;\n');
fprintf('Q_tau=Btau*U, G: gravity/geometry vector, C: velocity coupling vector.\n');
fprintf('The symbolic report lists every Mred(ij), Btau(ij), Q_tau(i), G(i), C(i),\n');
fprintf('then lists all five complete equations in equation-group form.\n');
fprintf('========================================================================\n');
end

function printEquilibrium(e)
fprintf('\n============= Static equilibrium and linearization point =============\n');
fprintf('X*=[s ds phi dphi theta_l_l dtheta_l_l theta_l_r dtheta_l_r theta_b dtheta_b]^T\n');
for k=1:10, fprintf('  %-16s = %+.12g\n',e.state_names{k},e.X(k)); end
fprintf('U*=[Twl_l Twl_r Tbl_l Tbl_r]^T N*m\n');
for k=1:4, fprintf('  %-16s = %+.12g N*m\n',e.input_names{k},e.U(k)); end
fprintf('Selection constraints: %s\n',e.selection_rule);
fprintf('Static residual norm = %.3e; xdot(X*,U*) norm = %.3e\n', ...
    e.residual_norm,norm(e.x_dot_at_equilibrium));
fprintf('=======================================================================\n');
end

function printFiveEquations(r)
e = r.equilibrium;
n = zju_nonlinear_eliminate(r.p,e.q,e.dq,e.U);
fprintf('\n============= Five reduced nonlinear equations: numerical point =============\n');
fprintf('Mred(q)*ddq = Btau(q)*U + G(q) + C(q,dq)\n');
fprintf('q and U below are the solved static equilibrium values; dq=0.\n');
fprintf('Mred (5x5), every element:\n'); disp(n.Mred);
fprintf('Btau (5x4), every element:\n'); disp(n.B_tau);
fprintf('Q_tau=Btau*U (5x1):\n'); disp(n.Q_tau);
fprintf('G(q) (5x1):\n'); disp(n.G_q);
fprintf('C(q,dq) (5x1):\n'); disp(n.C_qdq);
fprintf('Five scalar numerical equations, with ddq ordered [ww_l ww_r leg_l leg_r body]^T:\n');
for i=1:5
    fprintf('F%d: ',i); fprintf('%+.10g*ddq%d ',[n.Mred(i,:);1:5]);
    fprintf('= %+.10g + %+.10g + %+.10g\n',n.Q_tau(i),n.G_q(i),n.C_qdq(i));
end
fprintf('Residual at ddq=0: '); disp(n.residual(zeros(5,1)).');
fprintf('===============================================================================\n');
end

function printStateMatrices(r)
fprintf('\n============= Linearized 10-state matrices =============\n');
fprintf('delta_Xdot=A*delta_X+B*delta_U\nA (10x10), every element:\n'); disp(r.A);
fprintf('B (10x4), every element:\n'); disp(r.B);
fprintf('Q (10x10), every element:\n'); disp(r.Q);
fprintf('R (4x4), every element:\n'); disp(r.R);
fprintf('==========================================================\n');
end

function printStateEquations(r)
A = r.A; B = r.B;
xNames = {'delta_s','delta_ds','delta_phi','delta_dphi', ...
    'delta_theta_l_l','delta_dtheta_l_l','delta_theta_l_r', ...
    'delta_dtheta_l_r','delta_theta_b','delta_dtheta_b'};
uNames = {'delta_Twl_l','delta_Twl_r','delta_Tbl_l','delta_Tbl_r'};
dotNames = {'d(delta_s)/dt','d(delta_ds)/dt','d(delta_phi)/dt', ...
    'd(delta_dphi)/dt','d(delta_theta_l_l)/dt','d(delta_dtheta_l_l)/dt', ...
    'd(delta_theta_l_r)/dt','d(delta_dtheta_l_r)/dt', ...
    'd(delta_theta_b)/dt','d(delta_dtheta_b)/dt'};
fprintf('\n============= Full 10th-order linearized equations =============\n');
fprintf('delta_Xdot=A*delta_X+B*delta_U; delta_X=X-X*; delta_U=U-U*\n');
for i=1:10
    rhs = linearExpression(A(i,:),xNames);
    inputPart = linearExpression(B(i,:),uNames);
    if ~strcmp(inputPart,'0'), rhs = [rhs ' + ' inputPart]; end %#ok<AGROW>
    fprintf('%-24s = %s\n',dotNames{i},rhs);
end
fprintf('==================================================================\n');
end

function text = linearExpression(coefficient,names)
terms = strings(0,1);
for k=1:numel(coefficient)
    if abs(coefficient(k)) > 1e-9
        terms(end+1) = sprintf('%+.7g*%s',coefficient(k),names{k}); %#ok<AGROW>
    end
end
if isempty(terms), text = '0'; else, text = char(strjoin(terms,' ')); end
end

function printLqr(r)
fprintf('\n============= Continuous LQR result =============\n');
fprintf('Control law: %s\n',r.control_law);
fprintf('P (10x10), every element:\n'); disp(r.P);
fprintf('K (4x10), every element, input order [Twl_l Twl_r Tbl_l Tbl_r]:\n'); disp(r.K);
fprintf('Open-loop poles:\n'); disp(r.open_loop_poles.');
fprintf('Closed-loop poles:\n'); disp(r.closed_loop_poles.');
fprintf('===================================================\n');
end

function printChecks(r)
e = r.equilibrium;
n = zju_nonlinear_eliminate(r.p,e.q,e.dq,e.U);
M = n.Mred;
fprintf('\n============= Calculation validity checks =============\n');
fprintf('static five-equation residual norm:     %.3e\n',e.residual_norm);
fprintf('state derivative norm at (X*,U*):       %.3e\n',norm(e.x_dot_at_equilibrium));
fprintf('Mred rank / rcond at equilibrium:       %d / %.3e\n',rank(M),rcond(M));
fprintf('controllability rank [A B]:              %d / 10\n',rank(ctrb(r.A,r.B)));
fprintf('CARE residual Frobenius norm:            %.3e\n',r.care_residual_norm);
fprintf('max real open-loop pole:                 %+.8g\n',max(real(r.open_loop_poles)));
fprintf('max real closed-loop pole:               %+.8g\n',max(real(r.closed_loop_poles)));
fprintf('closed-loop asymptotically stable:       %d\n',all(real(r.closed_loop_poles)<0));
fprintf('theta_b*=%.3e rad; Twl_l*-Twl_r*=%.3e N*m\n',e.X(9),e.U(1)-e.U(2));
fprintf('=====================================================\n');
end
