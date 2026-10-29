function p = zju_parameters_template(leg_length)
%ZJU_PARAMETERS_TEMPLATE Current vehicle equivalent straight-leg parameters.
% Values come directly from D:\code\chuanlian_leg\get_K_jiao_LQR.m.
% l_c and phi_c describe the vector from the hip midpoint to the body CoM
% in the sagittal plane.  At theta_b=0: dy_forward=l_c*sin(phi_c),
% dz_up=l_c*cos(phi_c).  The current vehicle setting is 8 mm rearward,
% with no vertical or lateral offset.
%
% Body axes intentionally match the old K script and firmware: x=right,
% y=forward, z=up; +phi is a left yaw about +z.  In the 2-D leg
% calculation x_leg=back=-y_body and y_leg=down=-z_body.  theta_l is the
% model's wheel-to-body pendulum angle; firmware theta_L/theta_R is the
% hip-to-wheel angle.  Their small-angle signs are therefore opposite even
% though the body axes are identical; zju_firmware_schedule exports -theta_l.

if nargin < 1 || isempty(leg_length), leg_length = 0.20; end
validateattributes(leg_length, {'numeric'}, {'real','scalar','finite','positive'});

g = 9.81; R_w = 0.0525; R_l = 0.20;
m_w = 0.30; m_b = 15.75;
com_forward_m = -0.008; % +body y: 8 mm behind hip midpoint
com_up_m = 0.000;       % +body z: no vertical offset
l_c = hypot(com_forward_m, com_up_m);
phi_c = atan2(com_forward_m, com_up_m);
I_w = 0.5*m_w*R_w^2;
I_b = m_b*(0.485^2 + 0.152^2)/12.0;
I_zz = 0.226;

% Same uniform equivalent-leg assumptions as old get_K_jiao_LQR.m.
m_l = 0.80; m_r = 0.80;
l_l = leg_length; l_r = leg_length;
l_w_l = leg_length/2; l_w_r = leg_length/2;
l_b_l = leg_length/2; l_b_r = leg_length/2;
I_l = m_l*(leg_length^2 + 0.048^2)/12.0;
I_r = m_r*(leg_length^2 + 0.048^2)/12.0;

p = struct('g',g,'R_w',R_w,'R_l',R_l, ...
    'm_b',m_b,'I_b',I_b,'I_zz',I_zz,'l_c',l_c,'phi_c',phi_c, ...
    'm_w',m_w,'I_w',I_w,'m_l',m_l,'m_r',m_r,'I_l',I_l,'I_r',I_r, ...
    'l_l',l_l,'l_r',l_r,'l_w_l',l_w_l,'l_w_r',l_w_r, ...
    'l_b_l',l_b_l,'l_b_r',l_b_r, ...
    'Ts',0.002,'linearization_step',1e-6, ...
    'coordinate_convention',struct( ...
        'body_axes','+x right, +y forward, +z up', ...
        'yaw_positive','left turn about +z', ...
        'leg_plane_axes','+x backward, +y downward', ...
        'leg_angle_positive','forward swing', ...
        'state_input_order','same order as get_K_jiao_LQR.m'));
end
