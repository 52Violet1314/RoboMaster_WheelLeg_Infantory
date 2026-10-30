function result = zju_firmware_schedule(Qdiag, Rdiag, print_report)
%ZJU_FIRMWARE_SCHEDULE Build the 41-point C-firmware gain/angle schedule.
% The schedule uses the physical parameters in zju_parameters_template.m.
% It uses the current vehicle Q/R defaults unless values are supplied.
%
% result.gain(n,:,:) is the gain in the exact sign convention consumed by
% CalculateTask.cpp. result.theta_ref_rad(n) is the per-leg target in the
% real-firmware theta_L/theta_R convention. Both use 0.150:0.005:0.350 m.

if nargin < 1 || isempty(Qdiag)
    % [X, dX, yaw, dyaw, thetaL, dthetaL, thetaR, dthetaR, pitch, dpitch]
    % 当前调参：X=40，dX=4，dyaw=2，thetaL/R=35000，dthetaL/R=9，dpitch=4。
    Qdiag = [40 4 400 2 35000 9 35000 9 40000 4];
end
if nargin < 2 || isempty(Rdiag)
    Rdiag = [10 10 1 1];
end
if nargin < 3 || isempty(print_report)
    print_report = false;
end

lengths = 0.150:0.005:0.350;
count = numel(lengths);
gain = zeros(count, 4, 10);
theta_ref_rad = zeros(count, 1);
equilibrium_u_nm = zeros(count, 4);
care_residual = zeros(count, 1);
max_closed_loop_real = zeros(count, 1);

% MATLAB's K has U=-K*dX.  This element sign map is the existing,
% validated MATLAB-to-firmware conversion: it reproduces the deployed
% 250 mm table when the old 250 mm model is supplied.
firmware_sign = [ ...
    -1 -1 -1 -1  1  1  1  1  1  1; ...
    -1 -1 -1 -1  1  1  1  1  1  1; ...
    -1 -1 -1 -1  1  1  1 -1  1  1; ...
    -1 -1 -1 -1  1 -1  1  1  1  1];

for index = 1:count
    l0 = lengths(index);
    lqr_result = zju_lqr_gain(Qdiag, Rdiag, l0);
    assert(all(isfinite(lqr_result.K), 'all'), 'K contains NaN/Inf at L0=%.3f m.', l0);
    assert(all(real(lqr_result.closed_loop_poles) < 0), ...
        'Closed loop is unstable at L0=%.3f m.', l0);

    gain(index,:,:) = firmware_sign .* lqr_result.K;
    % theta_l is the model's wheel-to-body pendulum angle.  Firmware
    % theta_L/theta_R is the hip-to-wheel angle, so the same physical pose
    % has the opposite small-angle sign.  Export the actual C target here.
    theta_ref_rad(index) = -lqr_result.equilibrium.X(5);
    equilibrium_u_nm(index,:) = lqr_result.equilibrium.U.';
    care_residual(index) = lqr_result.care_residual_norm;
    max_closed_loop_real(index) = max(real(lqr_result.closed_loop_poles));
end

result = struct( ...
    'length_m', lengths(:), 'gain', gain, 'theta_ref_rad', theta_ref_rad, ...
    'equilibrium_u_nm', equilibrium_u_nm, ...
    'care_residual', care_residual, ...
    'max_closed_loop_real', max_closed_loop_real, ...
    'Qdiag', Qdiag(:), 'Rdiag', Rdiag(:), ...
    'firmware_sign', firmware_sign);

if print_report
    current = deployedFirmwareReference();
    reference_index = [1 21 41];
    cross_coupling = false(4,10);
    cross_coupling(3,8) = true;
    cross_coupling(4,6) = true;
    gain_ratios = [];
    polarity_mismatch = 0;
    for index = 1:numel(reference_index)
        k = reference_index(index);
        scheduled_gain = squeeze(gain(k,:,:));
        reference_gain = current(:,:,index);
        local_ratio = abs(scheduled_gain(:) ./ reference_gain(:));
        gain_ratios = [gain_ratios; local_ratio]; %#ok<AGROW>
        [mismatch_row,mismatch_column] = find(sign(scheduled_gain) ~= sign(reference_gain));
        polarity_mismatch = polarity_mismatch + numel(mismatch_row);
        fprintf('L0=%.3f MAG_RATIO_MIN=%.6f MAG_RATIO_MAX=%.6f\n', ...
            lengths(k), min(local_ratio), max(local_ratio));
        non_cross_ratio = abs(scheduled_gain(~cross_coupling) ./ ...
                              reference_gain(~cross_coupling));
        fprintf(['L0=%.3f EXCLUDING_CROSS_DTHETA ' ...
                 'MAG_RATIO_MIN=%.6f MAG_RATIO_MAX=%.6f\n'], ...
            lengths(k), min(non_cross_ratio), max(non_cross_ratio));
        for mismatch_index = 1:numel(mismatch_row)
            row = mismatch_row(mismatch_index);
            column = mismatch_column(mismatch_index);
            fprintf(['POLARITY_MISMATCH L0=%.3f row=%d column=%d ' ...
                     'new=%+.9f deployed=%+.9f\n'], ...
                lengths(k), row, column, scheduled_gain(row,column), ...
                reference_gain(row,column));
        end
    end
    fprintf('COUNT=%d\n', count);
    fprintf('THETA_RAD_MIN=%.9f THETA_RAD_MAX=%.9f\n', ...
        min(theta_ref_rad), max(theta_ref_rad));
    fprintf('THETA_DEG_MIN=%.6f THETA_DEG_MAX=%.6f\n', ...
        min(theta_ref_rad)*180/pi, max(theta_ref_rad)*180/pi);
    fprintf('STATIC_U_NM_MIN=%.9f STATIC_U_NM_MAX=%.9f\n', ...
        min(equilibrium_u_nm, [], 'all'), max(equilibrium_u_nm, [], 'all'));
    fprintf('CARE_RESIDUAL_MAX=%.3e\n', max(care_residual));
    fprintf('CLOSED_LOOP_MAX_REAL=%.9f\n', max(max_closed_loop_real));
    fprintf('POLARITY_MISMATCH_VS_DEPLOYED=%d\n', polarity_mismatch);
    fprintf('MAG_RATIO_MIN=%.6f MAG_RATIO_MAX=%.6f MAG_RATIO_MEDIAN=%.6f\n', ...
        min(gain_ratios), max(gain_ratios), median(gain_ratios));
end
end

function current = deployedFirmwareReference()
% Actual 150/250/350 mm tables presently deployed in app_K_value.cpp.
current = zeros(4,10,3);
current(:,:,1) = [ ...
 .6243 2.0785 3.1106 .4550 -35.1025 -.9911 -2.8527 -.2108 -32.4029 -2.1963; ...
 .6243 2.0785 -3.1106 -.4550 -2.8527 -.2108 -35.1025 -.9911 -32.4029 -2.1963; ...
 -2.4704 -8.1973 10.1607 1.5178 102.7465 3.0402 -8.1611 .3434 -115.8194 -5.6326; ...
 -2.4704 -8.1973 -10.1607 -1.5178 -8.1611 .3434 102.7465 3.0402 -115.8194 -5.6326];
current(:,:,2) = [ ...
 .7796 2.8815 2.4198 .3817 -53.1388 -1.5022 -3.0942 -.4155 -21.9072 -1.5123; ...
 .7796 2.8815 -2.4198 -.3817 -3.0942 -.4155 -53.1388 -1.5022 -21.9072 -1.5123; ...
 -1.9804 -7.2994 11.8932 1.9557 108.1340 3.2626 -13.9804 -.3701 -119.9538 -6.0837; ...
 -1.9804 -7.2994 -11.8932 -1.9557 -13.9804 -.3701 108.1340 3.2626 -119.9538 -6.0837];
current(:,:,3) = [ ...
 .8581 3.2354 1.8774 .3114 -60.7707 -1.9593 -2.7918 -.6050 -14.8798 -1.1022; ...
 .8581 3.2354 -1.8774 -.3114 -2.7918 -.6050 -60.7707 -1.9593 -14.8798 -1.1022; ...
 -1.6236 -6.1025 12.8356 2.3073 96.1203 3.4236 -17.0960 -.2642 -110.1738 -5.9770; ...
 -1.6236 -6.1025 -12.8356 -2.3073 -17.0960 -.2642 96.1203 3.4236 -110.1738 -5.9770];
end
