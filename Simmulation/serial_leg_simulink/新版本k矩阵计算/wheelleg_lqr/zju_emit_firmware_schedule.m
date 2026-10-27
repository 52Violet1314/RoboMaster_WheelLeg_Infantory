function zju_emit_firmware_schedule(Qdiag, Rdiag)
%ZJU_EMIT_FIRMWARE_SCHEDULE Print the validated schedule as a C initializer.
% Copying is deliberately not manual: app_K_value.cpp is generated from this
% exact output after zju_firmware_schedule() has passed its checks.

if nargin < 1, Qdiag = []; end
if nargin < 2, Rdiag = []; end
schedule = zju_firmware_schedule(Qdiag, Rdiag, false);

fprintf('BEGIN_LQR_FIRMWARE_TABLE\n');
for index = 1:numel(schedule.length_m)
    fprintf('    {%.3ff, {\n', schedule.length_m(index));
    for row = 1:4
        fprintf('        {');
        for column = 1:10
            if column > 1, fprintf(', '); end
            fprintf('%+.9ff', schedule.gain(index,row,column));
        end
        fprintf('}%s\n', ternary(row < 4, ',', ''));
    end
    fprintf('    }}%s\n', ternary(index < numel(schedule.length_m), ',', ''));
end
fprintf('END_LQR_FIRMWARE_TABLE\n');
fprintf('BEGIN_LEG_ANGLE_TABLE\n');
for index = 1:numel(schedule.length_m)
    fprintf('    {%.3ff, %+.9ff}%s\n', schedule.length_m(index), ...
        schedule.theta_ref_rad(index), ternary(index < numel(schedule.length_m), ',', ''));
end
fprintf('END_LEG_ANGLE_TABLE\n');
end

function value = ternary(condition, true_value, false_value)
if condition
    value = true_value;
else
    value = false_value;
end
end
