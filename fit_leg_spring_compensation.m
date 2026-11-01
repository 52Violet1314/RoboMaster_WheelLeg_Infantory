%% 气弹簧 F0 补偿表连续函数拟合（仅分析，不写入固件）
% 正式版使用原表：130~350 mm 每 10 mm 一点，350~384 mm 固定使用 350 mm 值。
% 测试版使用本文件生成的连续拟合函数。
% 修改 force_measured_n 后直接运行本文件；命令行会输出可复制到 C++ 的表项。

clear;
clc;

length_mm = (130:10:350).';
force_measured_n = [ ...
     0.0;  -6.0; -10.0; -12.0; -11.0; -10.0; -10.0; -10.0; ...
   -11.0; -12.0; -13.0; -14.0; -14.0; -14.0; -14.0; -14.0; ...
   -13.5; -13.0; -13.0; -13.0; -13.0; -12.5; -12.0];

% 测试固件使用 degree 阶连续多项式。若需微调，先改上方实测数组再运行。
fit_degree = 6;

n = numel(length_mm);
length_center_mm = mean([length_mm(1), length_mm(end)]);
length_half_span_mm = (length_mm(end) - length_mm(1)) / 2;
z = (length_mm - length_center_mm) / length_half_span_mm;

% 先最小二乘拟合，再加一条线性修正，严格固定两端测量值。
poly_coeff = polyfit(z, force_measured_n, fit_degree);
endpoint_error_left = force_measured_n(1) - polyval(poly_coeff, -1.0);
endpoint_error_right = force_measured_n(end) - polyval(poly_coeff, 1.0);
linear_offset = (endpoint_error_left + endpoint_error_right) / 2;
linear_slope = (endpoint_error_right - endpoint_error_left) / 2;
poly_coeff(end - 1) = poly_coeff(end - 1) + linear_slope;
poly_coeff(end) = poly_coeff(end) + linear_offset;
force_fit_n = polyval(poly_coeff, z);

% 350~384 mm 固定使用最后一个表项；此处仅用于图示。
length_plot_mm = (130:1:384).';
z_plot = (min(length_plot_mm, 350) - length_center_mm) / length_half_span_mm;
force_plot_n = polyval(poly_coeff, z_plot);

result_table = table(length_mm, force_measured_n, force_fit_n, ...
    'VariableNames', {'Length_mm', 'Measured_N', 'Fitted_N'});
disp(result_table);

fprintf('\n可复制到 C++ 的拟合表：\n');
for k = 1:n
    fprintf('{%.3ff, %.2ff}', length_mm(k) / 1000, force_fit_n(k));
    if k < n
        fprintf(', ');
    end
    if mod(k, 4) == 0 || k == n
        fprintf('\n');
    end
end
fprintf('350~384 mm：固定 %.2f N\n', force_fit_n(end));
fprintf('\n测试固件函数系数（z=(L0_mm-240)/110，按降幂）：\n');
fprintf('%.9gf ', poly_coeff);
fprintf('\n');

figure('Name', 'Leg spring F0 compensation fit', 'Color', 'w');
plot(length_mm, force_measured_n, 'ko', 'LineWidth', 1.2, ...
     'MarkerFaceColor', 'k', 'DisplayName', 'Current table');
hold on;
plot(length_plot_mm, force_plot_n, 'b-', 'LineWidth', 2.0, ...
     'DisplayName', sprintf('Continuous polynomial fit (degree = %d)', fit_degree));
grid on;
xlabel('Leg length L0 (mm)');
ylabel('F0 compensation (N; negative = pull leg)');
title('Gas spring compensation: measured table vs. smooth fit');
legend('Location', 'best');
xlim([130, 384]);
