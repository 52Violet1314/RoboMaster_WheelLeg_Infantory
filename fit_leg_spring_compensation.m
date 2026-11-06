%% 腿部气弹簧 F0 补偿：平滑趋势拟合与固件参数导出
% 测量值包含静摩擦、端点接触和人工读数误差，不能逐点硬插值。
% 三阶归一化最小二乘拟合保留整体趋势：中段约 -35 N、两端逐渐变弱。
% 380 mm 的 +2 N 相比 370 mm 的 -35 N 跳变 37 N，被作为端点异常趋势点；
% 不强制穿过，避免向固件引入突变曲率。
%
% 有效区间严格为 140~380 mm；固件对 L0 < 140 mm 或 L0 > 380 mm 返回 0 N。
% 零补偿不是主动阻力命令，而是撤掉弹簧补偿，使机构本身与 PID 阻止继续越界。

clear;
clc;

length_mm = (140:10:380).';
force_measured_n = [ ...
   -28; -30; -30; -34; -35; -35; -35; -37; -37; -37; ...
   -37; -37; -37; -37; -37; -36; -36; -36; -35; -35; ...
   -35; -35; -35; -35;   2];

% 测试固件使用 degree 阶连续多项式。若需微调，先改上方实测数组再运行。
fit_degree = 3;

n = numel(length_mm);
length_center_mm = mean([length_mm(1), length_mm(end)]);
length_half_span_mm = (length_mm(end) - length_mm(1)) / 2;
z = (length_mm - length_center_mm) / length_half_span_mm;

% 不强制端点穿过：允许拟合抑制 380 mm 的孤立跳变。
poly_coeff = polyfit(z, force_measured_n, fit_degree);
force_fit_n = polyval(poly_coeff, z);

length_plot_mm = (130:1:390).';
z_plot = (min(max(length_plot_mm, 140), 380) - length_center_mm) / length_half_span_mm;
force_plot_n = polyval(poly_coeff, z_plot);
force_plot_n(length_plot_mm < 140 | length_plot_mm > 380) = 0;

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
fprintf('区间外：L0 < 0.140 m 或 L0 > 0.380 m，输出 0 N。\n');
fprintf('\n固件函数系数（z=(L0_mm-260)/120，按降幂）：\n');
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
title('Gas spring compensation: smooth trend fit');
legend('Location', 'best');
xline(140, '--r', '140 mm active boundary');
xline(380, '--r', '380 mm active boundary');
xlim([130, 390]);
