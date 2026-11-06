%% 腿部气弹簧 F0 补偿：受约束平滑趋势拟合与固件参数导出
% 测量值包含静摩擦、端点接触和人工读数误差，不能逐点硬插值。
% 四阶归一化最小二乘拟合保留整体趋势：中段约 -35 N、两端逐渐变弱。
% 380 mm 的 +2 N 相比 370 mm 的 -35 N 跳变 37 N，被作为端点异常趋势点；
% 不强制穿过，避免向固件引入突变曲率。
%
% 140 mm 必须严格保留实测 -28 N。130~140 mm 用 Hermite 曲线平滑接入，
% 130 mm 及以下才输出 0 N；这样 140 mm 附近仍有阻止继续收短的拉力，
% 同时避免跨过 140 mm 时 F0 跳变。
% 高端从 380 mm 起保持 380 mm 的拟合值，避免补偿突然归零；持续的负补偿
% 仍提供收腿方向的回拉趋势。

clear;
clc;

length_mm = (140:10:380).';
force_measured_n = [ ...
   -28; -30; -30; -34; -35; -35; -35; -37; -37; -37; ...
   -37; -37; -37; -37; -37; -36; -36; -36; -35; -35; ...
   -35; -35; -35; -35;   2];

% 测试固件使用受 140 mm 实测值约束的连续多项式。
% 若需微调，先改上方实测数组再运行。
fit_degree = 4;

n = numel(length_mm);
length_center_mm = mean([length_mm(1), length_mm(end)]);
length_half_span_mm = (length_mm(end) - length_mm(1)) / 2;
z = (length_mm - length_center_mm) / length_half_span_mm;

% 380 mm 是孤立跳变，降低其趋势权重；140 mm 则强制严格通过实测 -28 N。
weights = ones(n, 1);
weights(end) = 0.10;
A = zeros(n, fit_degree + 1);
for col = 1:(fit_degree + 1)
    A(:, col) = z .^ (fit_degree - col + 1);
end
constraint_row = z(1) .^ (fit_degree:-1:0);
kkt_matrix = [2 * A' * diag(weights .^ 2) * A, constraint_row'; ...
              constraint_row,                              0];
kkt_rhs = [2 * A' * diag(weights .^ 2) * force_measured_n; -28];
kkt_solution = kkt_matrix \ kkt_rhs;
poly_coeff = kkt_solution(1:end-1).';
force_fit_n = polyval(poly_coeff, z);

length_plot_mm = (130:1:410).';
clamped_plot_mm = min(max(length_plot_mm, 140), 380);
z_plot = (clamped_plot_mm - length_center_mm) / length_half_span_mm;
force_plot_n = polyval(poly_coeff, z_plot);
low_transition = length_plot_mm > 130 & length_plot_mm < 140;
t = (length_plot_mm(low_transition) - 130) / 10;
slope_at_140_n_per_m = polyval(polyder(poly_coeff), z(1)) / (length_half_span_mm / 1000);
force_plot_n(low_transition) = -28 * (-2 * t.^3 + 3 * t.^2) + ...
    (0.010 * slope_at_140_n_per_m) .* (t.^3 - t.^2);
force_plot_n(length_plot_mm <= 130) = 0;

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
fprintf('低端：L0 <= 0.130 m 输出 0 N；130~140 mm 平滑过渡到实测 -28 N；高端：L0 > 0.380 m 保持 %.3f N。\n', force_fit_n(end));
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
xline(130, '--r', '130 mm zero boundary');
xline(140, '--r', '140 mm measured -28 N');
xline(380, '--r', '380 mm active boundary');
xlim([130, 410]);
