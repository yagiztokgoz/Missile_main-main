%% Trajectory Plot — SRAAM6 Simulink
% To Workspace blokları "Array" formatında varsayılıyor.
% Eğer "Structure with time" ise: SBEL = SBEL.signals.values; kullan.
SBEL = out.SBEL;
STEL = out.STEL;
% --- Veri al ---
t    = SBEL.time;
sbel = SBEL.data;   % Nx3: [North, East, Down]
stel = squeeze(STEL.data)';   % Nx3: [North, East, Down]

% Altitude = -Down
mis_alt = -sbel(:,3);
tgt_alt = -stel(:,3);

% Miss distance
dr = sbel - stel;
miss = sqrt(sum(dr.^2, 2));
[min_miss, idx] = min(miss);
fprintf('Miss distance: %.2f m at t=%.3f s\n', min_miss, t(idx));

ancomx = out.ancomx.data;
dqcx = squeeze(out.dqcx.data);
figure;
subplot(2,1,1); plot(t, ancomx); ylabel('ancomx (g)'); grid on;
subplot(2,1,2); plot(t, dqcx); ylabel('dqcx (deg)'); grid on;


%% --- 3D Trajectory ---
figure('Name','3D Trajectory');
plot3(sbel(:,1), sbel(:,2), mis_alt, 'b-', 'LineWidth', 1.5); hold on;
plot3(stel(:,1), stel(:,2), tgt_alt, 'r--', 'LineWidth', 1.5);
plot3(sbel(1,1), sbel(1,2), mis_alt(1), 'bs', 'MarkerSize', 8, 'MarkerFaceColor','b');
plot3(stel(1,1), stel(1,2), tgt_alt(1), 'rs', 'MarkerSize', 8, 'MarkerFaceColor','r');
plot3(sbel(idx,1), sbel(idx,2), mis_alt(idx), 'kx', 'MarkerSize', 12, 'LineWidth', 2);
xlabel('North (m)'); ylabel('East (m)'); zlabel('Altitude (m)');
legend('Missile','Target','Missile IC','Target IC','CPA'); grid on;
title(sprintf('3D Trajectory — Miss: %.2f m', min_miss));

% İrtifa karşılaştırması
figure;
plot(t, -sbel(:,3), 'b', t, -stel(:,3), 'r--', 'LineWidth', 1.5);
xlabel('t(s)'); ylabel('Altitude (m)');
legend('Missile','Target'); grid on;
title('Altitude Comparison');



%% --- Miss Distance ---
figure('Name','Miss Distance');
plot(t, miss, 'k-', 'LineWidth', 1.5);
xline(t(idx), 'r--', sprintf('CPA=%.2fm', min_miss));
xlabel('Time (s)'); ylabel('Miss Distance (m)'); grid on;
title('Range to Target');

figure;
subplot(2,2,1);
plot(t, sbel(:,1), 'b', t, stel(:,1), 'r--');
xlabel('t(s)'); ylabel('North (m)'); legend('Missile','Target'); grid on;

subplot(2,2,2);
plot(t, sbel(:,2), 'b', t, stel(:,2), 'r--');
xlabel('t(s)'); ylabel('East (m)'); grid on;

subplot(2,2,3);
plot(t, -sbel(:,3), 'b', t, -stel(:,3), 'r--');
xlabel('t(s)'); ylabel('Altitude (m)'); grid on;

subplot(2,2,4);
plot(t, miss, 'k');
xlabel('t(s)'); ylabel('Miss distance (m)'); grid on;
