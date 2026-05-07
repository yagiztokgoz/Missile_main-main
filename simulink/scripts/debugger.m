% To Workspace bloklarından gelen verileri tablo olarak göster
% Sinyal isimlerini kendi workspace değişken adlarınla güncelle



t_vec     = out.tout;
dvbe_vec  = squeeze(out.dvbe.Data);      % dvbe To Workspace adı
hbe_vec   = squeeze(out.hbe.Data);     % hbe To Workspace adı
alphax_vec = squeeze(out.alphax.Data);  % alphax To Workspace adı
dqxc_vec   = squeeze(out.dqcx.Data);    % dqx To Workspace adı
ancomx_vec = squeeze(out.ancomx.Data); % ancomx To Workspace adı
dmd_vec = squeeze(out.dmd.Data);
clmdq_vec = squeeze(out.clmdq.Data);
ai33_vec = squeeze(out.ai33.Data);
ai11_vec = squeeze(out.ai11.Data);
Fm_vec = squeeze(out.Fm.Data);
xcg_vec = squeeze(out.xcg.Data);
dqx_vec = squeeze(out.dqx.Data);
qqx_vec = squeeze(out.qqx.Data);
ppx_vec = squeeze(out.ppx.Data);

% Belirli zaman noktalarında değerleri göster
t_check = [1.0, 2.0, 3.0, 4.0, 5.0];
fprintf('%6s %10s %12s %10s %10s %10s\n', 't', 'dvbe', 'hbe', 'alphax', 'dqx', 'ancomx');
for tc = t_check
    [~, idx] = min(abs(t_vec - tc));
    fprintf('%6.1f %10.3f %12.3f %10.4f %10.4f %10.4f\n', ...
        t_vec(idx), dvbe_vec(idx), hbe_vec(idx), ...
        alphax_vec(idx), dqxc_vec(idx), ancomx_vec(idx));
end


[~,idx] = min(abs(t_vec - 1.0));
fprintf('dmd_sim = %.4f\n', dmd_vec(idx));
fprintf('clmdq_sim = %.4f\n', clmdq_vec(idx));

[~,idx] = min(abs(t_vec - 1.0));
fprintf('qqx_sim = %.4f deg/s\n', qqx_vec(idx));

[~,idx] = min(abs(t_vec - 1.0));
fprintf('ai33_sim = %.4f\n', ai33_vec(idx));
fprintf('ai11_sim = %.4f\n', ai11_vec(idx));
fprintf('Fm_sim = %.4d\n', Fm_vec(idx));

[~,idx] = min(abs(t_vec - 1.0));
fprintf('xcg_sim = %.4f m\n',xcg_vec(idx));
fprintf('dqx = %.4f m\n',dqx_vec(idx));
fprintf('ppx = %.4f \n',ppx_vec(idx));


%% Roll Debug Plot
t = out.SBEL.time;

phiblx = squeeze(out.phiblx.data);
dpcx   = squeeze(out.dpcx.data);
dpx    = squeeze(out.dpx.data);

figure('Name','Roll Debug');
subplot(3,1,1);
plot(t, phiblx, 'b-', 'LineWidth', 1.5);
yline(0, 'r--'); ylabel('Roll angle (deg)'); grid on;
title('phiblx — 0 etrafında kalmalı');

subplot(3,1,2);
plot(t, dpcx, 'g-', 'LineWidth', 1.5);
ylabel('dpcx (deg)'); grid on;
title('Roll fin komutu');

subplot(3,1,3);
plot(t, dpx, 'm-', 'LineWidth', 1.5);
ylabel('dpx (deg)'); grid on;
title('Gerçek roll fin açısı');
xlabel('Time (s)');

fprintf('Max |phiblx| = %.2f deg at t=%.2f s\n', ...
    max(abs(phiblx)), t(find(abs(phiblx)==max(abs(phiblx)),1)));
