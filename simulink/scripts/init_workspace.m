%% SRAAM6 Simulink Workspace Initialiser
%  Run this script before opening any Simulink model.
%  It loads all lookup tables and parameters into the base workspace.
%
%  Usage:
%    >> cd <project_root>/simulink
%    >> run('scripts/init_workspace.m')

clear; clc;
fprintf('=== SRAAM6 Simulink Init ===\n');

% ── paths ──────────────────────────────────────────────────────────────────────
SCRIPT_DIR = fileparts(mfilename('fullpath'));
DATA_DIR   = fullfile(SCRIPT_DIR, '..', 'data');

% ── load tables ────────────────────────────────────────────────────────────────
fprintf('Loading lookup tables...\n');
load(fullfile(DATA_DIR, 'sraam6_tables.mat'));
load(fullfile(DATA_DIR, 'sraam6_config.mat'));
fprintf('  ✓ Tables and config loaded.\n');

% ── verify aero tables ────────────────────────────────────────────────────────
aero_required = {'mach_grid', 'alpha_grid', 'beta_grid'};
for i = 1:numel(aero_required)
    if ~exist(aero_required{i}, 'var')
        error('Missing aero variable: %s — re-run export_tables.py', aero_required{i});
    end
end
fprintf('  ✓ Aero grid axes present  (Mach:%d  Alpha:%d  Beta:%d)\n', ...
        numel(mach_grid), numel(alpha_grid), numel(beta_grid));

% ── verify propulsion tables ───────────────────────────────────────────────────
prop_required = {'prop_thrust_t', 'prop_thrust_v', ...
                 'prop_mass_t',   'prop_mass_v',   ...
                 'prop_cg_t',     'prop_cg_v'};
for i = 1:numel(prop_required)
    if ~exist(prop_required{i}, 'var')
        error('Missing prop variable: %s — re-run export_tables.py', prop_required{i});
    end
end
fprintf('  ✓ Propulsion tables present\n');
fprintf('    thrust : t=%.2f..%.2f s   T=%.0f..%.0f N\n', ...
        prop_thrust_t(1), prop_thrust_t(end), ...
        prop_thrust_v(1), prop_thrust_v(end));
fprintf('    mass   : %.2f → %.2f kg\n', prop_mass_v(1), prop_mass_v(end));
fprintf('    cg     : %.4f → %.4f m\n', prop_cg_v(1), prop_cg_v(end));

% ── verify config ─────────────────────────────────────────────────────────────
cfg_required = {'mass', 'ai33', 'wnact', 'wn_ndi_q', 'dt'};
for i = 1:numel(cfg_required)
    if ~exist(cfg_required{i}, 'var')
        error('Missing config variable: %s — re-run export_tables.py', cfg_required{i});
    end
end

% INS error parameters (drawn once)

mins = 0;

rng(42);   % tekrarlanabilirlik için seed
EBIASG = randn(3,1) * 3.2e-6;
ESCALG = randn(3,1) * 2.5e-5;
EMISG  = randn(3,1) * 1.1e-4;
EWALKG = ones(3,1)  * 1.0e-6;
EUNBG  = zeros(3,1);
EBIASA = randn(3,1) * 3.56e-3;
ESCALA = randn(3,1) * 5.0e-4;
EMISA  = randn(3,1) * 1.1e-4;
EWALKA = ones(3,1)  * 1.0e-4;
biasal = 0.0;
randal = 0.0;


% ── derived constants ─────────────────────────────────────────────────────────
RAD = pi / 180;
DEG = 180 / pi;

fprintf('\nMissile parameters:\n');
fprintf('  mass   = %.2f kg\n',  mass);
fprintf('  ai33   = %.2f kg*m2\n', ai33);
fprintf('  xcg    = %.4f m\n', xcg);
fprintf('  wnact  = %.1f rad/s\n', wnact);
fprintf('  wn_ndi = %.1f / %.1f rad/s  (q/r)\n', wn_ndi_q, wn_ndi_r);
fprintf('  alimit = %.1f g\n', alimit);
fprintf('  dt     = %.4f s  (%.0f Hz)\n', dt, 1/dt);

fprintf('\n✓ Workspace ready.  Open models/sraam6_engagement.slx\n');
