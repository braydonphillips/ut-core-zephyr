%% AirBearingCalibration — gyro / accel / mag bias & alignment for mainAirBearing.cpp
%
% Firmware must be built with -DCAL_EMIT=ON so the serial stream is the
% per-cycle CSV format below (no [ADCS] telemetry):
%
%     IMUCSV,t_ms,sensor,ax,ay,az,gx,gy,gz       % LSM6DSV (sensor 0,1)
%     GYRCSV,t_ms,sensor,gx,gy,gz                % I3G4250D (sensor 0,1)
%     MAGCSV,t_ms,sensor,addr,mx,my,mz,temp,0    % MLX90393 (sensor 0,1,2)
%
% Build:
%     west build -p always -b ut_core ut-core\app\adcsComputeTest -- ^
%                -DAIR_BEARING=ON -DCAL_EMIT=ON
%
% Usage:
%   AirBearingCalibration('gyro',  "COM5")
%       Hold satellite stationary. Collects N samples (default 500),
%       averages each gyro chip per axis, prints biases ready to paste
%       into GYRO_BIAS_* in mainAirBearing.cpp.
%
%   AirBearingCalibration('accel', "COM5", 'gravity', [0 0 -9.80665])
%       Hold satellite still in known orientation. Provide expected
%       gravity vector in raw sensor frame. Prints ACCEL_BIAS_* values.
%
%   AirBearingCalibration('mag',   "COM5", 'freq', 0.5, 'duration', 30)
%       Hold satellite aligned with helmholtz coil frame. For each coil
%       axis (X, then Y, then Z) drive a sinusoid at `freq` Hz; press
%       Enter when the drive is on, the script captures for `duration`
%       seconds. After all 3 axes, sin/cos demodulation builds a 3x3
%       alignment matrix per magnetometer.
%
%   AirBearingCalibration('magview', "COM5")
%   AirBearingCalibration('magview', "COM5", 'history', 1500, 'apply_cal', true)
%       Live viewer for all 3 magnetometers. Time-series subplots for
%       Bx/By/Bz per sensor plus a 3D scatter cloud. Default shows raw
%       LSB; pass apply_cal=true to display calibrated field in uT
%       (uses MAG_BIAS / MAG_CAL_SCALE constants in this script — keep
%       them in sync with mainAirBearing.cpp). Close the figure to stop.
%
% After collection paste the printed constants into the "CALIBRATION
% CONSTANTS" block at the top of mainAirBearing.cpp, rebuild without
% -DCAL_EMIT.

function AirBearingCalibration(mode, port, varargin)
    p = inputParser;
    addRequired(p, 'mode', @(x) any(strcmpi(x, {'gyro','accel','mag','magview'})));
    addRequired(p, 'port', @(x) ischar(x) || isstring(x));
    addParameter(p, 'samples',  500, @(x) isnumeric(x) && x > 0);
    addParameter(p, 'gravity',  [0 0 -9.80665], @(x) isnumeric(x) && numel(x) == 3);
    addParameter(p, 'freq',     0.5, @(x) isnumeric(x) && x > 0);
    addParameter(p, 'duration', 30,  @(x) isnumeric(x) && x > 0);
    addParameter(p, 'history',  600, @(x) isnumeric(x) && x > 1);
    addParameter(p, 'apply_cal', false, @islogical);
    addParameter(p, 'baud',     115200, @isnumeric);
    parse(p, mode, port, varargin{:});
    R = p.Results;

    sp = serialport(R.port, R.baud, "Timeout", 5);
    configureTerminator(sp, "LF");
    flush(sp);
    cleanupObj = onCleanup(@() delete(sp)); %#ok<NASGU>
    fprintf("Connected to %s at %d baud.\n", R.port, R.baud);

    switch lower(R.mode)
        case 'gyro',    runGyro(sp, R.samples);
        case 'accel',   runAccel(sp, R.samples, R.gravity);
        case 'mag',     runMag(sp, R.freq, R.duration);
        case 'magview', runMagView(sp, R.history, R.apply_cal);
    end
end

% =====================================================================
% Sensor scaling — must match constants in mainAirBearing.cpp
% =====================================================================
function k = gyroLsbToRadps()
    k = 8.75e-3 * (pi / 180);   % matches GYRO_LSB_RADPS
end

function k = accelLsbToMps2()
    k = 0.061e-3 * 9.80665;     % matches ACCEL_SCALE
end

function corr = gyroCorr(name)
    % Mirrors GYRO_CORR_* in mainAirBearing.cpp
    switch name
        case 'LSM0', corr = [0.5144, 0.4848, 0.5086];
        case 'LSM1', corr = [0.5778, 0.4523, 0.4436];
        case 'I3G0', corr = [1.0163, 1.0045, 1.0502];
        case 'I3G1', corr = [1.0915, 1.0303, 0.9863];
    end
end

% =====================================================================
% Gyro bias
% =====================================================================
function runGyro(sp, N)
    fprintf("\n[GYRO] Hold satellite stationary. Collecting %d samples per sensor...\n", N);
    raw = struct('LSM0', [], 'LSM1', [], 'I3G0', [], 'I3G1', []);
    counts = struct('LSM0', 0, 'LSM1', 0, 'I3G0', 0, 'I3G1', 0);
    target = N;
    last_report = tic;

    while min(structfun(@(v) v, counts)) < target
        line = readlineSafe(sp);
        if isempty(line), continue; end
        [tag, vals] = parseCsv(line);
        switch tag
            case 'IMU'
                idx = vals(1); g = vals(5:7);
                key = sprintf('LSM%d', idx);
                if counts.(key) < target
                    raw.(key)(end+1, :) = g;
                    counts.(key) = counts.(key) + 1;
                end
            case 'GYR'
                idx = vals(1); g = vals(2:4);
                key = sprintf('I3G%d', idx);
                if counts.(key) < target
                    raw.(key)(end+1, :) = g;
                    counts.(key) = counts.(key) + 1;
                end
        end
        if toc(last_report) > 1.0
            fprintf("  LSM0=%d LSM1=%d I3G0=%d I3G1=%d / %d\n", ...
                counts.LSM0, counts.LSM1, counts.I3G0, counts.I3G1, target);
            last_report = tic;
        end
    end

    fprintf("\n=== Gyro bias [rad/s], paste into mainAirBearing.cpp ===\n");
    keys = {'LSM0','LSM1','I3G0','I3G1'};
    for i = 1:numel(keys)
        k = keys{i};
        b = mean(raw.(k), 1) * gyroLsbToRadps() .* gyroCorr(k);
        fprintf("static constexpr float GYRO_BIAS_%s[3] = {%+.6ff, %+.6ff, %+.6ff};\n", ...
                k, b(1), b(2), b(3));
    end
    fprintf("\n");
end

% =====================================================================
% Accel bias  (mean - expected_gravity)
% =====================================================================
function runAccel(sp, N, gravity)
    fprintf("\n[ACCEL] Hold satellite still. Expected gravity (raw frame) = [%g %g %g] m/s^2\n", ...
            gravity(1), gravity(2), gravity(3));
    fprintf("        Collecting %d samples per IMU...\n", N);
    raw = {[], []};
    counts = [0 0];
    last_report = tic;

    while min(counts) < N
        line = readlineSafe(sp);
        if isempty(line), continue; end
        [tag, vals] = parseCsv(line);
        if ~strcmp(tag, 'IMU'), continue; end
        idx = vals(1) + 1;     % 1-based
        a = vals(2:4);
        if counts(idx) < N
            raw{idx}(end+1, :) = a;
            counts(idx) = counts(idx) + 1;
        end
        if toc(last_report) > 1.0
            fprintf("  LSM0=%d LSM1=%d / %d\n", counts(1), counts(2), N);
            last_report = tic;
        end
    end

    fprintf("\n=== Accel bias [m/s^2], paste into mainAirBearing.cpp ===\n");
    for i = 1:2
        m = mean(raw{i}, 1) * accelLsbToMps2();
        b = m - gravity(:)';
        fprintf("static constexpr float ACCEL_BIAS_LSM%d[3] = {%+.6ff, %+.6ff, %+.6ff};\n", ...
                i-1, b(1), b(2), b(3));
        fprintf("    // raw mean = [%+.4f %+.4f %+.4f]  bias = mean - g_expected\n", m(1), m(2), m(3));
    end
    fprintf("\n");
end

% =====================================================================
% Mag alignment via sin/cos demodulation
% =====================================================================
function runMag(sp, freq, duration)
    coil_axes = {'X', 'Y', 'Z'};
    A = nan(3, 3, 3);   % A(sensor, mag_axis, coil_axis): demod amplitude

    for c = 1:3
        fprintf("\n[MAG] Drive helmholtz +%s coil at %.3f Hz with the satellite stationary.\n", ...
                coil_axes{c}, freq);
        input(sprintf("       Press Enter when the drive is ON to capture for %.0f seconds: ", duration), 's');
        flush(sp);
        [t, m] = collectMag(sp, duration);
        fprintf("  Captured %d samples per sensor.\n", min(arrayfun(@(k) size(m{k}, 2), 1:3)));

        for s = 1:3
            ts = t{s} - t{s}(1);                    % seconds, zero-based
            ms = m{s};                              % 3 x N raw counts
            % Lock-in detect at `freq` against zero-mean samples
            for ax = 1:3
                x = ms(ax, :) - mean(ms(ax, :));
                I = mean(x .* cos(2*pi*freq*ts));
                Q = mean(x .* sin(2*pi*freq*ts));
                A(s, ax, c) = sqrt(I^2 + Q^2);
            end
        end
    end

    fprintf("\n=== Mag alignment, paste into mainAirBearing.cpp ===\n");
    fprintf("static const float MAG_ALIGN[3][3][3] = {\n");
    for s = 1:3
        % Per-coil unit direction in the magnetometer frame
        cols = squeeze(A(s, :, :));    % 3 mag axes  x  3 coil drives
        norms = vecnorm(cols, 2, 1);
        norms(norms < eps) = 1;
        R_mag_from_coil = cols ./ norms;
        % Map mag readings into coil/body frame
        R_align = R_mag_from_coil';     % rotation: B_body = R_align * B_mag
        fprintf("    /* sensor %d */\n", s-1);
        fprintf("    { {%+.6ff, %+.6ff, %+.6ff},\n", R_align(1,1), R_align(1,2), R_align(1,3));
        fprintf("      {%+.6ff, %+.6ff, %+.6ff},\n", R_align(2,1), R_align(2,2), R_align(2,3));
        fprintf("      {%+.6ff, %+.6ff, %+.6ff} },\n", R_align(3,1), R_align(3,2), R_align(3,3));
    end
    fprintf("};\n\n");

    fprintf("Demod amplitudes per sensor (rows = mag axis x/y/z, cols = coil drive X/Y/Z):\n");
    for s = 1:3
        fprintf("  sensor %d:\n", s-1);
        disp(squeeze(A(s, :, :)));
    end
end

function [t, m] = collectMag(sp, duration)
    % Returns t{1..3} (sec) and m{1..3} (3 x N raw counts) per sensor.
    t = {[], [], []};
    m = {[], [], []};
    t0 = tic;
    last_report = tic;
    while toc(t0) < duration
        line = readlineSafe(sp);
        if isempty(line), continue; end
        toks = regexp(line, '^MAGCSV,(\d+),(\d+),0x[0-9A-Fa-f]+,(-?\d+),(-?\d+),(-?\d+),', 'tokens', 'once');
        if isempty(toks), continue; end
        t_ms = str2double(toks{1});
        idx  = str2double(toks{2}) + 1;     % 1..3
        mx   = str2double(toks{3});
        my   = str2double(toks{4});
        mz   = str2double(toks{5});
        t{idx}(end+1)    = t_ms / 1000.0;
        m{idx}(:, end+1) = [mx; my; mz];
        if toc(last_report) > 1.0
            fprintf("  capturing... %.1fs / %.0fs\n", toc(t0), duration);
            last_report = tic;
        end
    end
end

% =====================================================================
% Helpers
% =====================================================================
function line = readlineSafe(sp)
    try
        line = char(readline(sp));
    catch
        line = '';
    end
    if isempty(line), return; end
    line = strtrim(line);
end

function [tag, vals] = parseCsv(line)
    % Returns tag in {'IMU','GYR','MAG',''} and a numeric payload vector.
    % IMU: vals = [sensor, ax, ay, az, gx, gy, gz]   (t_ms not returned)
    % GYR: vals = [sensor, gx, gy, gz]
    % MAG: vals = [sensor, mx, my, mz, temp, status]  (t_ms and addr stripped)
    tag = ''; vals = [];
    toks = regexp(line, '^IMUCSV,(\d+),(\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+)$', 'tokens', 'once');
    if ~isempty(toks)
        tag = 'IMU';
        nums = str2double(toks);
        vals = [nums(2), nums(3:8)];   % sensor, ax,ay,az,gx,gy,gz
        return;
    end
    toks = regexp(line, '^GYRCSV,(\d+),(\d+),(-?\d+),(-?\d+),(-?\d+)$', 'tokens', 'once');
    if ~isempty(toks)
        tag = 'GYR';
        nums = str2double(toks);
        vals = [nums(2), nums(3:5)];
        return;
    end
    toks = regexp(line, '^MAGCSV,(\d+),(\d+),0x[0-9A-Fa-f]+,(-?\d+),(-?\d+),(-?\d+),(\d+),(\d+)$', 'tokens', 'once');
    if ~isempty(toks)
        tag = 'MAG';
        nums = str2double(toks);
        vals = [nums(2), nums(3:7)];   % sensor, mx,my,mz,temp,status
        return;
    end
end

% =====================================================================
% Magnetometer live viewer
% =====================================================================
function runMagView(sp, history_len, apply_cal)
    % Live plot of all 3 magnetometers (raw or calibrated) plus a 3D scatter
    % cloud. Useful for surveying the lab's environmental field, watching the
    % helmholtz drive, or sanity-checking the existing MAG_BIAS / MAG_CAL_SCALE
    % constants.
    %
    %   apply_cal = false : show raw LSB counts straight from MAGCSV
    %   apply_cal = true  : apply MAG_BIAS, MAG_CAL_SCALE, MAG_SCALE_XY/Z to
    %                       display calibrated field in microtesla
    [MAG_BIAS, MAG_SCALE, MAG_SCALE_XY, MAG_SCALE_Z] = magConstants();

    if apply_cal
        unit = '\muT';
    else
        unit = 'LSB';
    end
    fprintf("\n[MAGVIEW] Streaming %d magnetometer samples (%s). Close the figure to stop.\n", ...
            history_len, unit);

    colors = [0.90 0.20 0.20; 0.20 0.65 0.20; 0.20 0.35 0.90];
    sensor_names = {'S0 (0x0C)', 'S1 (0x0D)', 'S2 (0x0F)'};

    fig = figure('Name','MagView — live magnetometer','NumberTitle','off', ...
                 'Color','w','Position',[60 80 1280 820]);

    % Time-series subplots (one per axis x/y/z)
    ax_ts = gobjects(3,1);
    h_ts  = gobjects(3,3);   % axis row, sensor col
    axis_names = {'B_x','B_y','B_z'};
    for a = 1:3
        ax_ts(a) = subplot(3, 2, 2*a - 1, 'Parent', fig);
        hold(ax_ts(a), 'on'); grid(ax_ts(a), 'on'); box(ax_ts(a), 'on');
        ylabel(ax_ts(a), sprintf('%s (%s)', axis_names{a}, unit));
        if a == 1
            title(ax_ts(a), 'Magnetometer time series');
        end
        if a == 3
            xlabel(ax_ts(a), 'Time (s)');
        end
        for s = 1:3
            h_ts(a, s) = plot(ax_ts(a), nan, nan, 'Color', colors(s,:), ...
                              'LineWidth', 1.2, 'DisplayName', sensor_names{s});
        end
        if a == 1
            legend(ax_ts(a), 'Location', 'northeast', 'FontSize', 8);
        end
    end

    % 3D scatter cloud
    ax3d = subplot(3, 2, [2 4 6], 'Parent', fig);
    hold(ax3d, 'on'); grid(ax3d, 'on'); box(ax3d, 'on'); axis(ax3d, 'equal');
    xlabel(ax3d, sprintf('B_x (%s)', unit));
    ylabel(ax3d, sprintf('B_y (%s)', unit));
    zlabel(ax3d, sprintf('B_z (%s)', unit));
    title(ax3d, 'Magnetometer cloud (latest sample highlighted)');
    view(ax3d, [135 25]);
    h_scatter = gobjects(3,1);
    h_latest  = gobjects(3,1);
    for s = 1:3
        h_scatter(s) = scatter3(ax3d, nan, nan, nan, 6, colors(s,:), 'filled', ...
                                'MarkerFaceAlpha', 0.35, 'DisplayName', sensor_names{s});
        h_latest(s)  = plot3(ax3d, nan, nan, nan, 'o', 'Color', colors(s,:), ...
                             'MarkerFaceColor', colors(s,:), 'MarkerSize', 9, ...
                             'LineWidth', 1.5, 'HandleVisibility', 'off');
    end
    legend(ax3d, 'Location', 'northeast', 'FontSize', 8);

    % Ring buffers per sensor
    t_buf = nan(3, history_len);
    m_buf = nan(3, 3, history_len);   % sensor, axis, sample
    n_buf = zeros(1, 3);
    t0 = NaN;
    last_draw = tic;
    last_status = tic;
    samples_total = 0;

    while isvalid(fig)
        line = readlineSafe(sp);
        if isempty(line), continue; end
        toks = regexp(line, '^MAGCSV,(\d+),(\d+),0x[0-9A-Fa-f]+,(-?\d+),(-?\d+),(-?\d+),(\d+),(\d+)$', 'tokens', 'once');
        if isempty(toks), continue; end

        t_ms   = str2double(toks{1});
        s_idx  = str2double(toks{2}) + 1;     % 1..3
        raw    = [str2double(toks{3}); str2double(toks{4}); str2double(toks{5})];
        status = str2double(toks{7});
        if status ~= 0 || s_idx < 1 || s_idx > 3, continue; end

        if isnan(t0), t0 = t_ms; end
        t_s = (t_ms - t0) / 1000.0;

        if apply_cal
            centered = raw - MAG_BIAS(s_idx, :)';
            scaled   = centered .* MAG_SCALE(s_idx, :)';
            % Convert LSB->T then T->uT for readable axes
            display_vec = scaled .* [MAG_SCALE_XY; MAG_SCALE_XY; MAG_SCALE_Z] * 1e6;
        else
            display_vec = raw;
        end

        % Append into ring buffer
        if n_buf(s_idx) < history_len
            n_buf(s_idx) = n_buf(s_idx) + 1;
            j = n_buf(s_idx);
            t_buf(s_idx, j) = t_s;
            m_buf(s_idx, :, j) = display_vec;
        else
            t_buf(s_idx, :) = [t_buf(s_idx, 2:end), t_s];
            m_buf(s_idx, :, :) = cat(3, m_buf(s_idx, :, 2:end), reshape(display_vec, 1, 3, 1));
        end
        samples_total = samples_total + 1;

        % Throttle redraws so the loop can keep up with serial
        if toc(last_draw) > 0.08
            for s = 1:3
                n = n_buf(s);
                if n < 1, continue; end
                tt = t_buf(s, 1:n);
                xyz = squeeze(m_buf(s, :, 1:n));   % 3 x n
                for a = 1:3
                    set(h_ts(a, s), 'XData', tt, 'YData', xyz(a, :));
                end
                set(h_scatter(s), 'XData', xyz(1,:), 'YData', xyz(2,:), 'ZData', xyz(3,:));
                set(h_latest(s),  'XData', xyz(1,end), 'YData', xyz(2,end), 'ZData', xyz(3,end));
            end

            valid = ~isnan(t_buf);
            if any(valid(:))
                tmin = min(t_buf(valid)); tmax = max(t_buf(valid));
                if tmax > tmin
                    for a = 1:3, xlim(ax_ts(a), [tmin tmax]); end
                end
            end

            % Update title with live |B| and per-sensor mean
            stats = strings(1,3);
            for s = 1:3
                n = n_buf(s);
                if n < 1, stats(s) = sprintf('%s n/a', sensor_names{s}); continue; end
                last = squeeze(m_buf(s, :, n));
                bn = norm(last);
                stats(s) = sprintf('%s |B|=%.2f', sensor_names{s}, bn);
            end
            title(ax3d, sprintf('Mag cloud (%s)  |  %s  |  %s  |  %s', ...
                                unit, stats(1), stats(2), stats(3)));

            drawnow limitrate;
            last_draw = tic;
        end

        if toc(last_status) > 2.0
            fprintf("  total samples=%d  buffered S0=%d S1=%d S2=%d\n", ...
                    samples_total, n_buf(1), n_buf(2), n_buf(3));
            last_status = tic;
        end
    end

    fprintf("MagView closed. Captured %d samples total.\n", samples_total);
end

function [BIAS, SCALE, SXY, SZ] = magConstants()
    % Mirror of MAG_BIAS / MAG_CAL_SCALE / MAG_SCALE_XY / MAG_SCALE_Z in
    % mainAirBearing.cpp. Update if you change the firmware constants.
    BIAS = [ ...
        -396.500,  2064.500,  1274.500;
         3192.000, -2262.000,   133.500;
         -375.500,  2299.000, -2352.000];

    SCALE = [ ...
        0.894505, 0.966746, 1.179710;
        0.923704, 0.989683, 1.102564;
        0.887704, 0.950533, 1.217349];

    SXY = 0.150e-6;     % T per LSB (X,Y axes)
    SZ  = 0.242e-6;     % T per LSB (Z axis)
end
