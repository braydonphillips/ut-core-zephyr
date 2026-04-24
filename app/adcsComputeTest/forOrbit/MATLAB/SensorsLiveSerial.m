%% SensorsLiveSerial — Real-time gyro + magnetometer telemetry from serial port
%
%  Reads the UART output from the adcsComputeTest Zephyr app (AIR_BEARING=ON)
%  and plots each rate gyro on a per-axis overlay so scale/bias disagreements
%  between the LSM6DSV pair and the I3G4250D pair are visually obvious.
%  Also plots the fused magnetometer (Bx, By, Bz in µT).
%
%  Four subplots: omega_x, omega_y, omega_z (gyro), then B_x/B_y/B_z (mag)
%    Gyro traces: LSM0, LSM1, I3G0, I3G1, FUSED
%    Mag traces:  Bx (blue), By (red), Bz (green)
%
%  After N_CAPTURE gyro diagnostic lines are received, exports a CSV with columns:
%    t [s],
%    lsm0_xyz, lsm1_xyz, i3g0_xyz, i3g1_xyz, fused_xyz  [rad/s],
%    mag_x, mag_y, mag_z  [T]  (latest fused mag at time of each gyro sample)
%
%  Requires mainAirBearing.cpp to emit:
%    GYRO [rad/s] LSM0:+x +y +z | LSM1:+x +y +z | I3G0:+x +y +z | I3G1:+x +y +z | FUSED:+x +y +z
%    [ADCS] B [T]: x y z  |B|=... (... uT)
%
%  Usage:
%    SensorsLiveSerial("COM5")          % Windows
%    SensorsLiveSerial("/dev/ttyUSB0")  % Linux
%    SensorsLiveSerial("COM5", 400)     % custom history length (samples)
%
%  Press Ctrl+C or close the figure to stop.

function SensorsLiveSerial(port, history_len)
    N_CAPTURE = 25;

    if nargin < 2, history_len = 400; end

    % =========================================================================
    %  Serial port setup
    % =========================================================================
    s = serialport(port, 115200, "Timeout", 5);
    configureTerminator(s, "LF");
    flush(s);
    cleanupObj = onCleanup(@() delete(s));

    fprintf("Connected to %s — waiting for sensor lines...\n", port);
    fprintf("Will capture %d gyro samples then export CSV.\n", N_CAPTURE);

    % =========================================================================
    %  Ring buffers (live plot)
    % =========================================================================
    tg_buf    = NaN(1, history_len);
    lsm0_buf  = NaN(3, history_len);
    lsm1_buf  = NaN(3, history_len);
    i3g0_buf  = NaN(3, history_len);
    i3g1_buf  = NaN(3, history_len);
    fused_buf = NaN(3, history_len);
    tm_buf    = NaN(1, history_len);
    mag_buf   = NaN(3, history_len);   % [T]

    % Capture buffers (rad/s for gyro, T for mag)
    cap_t     = NaN(N_CAPTURE, 1);
    cap_lsm0  = NaN(N_CAPTURE, 3);
    cap_lsm1  = NaN(N_CAPTURE, 3);
    cap_i3g0  = NaN(N_CAPTURE, 3);
    cap_i3g1  = NaN(N_CAPTURE, 3);
    cap_fused = NaN(N_CAPTURE, 3);
    cap_mag   = NaN(N_CAPTURE, 3);
    n_cap     = 0;
    exported  = false;
    latest_mag_T = NaN(1, 3);

    t0 = NaN;

    % =========================================================================
    %  Figure — three gyro subplots + one mag subplot
    % =========================================================================
    fig = figure('Name', 'Sensors Live — Gyro + Mag', 'NumberTitle', 'off', ...
                 'Color', 'w', 'Position', [80 80 1000 1050]);

    axis_labels = {'\omega_x','\omega_y','\omega_z'};
    sensor_colors = struct( ...
        'LSM0',  [0.00 0.45 0.74], ...
        'LSM1',  [0.30 0.75 0.93], ...
        'I3G0',  [0.85 0.33 0.10], ...
        'I3G1',  [0.98 0.70 0.30], ...
        'FUSED', [0.00 0.00 0.00]);
    mag_colors = {[0.00 0.45 0.74], [0.85 0.33 0.10], [0.47 0.67 0.19]};

    ax_g = gobjects(3,1);
    hl0  = gobjects(3,1);
    hl1  = gobjects(3,1);
    hi0  = gobjects(3,1);
    hi1  = gobjects(3,1);
    hfu  = gobjects(3,1);

    for axi = 1:3
        ax_g(axi) = subplot(4,1,axi, 'Parent', fig);
        hold(ax_g(axi),'on'); grid(ax_g(axi),'on');
        title(ax_g(axi), sprintf('Gyro %s — per-sensor overlay', axis_labels{axi}));
        ylabel(ax_g(axi), [axis_labels{axi} ' (deg/s)']);

        hl0(axi) = plot(ax_g(axi), tg_buf, lsm0_buf(axi,:), ...
                        'Color', sensor_colors.LSM0, 'LineWidth', 1.2, 'DisplayName', 'LSM0');
        hl1(axi) = plot(ax_g(axi), tg_buf, lsm1_buf(axi,:), ...
                        'Color', sensor_colors.LSM1, 'LineWidth', 1.2, 'DisplayName', 'LSM1');
        hi0(axi) = plot(ax_g(axi), tg_buf, i3g0_buf(axi,:), ...
                        'Color', sensor_colors.I3G0, 'LineWidth', 1.2, 'DisplayName', 'I3G0');
        hi1(axi) = plot(ax_g(axi), tg_buf, i3g1_buf(axi,:), ...
                        'Color', sensor_colors.I3G1, 'LineWidth', 1.2, 'DisplayName', 'I3G1');
        hfu(axi) = plot(ax_g(axi), tg_buf, fused_buf(axi,:), ...
                        'Color', sensor_colors.FUSED, 'LineStyle', '--', ...
                        'LineWidth', 1.8, 'DisplayName', 'FUSED');
        legend(ax_g(axi), 'Location', 'eastoutside', 'FontSize', 8);
    end

    % Mag subplot (bottom)
    ax_m = subplot(4,1,4, 'Parent', fig);
    hold(ax_m, 'on'); grid(ax_m, 'on');
    title(ax_m, 'Fused Magnetometer');
    ylabel(ax_m, 'B (\muT)');
    xlabel(ax_m, 'Time (s)');
    hm(1) = plot(ax_m, tm_buf, mag_buf(1,:)*1e6, 'Color', mag_colors{1}, 'LineWidth', 1.4, 'DisplayName', 'B_x');
    hm(2) = plot(ax_m, tm_buf, mag_buf(2,:)*1e6, 'Color', mag_colors{2}, 'LineWidth', 1.4, 'DisplayName', 'B_y');
    hm(3) = plot(ax_m, tm_buf, mag_buf(3,:)*1e6, 'Color', mag_colors{3}, 'LineWidth', 1.4, 'DisplayName', 'B_z');
    legend(ax_m, 'Location', 'eastoutside', 'FontSize', 8);

    hStatus = title(ax_g(1), sprintf('Gyro %s — per-sensor overlay  [capturing 0 / %d]', ...
                                      axis_labels{1}, N_CAPTURE));

    % =========================================================================
    %  Main loop
    % =========================================================================
    fprintf("Streaming...\n");

    gyro_pat = ['^GYRO\s+\[rad/s\]\s+' ...
                'LSM0:\s*' num_pat(3) '\s*\|\s*' ...
                'LSM1:\s*' num_pat(3) '\s*\|\s*' ...
                'I3G0:\s*' num_pat(3) '\s*\|\s*' ...
                'I3G1:\s*' num_pat(3) '\s*\|\s*' ...
                'FUSED:\s*' num_pat(3)];

    mag_pat = ['^\[ADCS\] B \[T\]:\s+' num_pat(3)];

    while isvalid(fig)
        try
            line = readline(s);
        catch
            continue;
        end
        if isempty(line), continue; end
        line = char(line);

        if isnan(t0)
            t0 = double(posixtime(datetime('now')));
        end
        tnow = double(posixtime(datetime('now'))) - t0;

        % --- magnetometer line ---
        m_toks = regexp(line, mag_pat, 'tokens');
        if ~isempty(m_toks)
            mvals = str2double(m_toks{1});
            if ~any(isnan(mvals))
                latest_mag_T = mvals;
                tm_buf  = [tm_buf(2:end), tnow];
                mag_buf = [mag_buf(:,2:end), mvals(:)];
                for k = 1:3
                    set(hm(k), 'XData', tm_buf, 'YData', mag_buf(k,:)*1e6);
                end
                valid = ~isnan(tm_buf);
                if sum(valid) > 1
                    xlim(ax_m, [min(tm_buf(valid)) max(tm_buf(valid))]);
                end
            end
        end

        % --- gyro line ---
        g_toks = regexp(line, gyro_pat, 'tokens');
        if isempty(g_toks)
            drawnow limitrate;
            continue;
        end

        vals = str2double(g_toks{1});
        if any(isnan(vals)), continue; end

        lsm0_rps = vals(1:3);
        lsm1_rps = vals(4:6);
        i3g0_rps = vals(7:9);
        i3g1_rps = vals(10:12);
        fus_rps  = vals(13:15);

        % Plot buffers (deg/s for readability)
        tg_buf    = [tg_buf(2:end),    tnow];
        lsm0_buf  = [lsm0_buf(:,2:end),  rad2deg(lsm0_rps(:))];
        lsm1_buf  = [lsm1_buf(:,2:end),  rad2deg(lsm1_rps(:))];
        i3g0_buf  = [i3g0_buf(:,2:end),  rad2deg(i3g0_rps(:))];
        i3g1_buf  = [i3g1_buf(:,2:end),  rad2deg(i3g1_rps(:))];
        fused_buf = [fused_buf(:,2:end), rad2deg(fus_rps(:))];

        % Capture (raw rad/s; latest mag in T)
        if n_cap < N_CAPTURE
            n_cap = n_cap + 1;
            cap_t(n_cap)       = tnow;
            cap_lsm0(n_cap,:)  = lsm0_rps;
            cap_lsm1(n_cap,:)  = lsm1_rps;
            cap_i3g0(n_cap,:)  = i3g0_rps;
            cap_i3g1(n_cap,:)  = i3g1_rps;
            cap_fused(n_cap,:) = fus_rps;
            cap_mag(n_cap,:)   = latest_mag_T;
        end

        if n_cap == N_CAPTURE && ~exported
            exported = true;
            csv_file = export_csv(cap_t, cap_lsm0, cap_lsm1, cap_i3g0, cap_i3g1, cap_fused, cap_mag);
            fprintf("Exported %d samples to: %s\n", N_CAPTURE, csv_file);
            set(hStatus, 'String', ...
                sprintf('Gyro %s — per-sensor overlay  [captured %d — saved to %s]', ...
                        axis_labels{1}, N_CAPTURE, csv_file));
        elseif ~exported
            set(hStatus, 'String', ...
                sprintf('Gyro %s — per-sensor overlay  [capturing %d / %d]', ...
                        axis_labels{1}, n_cap, N_CAPTURE));
        end

        for axi = 1:3
            set(hl0(axi), 'XData', tg_buf, 'YData', lsm0_buf(axi,:));
            set(hl1(axi), 'XData', tg_buf, 'YData', lsm1_buf(axi,:));
            set(hi0(axi), 'XData', tg_buf, 'YData', i3g0_buf(axi,:));
            set(hi1(axi), 'XData', tg_buf, 'YData', i3g1_buf(axi,:));
            set(hfu(axi), 'XData', tg_buf, 'YData', fused_buf(axi,:));
        end

        valid = ~isnan(tg_buf);
        if any(valid)
            tmin = min(tg_buf(valid));
            tmax = max(tg_buf(valid));
            if tmax > tmin
                for axi = 1:3, xlim(ax_g(axi), [tmin tmax]); end
            end
        end

        drawnow limitrate;
    end

    fprintf("Viewer closed.\n");
end

% =========================================================================
%  Write gyro + mag capture to a timestamped CSV; returns filename.
%  Gyro columns in rad/s; mag columns in T.
% =========================================================================
function fname = export_csv(cap_t, cap_lsm0, cap_lsm1, cap_i3g0, cap_i3g1, cap_fused, cap_mag)
    fname = sprintf('sensors_capture_%s.csv', char(datetime('now','Format','yyyyMMdd_HHmmss')));
    cols = {'t', ...
            'lsm0_x','lsm0_y','lsm0_z', ...
            'lsm1_x','lsm1_y','lsm1_z', ...
            'i3g0_x','i3g0_y','i3g0_z', ...
            'i3g1_x','i3g1_y','i3g1_z', ...
            'fused_x','fused_y','fused_z', ...
            'mag_x','mag_y','mag_z'};
    T = array2table([cap_t, cap_lsm0, cap_lsm1, cap_i3g0, cap_i3g1, cap_fused, cap_mag], ...
                    'VariableNames', cols);
    writetable(T, fname);
end

% =========================================================================
%  Helper: build a regex token group string for N floating-point numbers
% =========================================================================
function s = num_pat(n)
    tok = '([\-+\d\.eE]+)';
    s = tok;
    for k = 2:n
        s = [s '\s+' tok]; %#ok<AGROW>
    end
end
