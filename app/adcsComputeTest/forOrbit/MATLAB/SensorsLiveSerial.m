%% SensorsLiveSerial — Real-time raw sensor telemetry from serial port
%
%  Reads the UART output from the adcsComputeTest Zephyr app (AIR_BEARING=ON)
%  and plots raw sensor measurements:
%    1) Accelerometer  (m/s²)
%    2) Raw gyroscope  (deg/s)
%    3) Magnetometer   (µT)
%
%  After N_CAPTURE complete samples are received, exports a CSV with columns:
%    ax, ay, az, omega_x, omega_y, omega_z
%  (accel in m/s², gyro in rad/s — raw, unconverted, for bias/noise analysis)
%
%  Requires that mainAirBearing.cpp emits the lines:
%    [ADCS] a [m/s^2]: ax ay az
%    [ADCS] gy_raw [rad/s]: gx gy gz
%    [ADCS] B [T]: Bx By Bz
%
%  Usage:
%    SensorsLiveSerial("COM5")         % Windows
%    SensorsLiveSerial("/dev/ttyUSB0") % Linux
%    SensorsLiveSerial("COM5", 300)    % custom history length (samples)
%
%  Press Ctrl+C or close the figure to stop.

function SensorsLiveSerial(port, history_len)
    % Pull N_CAPTURE from the enclosing script scope
    N_CAPTURE = 500;

    if nargin < 2, history_len = 200; end

    % =========================================================================
    %  Serial port setup
    % =========================================================================
    s = serialport(port, 115200, "Timeout", 5);
    configureTerminator(s, "LF");
    flush(s);
    cleanupObj = onCleanup(@() delete(s));

    fprintf("Connected to %s — waiting for sensor data...\n", port);
    fprintf("Will capture %d samples then export CSV.\n", N_CAPTURE);

    % =========================================================================
    %  Ring buffers (live plot)
    % =========================================================================
    t_buf   = NaN(1, history_len);
    a_buf   = NaN(3, history_len);   % accelerometer [m/s²]
    gy_buf  = NaN(3, history_len);   % raw gyro [deg/s] for display
    b_buf   = NaN(3, history_len);   % magnetometer [µT]

    % Capture buffers (raw, for CSV export)
    cap_a  = NaN(N_CAPTURE, 3);   % m/s²
    cap_gy = NaN(N_CAPTURE, 3);   % rad/s
    n_cap  = 0;
    exported = false;

    t0 = NaN;

    % Per-cycle scratch
    cur_a  = [0 0 0];
    cur_gy = [0 0 0];
    cur_b  = [0 0 0];
    got_a  = false;
    got_gy = false;
    got_b  = false;
    cycle_complete = false;

    % =========================================================================
    %  Figure and subplots
    % =========================================================================
    fig = figure('Name', 'Sensor Data', 'NumberTitle', 'off', ...
                 'Color', 'w', 'Position', [100 80 820 760]);

    axcolors = {'r','g','b'};

    % --- Accelerometer ---
    ax_a = subplot(3,1,1,'Parent', fig);
    hold(ax_a,'on'); grid(ax_a,'on');
    title(ax_a, 'Accelerometer'); ylabel(ax_a, 'a (m/s^2)');
    ha = gobjects(3,1);
    anames = {'a_x','a_y','a_z'};
    for i = 1:3
        ha(i) = plot(ax_a, t_buf, a_buf(i,:), axcolors{i}, ...
                     'LineWidth', 1.5, 'DisplayName', anames{i});
    end
    legend(ax_a, 'Location','eastoutside', 'FontSize', 9);

    % --- Raw gyroscope ---
    ax_g = subplot(3,1,2,'Parent', fig);
    hold(ax_g,'on'); grid(ax_g,'on');
    title(ax_g, 'Raw Gyroscope'); ylabel(ax_g, '\omega (deg/s)');
    hg = gobjects(3,1);
    gnames = {'\omega_x','\omega_y','\omega_z'};
    for i = 1:3
        hg(i) = plot(ax_g, t_buf, gy_buf(i,:), axcolors{i}, ...
                     'LineWidth', 1.5, 'DisplayName', gnames{i});
    end
    legend(ax_g, 'Location','eastoutside', 'FontSize', 9);

    % --- Magnetometer ---
    ax_b = subplot(3,1,3,'Parent', fig);
    hold(ax_b,'on'); grid(ax_b,'on');
    title(ax_b, 'Magnetometer'); ylabel(ax_b, 'B (\muT)');
    xlabel(ax_b, 'Time (s)');
    hb = gobjects(3,1);
    bnames = {'B_x','B_y','B_z'};
    for i = 1:3
        hb(i) = plot(ax_b, t_buf, b_buf(i,:), axcolors{i}, ...
                     'LineWidth', 1.5, 'DisplayName', bnames{i});
    end
    legend(ax_b, 'Location','eastoutside', 'FontSize', 9);

    % Status annotation shown on the accelerometer subplot title
    hStatus = title(ax_a, sprintf('Accelerometer  [capturing 0 / %d]', N_CAPTURE));

    % =========================================================================
    %  Main loop
    % =========================================================================
    fprintf("Streaming...\n");
    fp = '\[ADCS\]';   % common prefix anchor

    while isvalid(fig)
        try
            line = readline(s);
        catch
            continue;
        end
        if isempty(line), continue; end
        line = char(line);

        % --- Accelerometer ---
        toks = regexp(line, [fp '\s+a\s+\[m/s\^2\]:\s+' num_pat(3)], 'tokens');
        if ~isempty(toks)
            cur_a = str2double(toks{1});
            got_a = true;
            continue;
        end

        % --- Raw gyroscope ---
        toks = regexp(line, [fp '\s+gy_raw\s+\[rad/s\]:\s+' num_pat(3)], 'tokens');
        if ~isempty(toks)
            cur_gy = str2double(toks{1});
            got_gy = true;
            continue;
        end

        % --- Magnetometer ---
        toks = regexp(line, [fp '\s+B\s+\[T\]:\s+' num_pat(3)], 'tokens');
        if ~isempty(toks)
            cur_b = str2double(toks{1});
            got_b = true;
            continue;
        end

        % --- MTQ line: cycle-completion trigger ---
        if ~isempty(regexp(line, [fp '\s+mtq\s+\[A\*m\^2\]:'], 'once'))
            if got_a && got_gy && got_b
                cycle_complete = true;
            end
            got_a = false; got_gy = false; got_b = false;
        end

        % --- Push a complete sample ---
        if cycle_complete
            cycle_complete = false;

            if isnan(t0)
                t0 = double(posixtime(datetime('now')));
            end
            t_now = double(posixtime(datetime('now'))) - t0;

            % Live plot buffers
            t_buf  = [t_buf(2:end),   t_now];
            a_buf  = [a_buf(:,2:end),  cur_a(:)];
            gy_buf = [gy_buf(:,2:end), rad2deg(cur_gy(:))];
            b_buf  = [b_buf(:,2:end),  cur_b(:) * 1e6];

            % Capture buffer (raw units, for CSV)
            if n_cap < N_CAPTURE
                n_cap = n_cap + 1;
                cap_a(n_cap,:)  = cur_a;
                cap_gy(n_cap,:) = cur_gy;   % rad/s — keep raw for bias/noise
            end

            % Export once capture is complete
            if n_cap == N_CAPTURE && ~exported
                exported = true;
                csv_file = export_csv(cap_a, cap_gy);
                fprintf("Exported %d samples to: %s\n", N_CAPTURE, csv_file);
                set(hStatus, 'String', ...
                    sprintf('Accelerometer  [captured %d — saved to %s]', N_CAPTURE, csv_file));
            elseif ~exported
                set(hStatus, 'String', ...
                    sprintf('Accelerometer  [capturing %d / %d]', n_cap, N_CAPTURE));
            end

            % Update plots
            for i = 1:3
                set(ha(i), 'XData', t_buf, 'YData', a_buf(i,:));
                set(hg(i), 'XData', t_buf, 'YData', gy_buf(i,:));
                set(hb(i), 'XData', t_buf, 'YData', b_buf(i,:));
            end

            % Auto-scroll x-axis
            valid = ~isnan(t_buf);
            if any(valid)
                tmin = min(t_buf(valid));
                tmax = max(t_buf(valid));
                if tmax > tmin
                    xlim(ax_a, [tmin tmax]);
                    xlim(ax_g, [tmin tmax]);
                    xlim(ax_b, [tmin tmax]);
                end
            end

            drawnow limitrate;
        end
    end

    fprintf("Viewer closed.\n");
end

% =========================================================================
%  Write capture data to a timestamped CSV; returns the filename used.
%  Columns: ax, ay, az [m/s²], omega_x, omega_y, omega_z [rad/s]
% =========================================================================
function fname = export_csv(cap_a, cap_gy)
    fname = sprintf('sensor_capture_%s.csv', char(datetime('now','Format','yyyyMMdd_HHmmss')));
    T = array2table([cap_a, cap_gy], ...
        'VariableNames', {'ax','ay','az','omega_x','omega_y','omega_z'});
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
