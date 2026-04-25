%% MagCalTool — MLX90393 calibration helper for magCalTest output
%
% Supports:
%   1) Live serial stream from firmware:
%        MagCalTool("COM5")
%        MagCalTool("COM5", 3000, 180)   % 3 minute auto-capture
%   2) Parse saved text log file:
%        MagCalTool("C:\logs\mag_run.txt")
%
% Expected data line format from firmware:
%   MAGCSV,t_ms,sensor,addr,mx,my,mz,temp,status
% Example:
%   MAGCSV,12345,1,0x0D,321,-80,1500,46123,0
%
% The script:
%   - Parses samples for each sensor
%   - Plots 3D point clouds and component traces
%   - Computes simple hard-iron offset + diagonal scale estimate
%   - Prints ready-to-copy constants
%
% Saved preset (from your latest run, sample 100):
%   MAG_BIAS =
%     [-342.0,   2022.0,  1234.0;
%      3238.5,  -2290.5,    68.0;
%      -320.5,   2263.5, -2411.0]
%   MAG_SCALE =
%     [0.7760, 1.2137, 1.1270;
%      0.8901, 1.0149, 1.1221;
%      1.0108, 0.8851, 1.1353]

function MagCalTool(source, max_points, capture_seconds)
    if nargin < 1
        error('Usage: MagCalTool("COM5") or MagCalTool("path_to_log.txt")');
    end
    if nargin < 2
        max_points = 3000;
    end
    if nargin < 3
        capture_seconds = 180; % default live capture duration: 3 minutes
    end

    is_serial = isSerialSource(source);

    % Per-sensor ring buffers.
    S = initSensors(max_points);
    t0_ms = NaN;

    if is_serial
        runLive(source, S, max_points, t0_ms, capture_seconds);
    else
        runFromFile(source, S, max_points, t0_ms);
    end
end

function [bias, scale] = latestCalibrationPreset()
    % Latest saved preset from terminal log CAL lines.
    bias = [ ...
        -342.0,  2022.0,  1234.0; ...
        3238.5, -2290.5,    68.0; ...
        -320.5,  2263.5, -2411.0];

    scale = [ ...
        0.7760, 1.2137, 1.1270; ...
        0.8901, 1.0149, 1.1221; ...
        1.0108, 0.8851, 1.1353];
end

function tf = isSerialSource(source)
    s = char(source);
    tf = startsWith(upper(s), "COM") || startsWith(s, "/dev/");
end

function S = initSensors(max_points)
    for k = 1:3
        S(k).id = k - 1; %#ok<AGROW>
        S(k).addr = "";
        S(k).t = nan(1, max_points);
        S(k).m = nan(3, max_points);
        S(k).n = 0;
    end
end

function runLive(port, S, max_points, t0_ms, capture_seconds)
    sp = serialport(port, 115200, "Timeout", 5);
    configureTerminator(sp, "LF");
    flush(sp);
    cleanupObj = onCleanup(@() delete(sp)); %#ok<NASGU>

    [fig3d, fig2d, h3d, h2d] = buildFigures();
    fprintf("Connected to %s. Waiting for MAGCSV lines...\n", port);
    fprintf("Auto-capture duration: %.0f seconds\n", capture_seconds);
    fprintf("Tip: you should see 'MAGCSV,...' lines in a serial terminal too.\n");
    lines_seen = 0;
    parsed_seen = 0;
    last_report = tic;
    capture_t0 = tic;

    while isvalid(fig3d) && isvalid(fig2d)
        if toc(capture_t0) >= capture_seconds
            fprintf("Capture window complete (%.1f s). Stopping.\n", toc(capture_t0));
            break;
        end

        try
            line = char(readline(sp));
        catch
            continue;
        end

        lines_seen = lines_seen + 1;
        [ok, rec] = parseMagCsvLine(line);
        if ~ok
            if toc(last_report) > 2.0
                t_left = max(0.0, capture_seconds - toc(capture_t0));
                fprintf("Live stats: total lines=%d, MAGCSV parsed=%d, time_left=%.0fs\n", ...
                    lines_seen, parsed_seen, t_left);
                last_report = tic;
            end
            continue;
        end
        parsed_seen = parsed_seen + 1;

        [S, t0_ms] = appendRecord(S, rec, max_points, t0_ms);
        refreshPlots(S, h3d, h2d);
        drawnow limitrate;
    end

    printCalibrationSummary(S);
    fprintf("MagCalTool closed.\n");
end

function runFromFile(path_in, S, max_points, t0_ms)
    if ~isfile(path_in)
        error("File not found: %s", path_in);
    end

    lines = string(splitlines(fileread(path_in)));
    n_parsed = 0;
    for i = 1:numel(lines)
        [ok, rec] = parseMagCsvLine(lines(i));
        if ~ok
            continue;
        end
        [S, t0_ms] = appendRecord(S, rec, max_points, t0_ms);
        n_parsed = n_parsed + 1;
    end

    if n_parsed == 0
        error("No MAGCSV lines found in file: %s", path_in);
    end

    [~, ~, h3d, h2d] = buildFigures();
    refreshPlots(S, h3d, h2d);
    drawnow;
    printCalibrationSummary(S);
    fprintf("Parsed %d MAGCSV lines from %s\n", n_parsed, path_in);
end

function [ok, rec] = parseMagCsvLine(line)
    ok = false;
    rec = struct();

    toks = regexp(strtrim(char(line)), ...
        '^MAGCSV\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(0x[0-9A-Fa-f]+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*$', ...
        'tokens');
    if isempty(toks)
        return;
    end

    vals = toks{1};
    rec.t_ms = str2double(vals{1});
    rec.sensor = str2double(vals{2});
    rec.addr = string(vals{3});
    rec.mx = str2double(vals{4});
    rec.my = str2double(vals{5});
    rec.mz = str2double(vals{6});
    rec.temp = str2double(vals{7}); %#ok<NASGU>
    rec.status = str2double(vals{8});
    ok = true;
end

function [S, t0_ms] = appendRecord(S, rec, max_points, t0_ms)
    idx = rec.sensor + 1;
    if idx < 1 || idx > numel(S) || rec.status ~= 0
        return;
    end

    if isnan(t0_ms)
        t0_ms = rec.t_ms;
    end
    t_s = (rec.t_ms - t0_ms) / 1000.0;

    if strlength(S(idx).addr) == 0
        S(idx).addr = rec.addr;
    end

    if S(idx).n < max_points
        S(idx).n = S(idx).n + 1;
        j = S(idx).n;
        S(idx).t(j) = t_s;
        S(idx).m(:, j) = [rec.mx; rec.my; rec.mz];
    else
        S(idx).t = [S(idx).t(2:end), t_s];
        S(idx).m = [S(idx).m(:,2:end), [rec.mx; rec.my; rec.mz]];
    end
end

function [fig3d, fig2d, h3d, h2d] = buildFigures()
    colors = [0.90 0.20 0.20; 0.20 0.65 0.20; 0.20 0.35 0.90];

    fig3d = figure('Name','MagCal 3D Clouds','NumberTitle','off', ...
                   'Color','w','Position',[40 120 700 620]);
    ax = axes('Parent', fig3d); hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
    axis(ax, 'equal');
    xlabel(ax, 'M_x (cal)'); ylabel(ax, 'M_y (cal)'); zlabel(ax, 'M_z (cal)');
    title(ax, 'Per-sensor Magnetometer Clouds (de-biased/de-scaled)');
    view(ax, [135 25]);
    h3d.ax = ax;
    for k = 1:3
        h3d.scatter(k) = scatter3(ax, nan, nan, nan, 8, colors(k,:), 'filled'); %#ok<AGROW>
        h3d.center(k)  = plot3(ax, nan, nan, nan, 'o', 'Color', colors(k,:), ...
                               'MarkerFaceColor', colors(k,:), 'MarkerSize', 8); %#ok<AGROW>
    end
    legend(ax, {'S0','S0 center','S1','S1 center','S2','S2 center'}, ...
           'Location','eastoutside');

    fig2d = figure('Name','MagCal Components','NumberTitle','off', ...
                   'Color','w','Position',[760 80 900 860]);

    labels = {'M_x','M_y','M_z'};
    h2d.axes = gobjects(3,1);
    h2d.lines = gobjects(3,3);
    for a = 1:3
        h2d.axes(a) = subplot(3,1,a,'Parent',fig2d); hold(h2d.axes(a),'on'); grid(h2d.axes(a),'on');
        ylabel(h2d.axes(a), sprintf('%s (LSB)', labels{a}));
        if a == 1
            title(h2d.axes(a), 'Magnetometer Components vs Time');
        end
        if a == 3
            xlabel(h2d.axes(a), 'Time (s)');
        end
        for k = 1:3
            h2d.lines(a,k) = plot(h2d.axes(a), nan, nan, 'LineWidth', 1.1);
        end
    end
    h2d.lines(1,1).Color = colors(1,:);
    h2d.lines(1,2).Color = colors(2,:);
    h2d.lines(1,3).Color = colors(3,:);
    for a = 2:3
        for k = 1:3
            h2d.lines(a,k).Color = h2d.lines(1,k).Color;
        end
    end
    legend(h2d.axes(1), {'S0','S1','S2'}, 'Location','eastoutside');
end

function refreshPlots(S, h3d, h2d)
    q_txt = strings(1,3);
    max_abs = 1.0;
    for k = 1:3
        n = S(k).n;
        if n < 2
            q_txt(k) = sprintf("S%d:N/A", k-1);
            continue;
        end
        t = S(k).t(1:n);
        m = S(k).m(:,1:n);

        [mc, ~, ~] = quickCalibrate(m);
        set(h3d.scatter(k), 'XData', mc(1,:), 'YData', mc(2,:), 'ZData', mc(3,:));
        c = mean(mc, 2, 'omitnan');
        set(h3d.center(k), 'XData', c(1), 'YData', c(2), 'ZData', c(3));
        max_abs = max(max_abs, max(abs(mc), [], 'all'));

        set(h2d.lines(1,k), 'XData', t, 'YData', m(1,:));
        set(h2d.lines(2,k), 'XData', t, 'YData', m(2,:));
        set(h2d.lines(3,k), 'XData', t, 'YData', m(3,:));

        q = calibrationQualityScore(m);
        q_txt(k) = sprintf("S%d:%d%%", k-1, round(100*q));
    end

    lim = 1.1 * max_abs;
    xlim(h3d.ax, [-lim lim]);
    ylim(h3d.ax, [-lim lim]);
    zlim(h3d.ax, [-lim lim]);

    title(h3d.ax, sprintf('De-biased/de-scaled clouds  |  Quality %s  %s  %s', ...
        q_txt(1), q_txt(2), q_txt(3)));

    for a = 1:3
        all_x = [];
        for k = 1:3
            all_x = [all_x, h2d.lines(a,k).XData]; %#ok<AGROW>
        end
        all_x = all_x(~isnan(all_x));
        if ~isempty(all_x)
            xmin = min(all_x); xmax = max(all_x);
            if xmax > xmin
                xlim(h2d.axes(a), [xmin xmax]);
            end
        end
    end
end

function printCalibrationSummary(S)
    fprintf('\n=== Calibration Summary (raw LSB domain) ===\n');
    fprintf('Note: solve calibration from RAW MAGCSV data; use calibrated data only for validation.\n');
    for k = 1:3
        n = S(k).n;
        if n < 20
            fprintf('Sensor %d (%s): not enough samples (%d)\n', ...
                k-1, char(defaultAddr(S(k).addr)), n);
            continue;
        end

        m = S(k).m(:,1:n);
        minv = min(m, [], 2);
        maxv = max(m, [], 2);
        ofs = 0.5 * (maxv + minv);
        rad = 0.5 * (maxv - minv);
        avg_rad = mean(rad);
        scl = avg_rad ./ max(rad, 1e-6);
        q = calibrationQualityScore(m);
        span_score = axisSpanScore(m);
        cons_score = calibrationConsistencyScore(m);
        v = verificationMetrics(m);

        fprintf('Sensor %d (%s)\n', k-1, char(defaultAddr(S(k).addr)));
        fprintf('  Quality score      = %.1f%%', 100*q);
        if q < 0.45
            fprintf('  (LOW: weak span and/or unstable fit)\n');
        elseif q < 0.70
            fprintf('  (OK-ish: usable, more stable span helps)\n');
        else
            fprintf('  (GOOD)\n');
        end
        fprintf('  Span score         = %.1f%%\n', 100*span_score);
        fprintf('  Consistency score  = %.1f%%\n', 100*cons_score);
        fprintf('  Offset [mx my mz] = [%.3f %.3f %.3f]\n', ofs(1), ofs(2), ofs(3));
        fprintf('  Scale  [sx sy sz] = [%.6f %.6f %.6f]\n', scl(1), scl(2), scl(3));
        fprintf('  Min    [mx my mz] = [%d %d %d]\n', minv(1), minv(2), minv(3));
        fprintf('  Max    [mx my mz] = [%d %d %d]\n', maxv(1), maxv(2), maxv(3));
        fprintf('  Verify center_err  = %.3f\n', v.center_error);
        fprintf('  Verify radius_cv   = %.3f\n', v.radius_cv);
        fprintf('  Verify axis_ratio  = %.3f\n', v.axis_ratio);
        fprintf('  Verify holdout     = %.3f\n', v.holdout_drift);
        fprintf('  Verification       = %s\n', v.verdict);
    end

    fprintf('\nPaste-ready C snippet template:\n');
    fprintf('static const float MAG_BIAS[3][3] = {\n');
    for k = 1:3
        n = S(k).n;
        if n < 20
            fprintf('    {0.0f, 0.0f, 0.0f},\n');
        else
            m = S(k).m(:,1:n);
            minv = min(m, [], 2);
            maxv = max(m, [], 2);
            ofs = 0.5 * (maxv + minv);
            fprintf('    {%.3ff, %.3ff, %.3ff},\n', ofs(1), ofs(2), ofs(3));
        end
    end
    fprintf('};\n');

    fprintf('static const float MAG_SCALE[3][3] = {\n');
    for k = 1:3
        n = S(k).n;
        if n < 20
            fprintf('    {1.0f, 1.0f, 1.0f},\n');
        else
            m = S(k).m(:,1:n);
            minv = min(m, [], 2);
            maxv = max(m, [], 2);
            rad = 0.5 * (maxv - minv);
            avg_rad = mean(rad);
            scl = avg_rad ./ max(rad, 1e-6);
            fprintf('    {%.6ff, %.6ff, %.6ff},\n', scl(1), scl(2), scl(3));
        end
    end
    fprintf('};\n');

    [pbias, pscale] = latestCalibrationPreset();
    fprintf('\nLatest saved preset in this script:\n');
    fprintf('MAG_BIAS preset =\n');
    disp(pbias);
    fprintf('MAG_SCALE preset =\n');
    disp(pscale);
end

function addr = defaultAddr(addrIn)
    if strlength(addrIn) == 0
        addr = "N/A";
    else
        addr = addrIn;
    end
end

function q = calibrationQualityScore(m)
    q = 0.6 * axisSpanScore(m) + 0.4 * calibrationConsistencyScore(m);
    q = max(0.0, min(1.0, q));
end

function s = axisSpanScore(m)
    n = size(m, 2);
    if n < 20
        s = 0.0;
        return;
    end

    minv = min(m, [], 2);
    maxv = max(m, [], 2);
    span = maxv - minv;
    span_mean = mean(span);
    if span_mean < 1e-6
        s = 0.0;
        return;
    end

    % Score isotropy of span. Best when all 3 spans are comparable.
    ratio = min(span) / max(span);
    ratio = max(0.0, min(1.0, ratio));

    % Reward absolute span relative to sensor noise floor.
    noise_floor_lsb = 30.0;
    mag = 1.0 - exp(-span_mean / noise_floor_lsb);
    mag = max(0.0, min(1.0, mag));

    s = 0.5 * ratio + 0.5 * mag;
end

function c = calibrationConsistencyScore(m)
    n = size(m, 2);
    if n < 60
        c = 0.0;
        return;
    end

    n1 = floor(n/2);
    m1 = m(:,1:n1);
    m2 = m(:,n1+1:end);

    [~, ofs1, scl1] = quickCalibrate(m1);
    [~, ofs2, scl2] = quickCalibrate(m2);

    ofs_norm = norm(ofs1 - ofs2) / max(norm(0.5*(ofs1 + ofs2)), 1e-6);
    scl_norm = norm(scl1 - scl2) / max(norm(0.5*(scl1 + scl2)), 1e-6);

    score_ofs = exp(-3.0 * ofs_norm);
    score_scl = exp(-3.0 * scl_norm);
    c = 0.5 * score_ofs + 0.5 * score_scl;
    c = max(0.0, min(1.0, c));
end

function v = verificationMetrics(m)
    v.center_error = inf;
    v.radius_cv = inf;
    v.axis_ratio = 0.0;
    v.holdout_drift = inf;
    v.verdict = "FAIL";

    n = size(m, 2);
    if n < 60
        return;
    end

    [mc, ~, ~] = quickCalibrate(m);
    r = sqrt(sum(mc.^2, 1));
    rmean = mean(r);
    if rmean < 1e-9
        return;
    end

    cvec = mean(mc, 2);
    v.center_error = norm(cvec) / rmean;
    v.radius_cv = std(r) / rmean;
    s = std(mc, 0, 2);
    v.axis_ratio = min(s) / max(max(s), 1e-9);

    n1 = floor(n/2);
    m1 = m(:,1:n1);
    m2 = m(:,n1+1:end);
    [mc1, ~, ~] = quickCalibrate(m1);
    [mc2, ~, ~] = quickCalibrate(m2);
    r1 = sqrt(sum(mc1.^2, 1));
    r2 = sqrt(sum(mc2.^2, 1));
    c1 = mean(mc1, 2);
    c2 = mean(mc2, 2);
    cm = 0.5 * (mean(r1) + mean(r2));
    if cm < 1e-9
        return;
    end
    center_d = norm(c1 - c2) / cm;
    radius_d = abs(mean(r1) - mean(r2)) / cm;
    v.holdout_drift = 0.5 * (center_d + radius_d);

    pass = (v.center_error < 0.15) && (v.radius_cv < 0.10) && ...
           (v.axis_ratio > 0.60) && (v.holdout_drift < 0.15);
    great = (v.center_error < 0.08) && (v.radius_cv < 0.06) && ...
            (v.axis_ratio > 0.75) && (v.holdout_drift < 0.08);
    marginal = (v.center_error < 0.25) && (v.radius_cv < 0.18) && ...
               (v.axis_ratio > 0.45) && (v.holdout_drift < 0.25);

    if great
        v.verdict = "PASS (great)";
    elseif pass
        v.verdict = "PASS";
    elseif marginal
        v.verdict = "MARGINAL";
    else
        v.verdict = "FAIL";
    end
end

function cov = coverageScore(m)
    % Kept for optional diagnostics, but no longer the primary metric.
    n = size(m, 2);
    if n < 20
        cov = 0.0;
        return;
    end

    [mc, ~, ~] = quickCalibrate(m);
    norms = sqrt(sum(mc.^2, 1));
    good = norms > 1e-9;
    if ~any(good)
        cov = 0.0;
        return;
    end

    u = mc(:, good) ./ norms(good);
    az = atan2(u(2,:), u(1,:));
    el = asin(max(-1.0, min(1.0, u(3,:))));

    nb_az = 18;
    nb_el = 9;
    az_edges = linspace(-pi, pi, nb_az + 1);
    el_edges = linspace(-pi/2, pi/2, nb_el + 1);
    h = histcounts2(az, el, az_edges, el_edges);

    occ = nnz(h);
    cov = occ / numel(h);
    cov = max(0.0, min(1.0, cov));
end

function [mc, ofs, scl] = quickCalibrate(m)
    minv = min(m, [], 2);
    maxv = max(m, [], 2);
    ofs = 0.5 * (maxv + minv);
    rad = 0.5 * (maxv - minv);
    avg_rad = mean(rad);
    scl = avg_rad ./ max(rad, 1e-6);
    mc = (m - ofs) .* scl;
end
