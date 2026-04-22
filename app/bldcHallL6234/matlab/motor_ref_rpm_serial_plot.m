% motor_ref_rpm_serial_plot — read MxLOG lines from UART and plot M1-M4.
%
% Firmware (main.c) prints each telemetry interval:
%   M<id>LOG,<t_ms>,<ref_rpm>,<rpm_meas>
% Example: M1LOG,12345,2000,1875
%
% Optional: set csvFile to a saved text log instead of live serial.
%
% Live serial: run this script while the board firmware is running and
% printing M1LOG lines. If the port is wrong or the MCU stops, readline can
% time out; missing/empty lines are skipped (no crash).
%
% Serial + livePlot: curves grow in real time (animatedline + drawnow limitrate).
% This script renders:
%   - 4 subplots: M1..M4 REF + Actual

%% --- user settings ---
comPort = "COM5";
baudRate = 115200;
maxLines = 10000;  % total parsed rows across motors (or use durationSec)
maxLinesPerMotor = 6000;
durationSec = 120; % serial capture window (seconds)
readTimeoutSec = 0.25; % short timeout; loop is non-blocking via NumBytesAvailable
livePlot = true;   % serial only: update plot while samples arrive (false = plot at end)
enableM1 = true;
enableM2 = true;
enableM3 = true;
enableM4 = true;

% csvFile = "motor_run_log.txt";
csvFile = "";

did_live_serial = false;

t1_ms = zeros(maxLinesPerMotor, 1);
ref1 = zeros(maxLinesPerMotor, 1);
rpm1 = zeros(maxLinesPerMotor, 1);
t2_ms = zeros(maxLinesPerMotor, 1);
ref2 = zeros(maxLinesPerMotor, 1);
rpm2 = zeros(maxLinesPerMotor, 1);
t3_ms = zeros(maxLinesPerMotor, 1);
ref3 = zeros(maxLinesPerMotor, 1);
rpm3 = zeros(maxLinesPerMotor, 1);
t4_ms = zeros(maxLinesPerMotor, 1);
ref4 = zeros(maxLinesPerMotor, 1);
rpm4 = zeros(maxLinesPerMotor, 1);
n1 = 0;
n2 = 0;
n3 = 0;
n4 = 0;

if strlength(csvFile) > 0
    txt = fileread(csvFile);
    lines = splitlines(txt);
    for i = 1:numel(lines)
        linec = local_line_to_char(lines(i));
        if isempty(linec)
            continue
        end
        [mid, tms, rref, ract, ok] = parse_mlog(linec);
        if ~ok
            continue
        end
        if mid == 1 && enableM1 && n1 < maxLinesPerMotor
            n1 = n1 + 1;
            t1_ms(n1) = tms;
            ref1(n1) = rref;
            rpm1(n1) = ract;
        elseif mid == 2 && enableM2 && n2 < maxLinesPerMotor
            n2 = n2 + 1;
            t2_ms(n2) = tms;
            ref2(n2) = rref;
            rpm2(n2) = ract;
        elseif mid == 3 && enableM3 && n3 < maxLinesPerMotor
            n3 = n3 + 1;
            t3_ms(n3) = tms;
            ref3(n3) = rref;
            rpm3(n3) = ract;
        elseif mid == 4 && enableM4 && n4 < maxLinesPerMotor
            n4 = n4 + 1;
            t4_ms(n4) = tms;
            ref4(n4) = rref;
            rpm4(n4) = ract;
        end
    end
else
    s = serialport(comPort, baudRate);
    s.Timeout = readTimeoutSec;
    configureTerminator(s, "LF");
    flush(s);

    n_total = 0;
    t0_ms = [];
    ax1 = [];
    ax2 = [];
    ax3 = [];
    ax4 = [];
    h1Ref = [];
    h1Act = [];
    h2Ref = [];
    h2Act = [];
    h3Ref = [];
    h3Act = [];
    h4Ref = [];
    h4Act = [];

    t_cap = tic;
    while n_total < maxLines && toc(t_cap) < durationSec
        if s.NumBytesAvailable == 0
            pause(0.01);
            drawnow limitrate
            continue
        end

        try
            line = readline(s);
        catch
            % Timeout or partial line; keep looping.
            continue
        end
        linec = local_line_to_char(line);
        if isempty(linec)
            continue
        end
        [mid, tms, rref, ract, ok] = parse_mlog(linec);
        if ~ok
            continue
        end
        if mid == 1 && enableM1
            if n1 >= maxLinesPerMotor
                continue
            end
            n1 = n1 + 1;
            t1_ms(n1) = tms;
            ref1(n1) = rref;
            rpm1(n1) = ract;
        elseif mid == 2 && enableM2
            if n2 >= maxLinesPerMotor
                continue
            end
            n2 = n2 + 1;
            t2_ms(n2) = tms;
            ref2(n2) = rref;
            rpm2(n2) = ract;
        elseif mid == 3 && enableM3
            if n3 >= maxLinesPerMotor
                continue
            end
            n3 = n3 + 1;
            t3_ms(n3) = tms;
            ref3(n3) = rref;
            rpm3(n3) = ract;
        elseif mid == 4 && enableM4
            if n4 >= maxLinesPerMotor
                continue
            end
            n4 = n4 + 1;
            t4_ms(n4) = tms;
            ref4(n4) = rref;
            rpm4(n4) = ract;
        else
            continue
        end
        n_total = n_total + 1;

        if livePlot
            if isempty(ax1)
                fig = figure("Name", "M1-M4 ref vs RPM (live)", "NumberTitle", "off");
                tiledlayout(fig, 4, 1, "TileSpacing", "compact", "Padding", "compact");

                ax1 = nexttile;
                hold(ax1, "on");
                grid(ax1, "on");
                ylabel(ax1, "M1 RPM");
                title(ax1, "M1 speed reference vs measured (live)");
                h1Ref = animatedline(ax1, Color = [0 0.447 0.741], LineWidth = 1.2, DisplayName = "M1 REF");
                h1Act = animatedline(ax1, Color = [0.8500 0.3250 0.0980], LineWidth = 1.2, DisplayName = "M1 Actual");
                legend(ax1, Location = "best");

                ax2 = nexttile;
                hold(ax2, "on");
                grid(ax2, "on");
                ylabel(ax2, "M2 RPM");
                title(ax2, "M2 speed reference vs measured (live)");
                h2Ref = animatedline(ax2, Color = [0 0.447 0.741], LineWidth = 1.2, DisplayName = "M2 REF");
                h2Act = animatedline(ax2, Color = [0.8500 0.3250 0.0980], LineWidth = 1.2, DisplayName = "M2 Actual");
                legend(ax2, Location = "best");

                ax3 = nexttile;
                hold(ax3, "on");
                grid(ax3, "on");
                ylabel(ax3, "M3 RPM");
                title(ax3, "M3 speed reference vs measured (live)");
                h3Ref = animatedline(ax3, Color = [0 0.447 0.741], LineWidth = 1.2, DisplayName = "M3 REF");
                h3Act = animatedline(ax3, Color = [0.8500 0.3250 0.0980], LineWidth = 1.2, DisplayName = "M3 Actual");
                legend(ax3, Location = "best");

                ax4 = nexttile;
                hold(ax4, "on");
                grid(ax4, "on");
                xlabel(ax4, "Time (s)");
                ylabel(ax4, "M4 RPM");
                title(ax4, "M4 speed reference vs measured (live)");
                h4Ref = animatedline(ax4, Color = [0 0.447 0.741], LineWidth = 1.2, DisplayName = "M4 REF");
                h4Act = animatedline(ax4, Color = [0.8500 0.3250 0.0980], LineWidth = 1.2, DisplayName = "M4 Actual");
                legend(ax4, Location = "best");

                linkaxes([ax1 ax2 ax3 ax4], "x");
            end
            if isempty(t0_ms)
                t0_ms = tms;
            end
            t_rel = (tms - t0_ms) / 1000;
            if mid == 1
                addpoints(h1Ref, t_rel, rref);
                addpoints(h1Act, t_rel, ract);
            elseif mid == 2
                addpoints(h2Ref, t_rel, rref);
                addpoints(h2Act, t_rel, ract);
            elseif mid == 3
                addpoints(h3Ref, t_rel, rref);
                addpoints(h3Act, t_rel, ract);
            else
                addpoints(h4Ref, t_rel, rref);
                addpoints(h4Act, t_rel, ract);
            end
            % Keep live view auto-fitted as data grows.
            xlim(ax1, "auto");
            ylim(ax1, "auto");
            xlim(ax2, "auto");
            ylim(ax2, "auto");
            xlim(ax3, "auto");
            ylim(ax3, "auto");
            xlim(ax4, "auto");
            ylim(ax4, "auto");
            title(ax1, sprintf("M1 speed reference vs measured (live, n=%d)", n1));
            title(ax2, sprintf("M2 speed reference vs measured (live, n=%d)", n2));
            title(ax3, sprintf("M3 speed reference vs measured (live, n=%d)", n3));
            title(ax4, sprintf("M4 speed reference vs measured (live, n=%d)", n4));
            drawnow limitrate
        end
    end
    clear s
    if livePlot && n_total > 0 && ~isempty(ax1)
        did_live_serial = true;
        title(ax1, sprintf("M1 speed reference vs measured (live, n=%d - capture ended)", n1));
        title(ax2, sprintf("M2 speed reference vs measured (live, n=%d - capture ended)", n2));
        title(ax3, sprintf("M3 speed reference vs measured (live, n=%d - capture ended)", n3));
        title(ax4, sprintf("M4 speed reference vs measured (live, n=%d - capture ended)", n4));
        drawnow
    end
end

if n1 > 0
    t1_ms = t1_ms(1:n1);
    ref1 = ref1(1:n1);
    rpm1 = rpm1(1:n1);
else
    t1_ms = zeros(0, 1);
    ref1 = zeros(0, 1);
    rpm1 = zeros(0, 1);
end
if n2 > 0
    t2_ms = t2_ms(1:n2);
    ref2 = ref2(1:n2);
    rpm2 = rpm2(1:n2);
else
    t2_ms = zeros(0, 1);
    ref2 = zeros(0, 1);
    rpm2 = zeros(0, 1);
end
if n3 > 0
    t3_ms = t3_ms(1:n3);
    ref3 = ref3(1:n3);
    rpm3 = rpm3(1:n3);
else
    t3_ms = zeros(0, 1);
    ref3 = zeros(0, 1);
    rpm3 = zeros(0, 1);
end
if n4 > 0
    t4_ms = t4_ms(1:n4);
    ref4 = ref4(1:n4);
    rpm4 = rpm4(1:n4);
else
    t4_ms = zeros(0, 1);
    ref4 = zeros(0, 1);
    rpm4 = zeros(0, 1);
end

if isempty(t1_ms) && isempty(t2_ms) && isempty(t3_ms) && isempty(t4_ms)
    error("No MxLOG rows parsed (M1-M4). Check COM port, baud, and firmware telemetry.");
end

if ~did_live_serial
    fig = figure("Name", "M1-M4 ref vs RPM", "NumberTitle", "off");
    tiledlayout(fig, 4, 1, "TileSpacing", "compact", "Padding", "compact");

    ax1 = nexttile;
    if ~isempty(t1_ms)
        t1_s = (t1_ms - t1_ms(1)) / 1000;
        plot(ax1, t1_s, ref1, "LineWidth", 1.2);
        hold(ax1, "on");
        plot(ax1, t1_s, rpm1, "LineWidth", 1.2);
    end
    grid(ax1, "on");
    ylabel(ax1, "M1 RPM");
    title(ax1, "M1 speed reference vs measured");
    legend(ax1, "M1 REF", "M1 Actual", Location = "best");

    ax2 = nexttile;
    if ~isempty(t2_ms)
        t2_s = (t2_ms - t2_ms(1)) / 1000;
        plot(ax2, t2_s, ref2, "LineWidth", 1.2);
        hold(ax2, "on");
        plot(ax2, t2_s, rpm2, "LineWidth", 1.2);
    end
    grid(ax2, "on");
    xlabel(ax2, "Time (s)");
    ylabel(ax2, "M2 RPM");
    title(ax2, "M2 speed reference vs measured");
    legend(ax2, "M2 REF", "M2 Actual", Location = "best");

    ax3 = nexttile;
    if ~isempty(t3_ms)
        t3_s = (t3_ms - t3_ms(1)) / 1000;
        plot(ax3, t3_s, ref3, "LineWidth", 1.2);
        hold(ax3, "on");
        plot(ax3, t3_s, rpm3, "LineWidth", 1.2);
    end
    grid(ax3, "on");
    ylabel(ax3, "M3 RPM");
    title(ax3, "M3 speed reference vs measured");
    legend(ax3, "M3 REF", "M3 Actual", Location = "best");

    ax4 = nexttile;
    if ~isempty(t4_ms)
        t4_s = (t4_ms - t4_ms(1)) / 1000;
        plot(ax4, t4_s, ref4, "LineWidth", 1.2);
        hold(ax4, "on");
        plot(ax4, t4_s, rpm4, "LineWidth", 1.2);
    end
    grid(ax4, "on");
    xlabel(ax4, "Time (s)");
    ylabel(ax4, "M4 RPM");
    title(ax4, "M4 speed reference vs measured");
    legend(ax4, "M4 REF", "M4 Actual", Location = "best");

    linkaxes([ax1 ax2 ax3 ax4], "x");
end

function [mid, tms, rref, ract, ok] = parse_mlog(linec)
    mid = 0;
    tms = 0;
    rref = 0;
    ract = 0;
    ok = false;
    vals = sscanf(linec, "M%dLOG,%f,%f,%f");
    if numel(vals) ~= 4
        return
    end
    mid = vals(1);
    tms = vals(2);
    rref = vals(3);
    ract = vals(4);
    ok = true;
end

function linec = local_line_to_char(line)
    linec = char.empty(0, 1);
    if isempty(line)
        return
    end
    if isa(line, "string")
        if ismissing(line)
            return
        end
        line = strtrim(line);
        if strlength(line) == 0
            return
        end
        linec = char(line);
        return
    end
    if ischar(line)
        linec = strtrim(line(:).');
        return
    end
    if iscell(line) && ~isempty(line) && (ischar(line{1}) || isa(line{1}, "string"))
        linec = local_line_to_char(line{1});
    end
end
