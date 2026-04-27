%% LiveSerialViewer — real-time ADCS telemetry from serial port
%
%  Reads UART output from adcsComputeTest (built without -DCAL_EMIT) and
%  shows live plots:
%    1) 3D cubesat model rotated by the estimated attitude
%    2) Stacked 4-row time-series: attitude q, body rates, accelerometer,
%       observed magnetic field
%
%  Usage:
%    LiveSerialViewer("COM5")
%    LiveSerialViewer("COM5", 500)
%    LiveSerialViewer("COM5", 'attitude', 'triad')
%    LiveSerialViewer("COM5", 'attitude', 'triad', ...
%                     'mag_ref', [22.5e-6 1.5e-6 41e-6], ...
%                     'gravity_ref', [0 0 -9.80665])
%
%  Options:
%    history_len   (positional)   ring-buffer length, default 200
%    attitude      'firmware'|'triad'   default 'firmware'
%                  'firmware' uses [ADCS] q: ... straight from the observer
%                  'triad'    runs MATLAB-side TRIAD on the latest calibrated
%                             accel + mag every cycle. Independent of the
%                             firmware observer's startup pose, so the digital
%                             cubesat shows absolute orientation. Requires
%                             properly calibrated mag/accel.
%    gravity_ref   inertial gravity vector [m/s^2], default [0 0 -9.80665]
%    mag_ref       inertial mag field [T], default Austin-area NED-ish
%    plot_period   redraw throttle in seconds, default 0.05 (~20 fps)
%
%  Close any figure to stop.


function LiveSerialViewer(port, varargin)

close all
    % ---- args -----------------------------------------------------------
    p = inputParser;
    p.KeepUnmatched = false;
    addOptional (p, 'history_len', 200, @(x) isnumeric(x) && x > 1);
    addParameter(p, 'attitude',    'firmware', @(x) any(strcmpi(x, {'firmware','triad'})));
    addParameter(p, 'gravity_ref', [0 0 -9.80665], @(x) isnumeric(x) && numel(x)==3);
    addParameter(p, 'mag_ref',     [22.156e-6 4.1909e-6 42.892e-6], @(x) isnumeric(x) && numel(x)==3);
    addParameter(p, 'plot_period', 0.05, @(x) isnumeric(x) && x > 0);
    addParameter(p, 'baud',        115200, @isnumeric);
    parse(p, varargin{:});
    R = p.Results;
    N = R.history_len;
    use_triad = strcmpi(R.attitude, 'triad');
    g_ref = R.gravity_ref(:);
    b_ref = R.mag_ref(:);

    % ---- serial port -----------------------------------------------------
    s = serialport(port, R.baud, "Timeout", 5);
    configureTerminator(s, "LF");
    flush(s);
    cleanupObj = onCleanup(@() delete(s)); %#ok<NASGU>
    fprintf("Connected to %s. Attitude source: %s.\n", port, R.attitude);

    % ---- ring buffers (pointer; no shift) -------------------------------
    t_buf = nan(1, N);
    q_buf = nan(4, N);
    w_buf = nan(3, N);   % deg/s for plot
    a_buf = nan(3, N);   % m/s^2
    b_buf = nan(3, N);   % T (plotted in uT)
    ptr   = 0;
    filled = false;

    % per-cycle scratch
    cur_q     = [1 0 0 0];
    cur_w     = [0 0 0];
    cur_a     = [0 0 0];
    cur_b     = [0 0 0];
    cur_valid = 1;
    cycle_complete = false;

    % ---- figures ---------------------------------------------------------
    [fig3d, fig2d, hT, hTitle3d, hq, hw, ha, hb, ax_q, ax_w, ax_a, ax_b] = ...
        buildFigures(N);

    % ---- timing ---------------------------------------------------------
    t_clock = tic;
    last_draw = tic;

    fprintf("Streaming...\n");
    while isvalid(fig3d) && isvalid(fig2d)
        try
            line = readline(s);
        catch
            continue;
        end
        if line == "", continue; end
        line = char(line);

        % Cheap prefix dispatch — most lines aren't [ADCS] telemetry.
        if numel(line) < 9 || line(1) ~= '['
            continue;
        end

        if startsWith(line, '[ADCS] q:')
            nums = sscanf(line, '[ADCS] q: %f %f %f %f');
            if numel(nums) == 4
                cur_q = nums.';
                cur_q(2) = -cur_q(2);  % flip X component to fix roll sign
            end
        elseif startsWith(line, '[ADCS] w:')
            nums = sscanf(line, '[ADCS] w: %f %f %f');
            if numel(nums) == 3, cur_w = nums.'; end
        elseif startsWith(line, '[ADCS] a [m/s^2]:')
            nums = sscanf(line, '[ADCS] a [m/s^2]: %f %f %f');
            if numel(nums) == 3, cur_a = nums.'; end
        elseif startsWith(line, '[ADCS] B [T]:')
            nums = sscanf(line, '[ADCS] B [T]: %f %f %f');
            if numel(nums) == 3, cur_b = nums.'; end
        elseif startsWith(line, '[ADCS] mode=')
            cur_valid = ~isempty(strfind(line, 'valid:1')); %#ok<STREMP>
        elseif startsWith(line, '[ADCS] mtq')
            % mtq is the last [ADCS] line per cycle. Use it as the cycle
            % terminator even though we no longer plot it.
            cycle_complete = true;
        else
            continue;
        end

        if ~cycle_complete, continue; end
        cycle_complete = false;

        % ---- compute attitude source ------------------------------------
        if use_triad
            q_disp = triadQuat(cur_a(:), cur_b(:), g_ref, b_ref, cur_q);
            q_disp(2) = -q_disp(2);  % flip X component to fix roll sign
        else
            q_disp = cur_q;
        end
        qn = norm(q_disp);
        if qn > 0, q_disp = q_disp / qn; else, q_disp = [1 0 0 0]; end

        % ---- write into ring buffer -------------------------------------
        if ptr >= N
            ptr = 1; filled = true;
        else
            ptr = ptr + 1;
        end
        t_buf(ptr)    = toc(t_clock);
        q_buf(:, ptr) = q_disp(:);
        w_buf(:, ptr) = rad2deg(cur_w(:));
        a_buf(:, ptr) = cur_a(:);
        b_buf(:, ptr) = cur_b(:);

        % ---- update 3D cubesat (cheap; just an hgtransform matrix) ------
        Rm = quat2rotm_local(q_disp);
        M = eye(4); M(1:3, 1:3) = Rm;
        set(hT, 'Matrix', M);

        % ---- throttle the heavy 2D redraw -------------------------------
        if toc(last_draw) < R.plot_period
            continue;
        end
        last_draw = tic;

        if filled
            ord = [ptr+1:N, 1:ptr];
        else
            ord = 1:ptr;
        end
        tt = t_buf(ord);

        % Batched data update: each call updates all lines on one axes
        set(hq, {'XData','YData'}, [{tt, q_buf(1, ord)}; {tt, q_buf(2, ord)}; ...
                                    {tt, q_buf(3, ord)}; {tt, q_buf(4, ord)}]);
        set(hw, {'XData','YData'}, [{tt, w_buf(1, ord)}; {tt, w_buf(2, ord)}; {tt, w_buf(3, ord)}]);
        set(ha, {'XData','YData'}, [{tt, a_buf(1, ord)}; {tt, a_buf(2, ord)}; {tt, a_buf(3, ord)}]);
        set(hb, {'XData','YData'}, [{tt, b_buf(1, ord)*1e6}; {tt, b_buf(2, ord)*1e6}; {tt, b_buf(3, ord)*1e6}]);

        if cur_valid, vstr = 'VALID'; else, vstr = 'INVALID'; end
        set(hTitle3d, 'String', sprintf('q=[%.3f %.3f %.3f %.3f]  %s  (%s)', ...
                                        q_disp(1), q_disp(2), q_disp(3), q_disp(4), ...
                                        vstr, R.attitude));

        if ~isnan(tt(1)) && tt(end) > tt(1)
            xlim(ax_q, [tt(1) tt(end)]);
            xlim(ax_w, [tt(1) tt(end)]);
            xlim(ax_a, [tt(1) tt(end)]);
            xlim(ax_b, [tt(1) tt(end)]);
        end

        drawnow limitrate;
    end

    fprintf("Viewer closed.\n");
end

% =====================================================================
% Figure construction
% =====================================================================
function [fig3d, fig2d, hT, hTitle3d, hq, hw, ha, hb, ax_q, ax_w, ax_a, ax_b] = buildFigures(N)
    % --- 3D cubesat figure ---
    fig3d = figure('Name','CubeSat Attitude','NumberTitle','off', ...
                   'Color','w','Position',[50 200 600 550]);
    ax3d = axes('Parent',fig3d); hold(ax3d,'on');
    axis(ax3d,'equal'); grid(ax3d,'on'); box(ax3d,'on');
    view(ax3d, [45 25]);   % was [135 25] — rotate camera −90° around 
    % Flip the X and Y display axes (Z preserved) so the rendered cubesat
    % matches the room frame seen on the air-bearing bench.
    lim = 0.25;
    xlim(ax3d, [-lim lim]); ylim(ax3d, [-lim lim]); zlim(ax3d, [-lim lim]);

xlabel(ax3d, 'Y');     % was 'X'
ylabel(ax3d, 'X');     % was 'Y'
zlabel(ax3d, 'Z');
    title(ax3d,'Estimated Attitude','FontSize',14);
    camlight(ax3d,'headlight'); camlight(ax3d,'right');
    lighting(ax3d,'gouraud');
    
    W = 0.1; L = 0.1; H = 0.3;
    V_body = [ W/2  L/2  H/2;
               W/2  L/2 -H/2;
               W/2 -L/2  H/2;
               W/2 -L/2 -H/2;
              -W/2  L/2  H/2;
              -W/2  L/2 -H/2;
              -W/2 -L/2  H/2;
              -W/2 -L/2 -H/2];
    F = [1 2 4 3; 5 6 8 7; 1 2 6 5; 3 4 8 7; 1 3 7 5; 2 4 8 6];
    face_colors = [0.85 0.15 0.15;
                   0.65 0.68 0.72;
                   0.15 0.65 0.15;
                   0.65 0.68 0.72;
                   0.15 0.15 0.85;
                   0.65 0.68 0.72];

    hT = hgtransform('Parent', ax3d);
    for p = 1:6
        patch('Vertices', V_body, 'Faces', F(p,:), ...
              'FaceColor', face_colors(p,:), 'FaceAlpha', 0.7, ...
              'EdgeColor', face_colors(p,:)*0.5, ...
              'FaceLighting','gouraud', 'LineWidth', 1.5, ...
              'Parent', hT);
    end
    sc = 0.2;
    quiver3(ax3d, 0,0,0, sc,0,0, 'r','LineWidth',2,'Parent',hT);
    quiver3(ax3d, 0,0,0, 0,sc,0, 'g','LineWidth',2,'Parent',hT);
    quiver3(ax3d, 0,0,0, 0,0,sc, 'b','LineWidth',2,'Parent',hT);
    quiver3(ax3d, 0,0,0, sc,0,0, 'r--','LineWidth',1);
    quiver3(ax3d, 0,0,0, 0,sc,0, 'g--','LineWidth',1);
    quiver3(ax3d, 0,0,0, 0,0,sc, 'b--','LineWidth',1);
    hTitle3d = title(ax3d, 'Estimated Attitude', 'FontSize', 14);

    % --- 2D figure: attitude / rates / accel / mag ---
    fig2d = figure('Name','ADCS Telemetry','NumberTitle','off', ...
                   'Color','w','Position',[670 80 820 860]);
    nan_x = nan(1, N);

    ax_q = subplot(4,1,1,'Parent',fig2d); hold(ax_q,'on'); grid(ax_q,'on');
    title(ax_q,'Attitude Quaternion'); ylabel(ax_q,'q');
    qcolors = {'k','r','g','b'};
    qnames  = {'q_0','q_1','q_2','q_3'};
    hq = gobjects(4,1);
    for i = 1:4
        hq(i) = plot(ax_q, nan_x, nan_x, qcolors{i}, ...
                     'LineWidth', 1.2, 'DisplayName', qnames{i});
    end
    legend(ax_q, 'Location','eastoutside','FontSize',9);

    xyzc = {'r','g','b'};

    ax_w = subplot(4,1,2,'Parent',fig2d); hold(ax_w,'on'); grid(ax_w,'on');
    title(ax_w,'Body Rates'); ylabel(ax_w,'\omega (deg/s)');
    wnames = {'\omega_x','\omega_y','\omega_z'};
    hw = gobjects(3,1);
    for i = 1:3
        hw(i) = plot(ax_w, nan_x, nan_x, xyzc{i}, ...
                     'LineWidth', 1.2, 'DisplayName', wnames{i});
    end
    legend(ax_w, 'Location','eastoutside','FontSize',9);

    ax_a = subplot(4,1,3,'Parent',fig2d); hold(ax_a,'on'); grid(ax_a,'on');
    title(ax_a,'Accelerometer'); ylabel(ax_a,'a (m/s^2)');
    anames = {'a_x','a_y','a_z'};
    ha = gobjects(3,1);
    for i = 1:3
        ha(i) = plot(ax_a, nan_x, nan_x, xyzc{i}, ...
                     'LineWidth', 1.2, 'DisplayName', anames{i});
    end
    legend(ax_a, 'Location','eastoutside','FontSize',9);

    ax_b = subplot(4,1,4,'Parent',fig2d); hold(ax_b,'on'); grid(ax_b,'on');
    title(ax_b,'Observed Magnetic Field'); ylabel(ax_b,'B (\muT)');
    xlabel(ax_b,'Time (s)');
    bnames = {'B_x','B_y','B_z'};
    hb = gobjects(3,1);
    for i = 1:3
        hb(i) = plot(ax_b, nan_x, nan_x, xyzc{i}, ...
                     'LineWidth', 1.2, 'DisplayName', bnames{i});
    end
    legend(ax_b, 'Location','eastoutside','FontSize',9);
end

% =====================================================================
% TRIAD attitude from gravity + magnetic field
% =====================================================================
function q = triadQuat(a_body, b_body, g_inertial, b_inertial, q_fallback)
    % Returns scalar-first quaternion mapping body to inertial.
    % If accel or mag are degenerate (near-zero, parallel), falls back to
    % the firmware quaternion to avoid NaN dropouts.
    na = norm(a_body); nb = norm(b_body);
    if na < 1e-3 || nb < 1e-12
        q = q_fallback; return;
    end
    % Body sees specific force = -gravity. Flip so v1b is gravity direction.
    v1b = -a_body / na;
    v2b =  b_body / nb;
    v1i = g_inertial / norm(g_inertial);
    v2i = b_inertial / norm(b_inertial);

    cb = cross(v1b, v2b); ncb = norm(cb);
    ci = cross(v1i, v2i); nci = norm(ci);
    if ncb < 1e-6 || nci < 1e-6
        q = q_fallback; return;
    end
    t1b = v1b;            t1i = v1i;
    t2b = cb / ncb;       t2i = ci / nci;
    t3b = cross(t1b, t2b);t3i = cross(t1i, t2i);

    % R takes body coords to inertial: v_i = R * v_b
    Rm = [t1i t2i t3i] * [t1b t2b t3b].';
    q = rotm2quat_local(Rm);
end

function q = rotm2quat_local(Rm)
    tr = Rm(1,1) + Rm(2,2) + Rm(3,3);
    if tr > 0
        s = sqrt(tr + 1.0) * 2;
        q0 = 0.25 * s;
        q1 = (Rm(3,2) - Rm(2,3)) / s;
        q2 = (Rm(1,3) - Rm(3,1)) / s;
        q3 = (Rm(2,1) - Rm(1,2)) / s;
    elseif (Rm(1,1) > Rm(2,2)) && (Rm(1,1) > Rm(3,3))
        s = sqrt(1.0 + Rm(1,1) - Rm(2,2) - Rm(3,3)) * 2;
        q0 = (Rm(3,2) - Rm(2,3)) / s;
        q1 = 0.25 * s;
        q2 = (Rm(1,2) + Rm(2,1)) / s;
        q3 = (Rm(1,3) + Rm(3,1)) / s;
    elseif Rm(2,2) > Rm(3,3)
        s = sqrt(1.0 + Rm(2,2) - Rm(1,1) - Rm(3,3)) * 2;
        q0 = (Rm(1,3) - Rm(3,1)) / s;
        q1 = (Rm(1,2) + Rm(2,1)) / s;
        q2 = 0.25 * s;
        q3 = (Rm(2,3) + Rm(3,2)) / s;
    else
        s = sqrt(1.0 + Rm(3,3) - Rm(1,1) - Rm(2,2)) * 2;
        q0 = (Rm(2,1) - Rm(1,2)) / s;
        q1 = (Rm(1,3) + Rm(3,1)) / s;
        q2 = (Rm(2,3) + Rm(3,2)) / s;
        q3 = 0.25 * s;
    end
    q = [q0 q1 q2 q3];
end

function Rm = quat2rotm_local(q)
    q = q / norm(q);
    q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
    Rm = [1-2*(q2^2+q3^2),   2*(q1*q2-q0*q3),  2*(q1*q3+q0*q2);
          2*(q1*q2+q0*q3),   1-2*(q1^2+q3^2),  2*(q2*q3-q0*q1);
          2*(q1*q3-q0*q2),   2*(q2*q3+q0*q1),  1-2*(q1^2+q2^2)];
end
