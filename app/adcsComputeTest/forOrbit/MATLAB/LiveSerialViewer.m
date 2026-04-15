%% LiveSerialViewer — Real-time ADCS telemetry from serial port
%
%  Reads the UART output from the adcsComputeTest Zephyr app, parses
%  attitude quaternion, body rates, magnetorquer dipole, and raw sensor
%  data, then displays:
%    1) A 3D cubesat model that rotates with the estimated quaternion
%    2) Live scrolling plots of quaternion components
%    3) Live scrolling plots of body rates (deg/s)
%    4) Live scrolling plots of magnetorquer dipole commands
%
%  Usage:
%    LiveSerialViewer("COM5")         % Windows
%    LiveSerialViewer("/dev/ttyUSB0") % Linux
%    LiveSerialViewer("COM5", 500)    % custom history length (samples)
%
%  Press Ctrl+C or close any figure to stop.

function LiveSerialViewer(port, history_len)
    if nargin < 2, history_len = 200; end

    % =====================================================================
    %  Serial port setup
    % =====================================================================
    s = serialport(port, 115200, "Timeout", 5);
    configureTerminator(s, "LF");
    flush(s);
    cleanupObj = onCleanup(@() delete(s));

    fprintf("Connected to %s — waiting for data...\n", port);

    % =====================================================================
    %  Data buffers (ring buffers via simple shift)
    % =====================================================================
    t_buf   = NaN(1, history_len);
    q_buf   = NaN(4, history_len);   % q0 q1 q2 q3
    w_buf   = NaN(3, history_len);   % wx wy wz  (will store in deg/s)
    mtq_buf = NaN(3, history_len);   % mtq x y z
    mag_buf = NaN(3, history_len);   % averaged raw mag (for display)

    idx = 0;
    t0  = NaN;

    % Per-cycle scratch (accumulated from multi-line cycle output)
    cur_q     = [1 0 0 0];
    cur_w     = [0 0 0];
    cur_mtq   = [0 0 0];
    cur_valid = 1;
    cur_mag   = [0 0 0];
    mag_count = 0;
    cycle_complete = false;

    % =====================================================================
    %  Build figures
    % =====================================================================

    % --- Figure 1: 3D cubesat ---
    fig3d = figure('Name','CubeSat Attitude','NumberTitle','off', ...
                   'Color','w','Position',[50 200 600 550]);
    ax3d = axes('Parent',fig3d); hold(ax3d,'on');
    axis(ax3d,'equal'); grid(ax3d,'on'); box(ax3d,'on');
    view(ax3d, [135 25]);
    lim = 0.25;
    xlim(ax3d, [-lim lim]); ylim(ax3d, [-lim lim]); zlim(ax3d, [-lim lim]);
    xlabel(ax3d,'X'); ylabel(ax3d,'Y'); zlabel(ax3d,'Z');
    title(ax3d,'Estimated Attitude','FontSize',14);
    camlight(ax3d,'headlight'); camlight(ax3d,'right');
    lighting(ax3d,'gouraud');

    % Cubesat geometry (10cm x 10cm x 30cm, centered at origin)
    W = 0.1; L = 0.1; H = 0.3;
    V_body = [ W/2  L/2  H/2;
               W/2  L/2 -H/2;
               W/2 -L/2  H/2;
               W/2 -L/2 -H/2;
              -W/2  L/2  H/2;
              -W/2  L/2 -H/2;
              -W/2 -L/2  H/2;
              -W/2 -L/2 -H/2];
    F = [1 2 4 3;   % +X
         5 6 8 7;   % -X
         1 2 6 5;   % +Y
         3 4 8 7;   % -Y
         1 3 7 5;   % +Z
         2 4 8 6];  % -Z
    face_colors = [0.85 0.15 0.15;   % +X  red
                   0.65 0.68 0.72;   % -X  grey
                   0.15 0.65 0.15;   % +Y  green (solar)
                   0.65 0.68 0.72;   % -Y  grey
                   0.15 0.15 0.85;   % +Z  blue
                   0.65 0.68 0.72];  % -Z  grey

    hT = hgtransform('Parent', ax3d);
    for p = 1:6
        patch('Vertices', V_body, 'Faces', F(p,:), ...
              'FaceColor', face_colors(p,:), 'FaceAlpha', 0.7, ...
              'EdgeColor', face_colors(p,:)*0.5, ...
              'FaceLighting','gouraud', 'LineWidth', 1.5, ...
              'Parent', hT);
    end

    % Body-frame axes (parented to hT so they rotate with the sat)
    sc = 0.2;
    quiver3(ax3d, 0,0,0, sc,0,0, 'r','LineWidth',2,'Parent',hT);
    quiver3(ax3d, 0,0,0, 0,sc,0, 'g','LineWidth',2,'Parent',hT);
    quiver3(ax3d, 0,0,0, 0,0,sc, 'b','LineWidth',2,'Parent',hT);

    % Inertial frame axes (fixed)
    quiver3(ax3d, 0,0,0, sc,0,0, 'r--','LineWidth',1);
    quiver3(ax3d, 0,0,0, 0,sc,0, 'g--','LineWidth',1);
    quiver3(ax3d, 0,0,0, 0,0,sc, 'b--','LineWidth',1);

    hTitle3d = title(ax3d, 'Estimated Attitude', 'FontSize', 14);

    % --- Figure 2: time-series plots ---
    fig2d = figure('Name','ADCS Telemetry','NumberTitle','off', ...
                   'Color','w','Position',[670 100 750 700]);

    % Quaternion subplot
    ax_q = subplot(3,1,1,'Parent',fig2d); hold(ax_q,'on'); grid(ax_q,'on');
    title(ax_q,'Attitude Quaternion'); ylabel(ax_q,'q');
    hq = gobjects(4,1);
    qnames = {'q_0','q_1','q_2','q_3'};
    qcolors = {'k','r','g','b'};
    for i = 1:4
        hq(i) = plot(ax_q, t_buf, q_buf(i,:), qcolors{i}, ...
                     'LineWidth', 1.5, 'DisplayName', qnames{i});
    end
    legend(ax_q, 'Location','eastoutside','FontSize',9);

    % Rate subplot
    ax_w = subplot(3,1,2,'Parent',fig2d); hold(ax_w,'on'); grid(ax_w,'on');
    title(ax_w,'Body Rates'); ylabel(ax_w,'\omega (deg/s)');
    hw = gobjects(3,1);
    wnames = {'\omega_x','\omega_y','\omega_z'};
    wcolors = {'r','g','b'};
    for i = 1:3
        hw(i) = plot(ax_w, t_buf, w_buf(i,:), wcolors{i}, ...
                     'LineWidth', 1.5, 'DisplayName', wnames{i});
    end
    legend(ax_w, 'Location','eastoutside','FontSize',9);

    % MTQ subplot
    ax_m = subplot(3,1,3,'Parent',fig2d); hold(ax_m,'on'); grid(ax_m,'on');
    title(ax_m,'Magnetorquer Dipole'); ylabel(ax_m,'m (A\cdotm^2)');
    xlabel(ax_m,'Time (s)');
    hm = gobjects(3,1);
    mnames = {'m_x','m_y','m_z'};
    for i = 1:3
        hm(i) = plot(ax_m, t_buf, mtq_buf(i,:), wcolors{i}, ...
                      'LineWidth', 1.5, 'DisplayName', mnames{i});
    end
    legend(ax_m, 'Location','eastoutside','FontSize',9);

    % =====================================================================
    %  Main loop — read and parse serial data line by line
    % =====================================================================
    fprintf("Streaming...\n");
    while isvalid(fig3d) && isvalid(fig2d)
        try
            line = readline(s);
        catch
            continue;
        end
        if isempty(line), continue; end
        line = char(line);

        % --- Parse "[ADCS] q: ..." ---
        toks = regexp(line, '\[ADCS\]\s+q:\s+([\-\d\.]+)\s+([\-\d\.]+)\s+([\-\d\.]+)\s+([\-\d\.]+)', 'tokens');
        if ~isempty(toks)
            cur_q = str2double(toks{1});
            continue;
        end

        % --- Parse "[ADCS] w: ..." ---
        toks = regexp(line, '\[ADCS\]\s+w:\s+([\-\d\.]+)\s+([\-\d\.]+)\s+([\-\d\.]+)', 'tokens');
        if ~isempty(toks)
            cur_w = str2double(toks{1});
            continue;
        end

        % --- Parse "[ADCS] mtq: ..." ---
        toks = regexp(line, '\[ADCS\]\s+mtq:\s+([\-\d\.]+)\s+([\-\d\.]+)\s+([\-\d\.]+)\s+valid:(\d)', 'tokens');
        if ~isempty(toks)
            cur_mtq   = str2double(toks{1}(1:3));
            cur_valid = str2double(toks{1}(4));
            cycle_complete = true;
        end

        % --- Parse "MAGn  M:..." for raw mag display ---
        toks = regexp(line, 'MAG\d\s+M:\s*([\-\d]+)\s+([\-\d]+)\s+([\-\d]+)', 'tokens');
        if ~isempty(toks)
            vals = str2double(toks{1});
            cur_mag = cur_mag + vals;
            mag_count = mag_count + 1;
            continue;
        end

        % --- Parse "--- cycle N ---" to reset per-cycle scratch ---
        toks = regexp(line, '---\s+cycle\s+(\d+)\s+---', 'tokens');
        if ~isempty(toks)
            mag_count = 0;
            cur_mag = [0 0 0];
            continue;
        end

        % --- When a full cycle's ADCS output is parsed, push to buffers ---
        if cycle_complete
            cycle_complete = false;
            idx = idx + 1;

            if isnan(t0)
                t0 = double(posixtime(datetime('now')));
            end
            t_now = double(posixtime(datetime('now'))) - t0;

            % Shift buffers left, append new sample
            t_buf   = [t_buf(2:end),   t_now];
            q_buf   = [q_buf(:,2:end), cur_q(:)];
            w_buf   = [w_buf(:,2:end), rad2deg(cur_w(:))];
            mtq_buf = [mtq_buf(:,2:end), cur_mtq(:)];
            if mag_count > 0
                mag_buf = [mag_buf(:,2:end), (cur_mag / mag_count)'];
            else
                mag_buf = [mag_buf(:,2:end), NaN(3,1)];
            end

            % =============================================================
            %  Update 3D cubesat
            % =============================================================
            q = cur_q / norm(cur_q);
            R = quat2rotm_local(q);
            M = eye(4); M(1:3,1:3) = R;
            set(hT, 'Matrix', M);

            if cur_valid
                vstr = 'VALID';
            else
                vstr = 'INVALID';
            end
            set(hTitle3d, 'String', ...
                sprintf('q=[%.3f %.3f %.3f %.3f]  %s', q(1),q(2),q(3),q(4), vstr));

            % =============================================================
            %  Update time-series plots
            % =============================================================
            for i = 1:4
                set(hq(i), 'XData', t_buf, 'YData', q_buf(i,:));
            end
            for i = 1:3
                set(hw(i), 'XData', t_buf, 'YData', w_buf(i,:));
                set(hm(i), 'XData', t_buf, 'YData', mtq_buf(i,:));
            end

            % Auto-scale x-axis to visible window
            valid = ~isnan(t_buf);
            if any(valid)
                tmin = min(t_buf(valid));
                tmax = max(t_buf(valid));
                if tmax > tmin
                    xlim(ax_q, [tmin tmax]);
                    xlim(ax_w, [tmin tmax]);
                    xlim(ax_m, [tmin tmax]);
                end
            end

            drawnow limitrate;
        end
    end

    fprintf("Viewer closed.\n");
end

% =========================================================================
%  Quaternion to rotation matrix (scalar-first: [q0 q1 q2 q3])
% =========================================================================
function R = quat2rotm_local(q)
    q = q / norm(q);
    q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
    R = [1-2*(q2^2+q3^2),   2*(q1*q2-q0*q3),  2*(q1*q3+q0*q2);
         2*(q1*q2+q0*q3),   1-2*(q1^2+q3^2),  2*(q2*q3-q0*q1);
         2*(q1*q3-q0*q2),   2*(q2*q3+q0*q1),  1-2*(q1^2+q2^2)];
end
