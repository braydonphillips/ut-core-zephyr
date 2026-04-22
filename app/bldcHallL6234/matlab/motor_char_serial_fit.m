% motor_char_serial_fit — capture UART CSV from bldcHallL6234 characterization
% firmware and estimate rpm ≈ k * duty (duty in percent, rpm magnitude).
%
% Prerequisites:
%   - Flash the app built with -DBLDC_MOTOR_CHARACTERIZATION=y
%   - Match BaudRate to your board console (often 115200).
%
% UART format (one sample per line):
%   duty_pct,rpm_abs
% Lines starting with '#' are comments (skipped).

%% --- user settings ---
comPort = "COM5";      % Windows: "COM3"; macOS: "/dev/cu.usbserial-*"
baudRate = 115200;
maxLines = 1200;     % safety cap (sweep is 951 data rows)
fitDutyMinPct = 1.0; % exclude low-duty stall/noise from k fit (set 0 to use all)

%% --- optional: load from saved log instead of serial ---
% csvFile = "motor_char_log.csv";
csvFile = "";

if strlength(csvFile) > 0
    M = readmatrix(csvFile, CommentStyle = "#");
    duty = M(:, 1);
    rpm = M(:, 2);
else
    s = serialport(comPort, baudRate);
    configureTerminator(s, "LF");
    flush(s);

    duty = zeros(maxLines, 1);
    rpm = zeros(maxLines, 1);
    n = 0;
    tStart = tic;
    while n < maxLines && toc(tStart) < 1200
        line = strtrim(readline(s));
        if strlength(line) == 0
            continue
        end
        if startsWith(line, "# end")
            break
        end
        if startsWith(line, "#")
            continue
        end
        vals = sscanf(line, "%f,%f");
        if numel(vals) ~= 2
            continue
        end
        n = n + 1;
        duty(n) = vals(1);
        rpm(n) = vals(2);
    end
    clear s
    duty = duty(1:n);
    rpm = rpm(1:n);
end

if isempty(duty)
    error("No CSV rows parsed. Check COM port, baud, and that the sweep firmware is running.");
end

%% --- fit rpm = k * duty (least squares through origin) ---
m = duty >= fitDutyMinPct;
d = duty(m);
r = rpm(m);
if isempty(d)
    error("No samples left after fitDutyMinPct filter.");
end

k = (d' * d) \ (d' * r);
% Equivalent: k = d \ r;

rpm_hat = k * duty;
res = rpm - rpm_hat;
rmse = sqrt(mean(res.^2));

fprintf("Fit on %d points (duty >= %.2f %%): k = %.6f rpm / (%% duty)\n", ...
    numel(d), fitDutyMinPct, k);
fprintf("Model: rpm = duty * %.6f   (duty in percent, 0..100)\n", k);
fprintf("RMSE (all parsed points): %.3f rpm\n", rmse);

figure;
scatter(duty, rpm, 8, "filled");
hold on;
plot([0; max(duty)], [0; k * max(duty)], "LineWidth", 1.5);
grid on;
xlabel("Duty (%)");
ylabel("|RPM|");
title(sprintf("Motor characterization: rpm = k * duty,  k = %.4f rpm/%%", k));
legend("Samples", "rpm = k * duty", Location = "northwest");
