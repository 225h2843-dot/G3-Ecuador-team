function plot_scan(bagFile)

    if nargin < 1
        [f, p] = uigetfile('*.bag', 'Selecciona un rosbag');
        if isequal(f,0)
            disp('No se ha seleccionado archivo.');
            return;
        end
        bagFile = fullfile(p, f);
    end

    bag = rosbag(bagFile);

    % Comprobar que existe /scan
    topics = bag.AvailableTopics.Properties.RowNames;
    if ~any(strcmp('/scan', topics))
        error('El bag no tiene el topic /scan');
    end

    % Leer todos los mensajes de /scan
    bagScan  = select(bag, 'Topic', '/scan');
    scanMsgs = readMessages(bagScan, 'DataFormat', 'struct');

    % Coger un scan "representativo" (el del medio)
    idx = round(numel(scanMsgs)/2);
    idx = 1;
    s   = scanMsgs{idx};

    fprintf('Scan %d de %d\n', idx, numel(scanMsgs));
    fprintf('RangeMin = %.3f, RangeMax = %.3f\n', s.RangeMin, s.RangeMax);

    % Pasar a double y construir ángulos
    ranges = double(s.Ranges(:));  % columna
    N      = numel(ranges);
    angles = double(s.AngleMin + (0:N-1)' * s.AngleIncrement);

    % FILTRO: solo puntos válidos que ven obstáculo
    mask = isfinite(ranges) & ...
           (ranges > s.RangeMin + 1e-3) & ...
           (ranges < s.RangeMax - 1e-3);

    ranges_obs = ranges(mask);
    angles_obs = angles(mask);

    x = ranges_obs .* cos(angles_obs);
    y = ranges_obs .* sin(angles_obs);

    figure('Name','Scan láser (solo obstáculos)','NumberTitle','off');
    scatter(x, y, 10, 'filled');   % puntos sueltos, sin unir
    axis equal; grid on;
    xlabel('x [m]');
    ylabel('y [m]');
    title(sprintf('Topic /scan – %d puntos de obstáculo', numel(x)));

end