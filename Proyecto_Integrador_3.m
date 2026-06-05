clc; clear; close all;

%%%%%%%%%%% 
% ----------------------------------
% CARGA DE DATOS DEL ROSBAG | Robot MiR100 %
% ----------------------------------
%%%%%%%%%%%

    %----------------------------
    % - Cargar  el archivo rosbag 
    %----------------------------
[f, p] = uigetfile('*.bag', 'Selecciona un rosbag');
if isequal(f, 0)
    disp('No se ha seleccionado archivo.');
    return;
end

bagFile = fullfile(p, f);
fprintf('Cargando rosbag: %s\n', bagFile);
bag = rosbag(bagFile);

fprintf('\nTopics disponibles:\n');
disp(bag.AvailableTopics.Properties.RowNames);

hasTopic = @(name) any(strcmp(name, bag.AvailableTopics.Properties.RowNames));
timeFromHeader = @(h) double(h.Stamp.Sec) + double(h.Stamp.Nsec)*1e-9;
quatToYaw = @(q) atan2(2*(q.W*q.Z + q.X*q.Y), 1 - 2*(q.Y^2 + q.Z^2));

%----------------------------
% 1) ODOMETRÍA ( /odom )
%----------------------------

odom = [];
if hasTopic('/odom')
    bagOdom = select(bag, 'Topic', '/odom');
    odomMsgs = readMessages(bagOdom, 'DataFormat', 'struct');

    n = numel(odomMsgs);
    odom.t = zeros(n,1);
    odom.x = zeros(n,1);
    odom.y = zeros(n,1);
    odom.yaw = zeros(n,1);

    for k = 1:n
        m = odomMsgs{k};
        odom.t(k) = timeFromHeader(m.Header);
        odom.x(k) = m.Pose.Pose.Position.X;
        odom.y(k) = m.Pose.Pose.Position.Y;
        odom.yaw(k) = quatToYaw(m.Pose.Pose.Orientation);
    end

    fprintf('Leídos %d mensajes de /odom\n', n);
else
    error('No se encontró /odom en el bag.');
end

%----------------------------
% 2) LÁSER ( /scan )
%----------------------------

scan = [];
if hasTopic('/scan')
    bagScan = select(bag, 'Topic', '/scan');
    scanMsgs = readMessages(bagScan, 'DataFormat', 'struct');

    n = numel(scanMsgs);
    scan.t = zeros(n,1);
    scan.ranges = cell(n,1);
    scan.angles = cell(n,1);

    rangeMin = double(scanMsgs{1}.RangeMin);
    rangeMax = double(scanMsgs{1}.RangeMax);
    maxRange = rangeMax;

    for k = 1:n
        m = scanMsgs{k};
        ranges_k = double(m.Ranges(:));
        N = numel(ranges_k);

        scan.t(k) = timeFromHeader(m.Header);
        scan.ranges{k} = ranges_k;
        scan.angles{k} = double(m.AngleMin) + (0:N-1)' * double(m.AngleIncrement);
    end

    fprintf('Leídos %d mensajes de /scan\n', n);
else
    error('No se encontró /scan en el bag.');
end

%----------------------------
% 3) IMU ( /imu_data )
%----------------------------

imu = [];
if hasTopic('/imu_data')
    bagIMU = select(bag, 'Topic', '/imu_data');
    imuMsgs = readMessages(bagIMU, 'DataFormat', 'struct');

    n = numel(imuMsgs);
    imu.t = zeros(n,1);
    imu.yaw = zeros(n,1);

    for k = 1:n
        m = imuMsgs{k};
        imu.t(k) = timeFromHeader(m.Header);
        imu.yaw(k) = quatToYaw(m.Orientation);
    end

    fprintf('Leídos %d mensajes de /imu_data\n', n);
else
    warning('No se encontró /imu_data. Se usará yaw de /odom.');
end


%%%%%%%%%%% 
% ----------------------------------------------
% SINCRONIZACIONES Y TRANSFORMACIONES DE DATOS 
% ----------------------------------------------
%%%%%%%%%%%

%--------------------------------------------
% Transformación LiDAR a frame de base_link 
%--------------------------------------------

laser_tf = [];
if hasTopic('/tf_static')
    bagTFs = select(bag, 'Topic', '/tf_static');
    tfMsgs = readMessages(bagTFs, 'DataFormat', 'struct');

    for k = 1:numel(tfMsgs)
        tfs = tfMsgs{k}.Transforms;
        for i = 1:numel(tfs)
            tf = tfs(i);
            if strcmp(tf.Header.FrameId, 'base_link') && contains(tf.ChildFrameId, 'laser')
                laser_tf = tf;
                break;
            end
        end
        if ~isempty(laser_tf)
            break;
        end
    end
end


%-------------------------------------------------
% Extracción desplazamiento y rotación del láser 
%-------------------------------------------------

if isempty(laser_tf)
    error('No se encontró transformación base_link -> laser en /tf_static.');
end

lx = double(laser_tf.Transform.Translation.X);
ly = double(laser_tf.Transform.Translation.Y);
qLaser = laser_tf.Transform.Rotation;
laserYaw = atan2(2*(qLaser.W*qLaser.Z + qLaser.X*qLaser.Y), 1 - 2*(qLaser.Y^2 + qLaser.Z^2));
%-------------------------------
% Sincronización de IMU con Odom
%-------------------------------

% Para la orientación del robot, vamos a utilizar la orientación del IMU
% * Más precisa
% * Menos error angular
% * Menos deriva
if ~isempty(imu)
    odom.yaw = interp1(double(imu.t), double(imu.yaw), double(odom.t), 'nearest', 'extrap');
    fprintf('Sustituido yaw de odometría por IMU.\n');
end
%----------------------------
% Sincronización LiDAR /scan con odom /odom
%----------------------------
scan.odom_idx = zeros(length(scan.t),1);
for k = 1:length(scan.t)
    [~, idx] = min(abs(odom.t - scan.t(k)));
    scan.odom_idx(k) = idx;
end
%%

%%%%%%%%%%% 
% ----------------------------------------------
% CREACIÓN MAPA DE OCUPACIÓN 
% ----------------------------------------------
%%%%%%%%%%%


%%%%%%%%%%%
% Determinar límites del mapa a partir de la odometría
%%%%%%%%%%%

% colocamos margenes de seguridad para evitar errores de hallar la pose
% fuera del mapa 

margin = 2.0;
xmin = min(odom.x) - margin;
xmax = max(odom.x) + margin;
ymin = min(odom.y) - margin;
ymax = max(odom.y) + margin;

mapWidth = xmax - xmin;
mapHeight = ymax - ymin;

%%%%%%%%%%%
% Creación del mapa de ocupación, definición de su resolución y
% alineación del frame
%%%%%%%%%%%

mapResolution = 10;

fprintf('Límites del mapa generado:\n');
fprintf('X: [%.2f, %.2f]\n', xmin, xmax);
fprintf('Y: [%.2f, %.2f]\n', ymin, ymax);

map = occupancyMap(mapWidth, mapHeight, mapResolution);
map.GridLocationInWorld = [xmin, ymin];

for k = 1:length(scan.t)
    idx = scan.odom_idx(k);
    theta = odom.yaw(idx);

    x_laser = odom.x(idx) + cos(theta)*lx - sin(theta)*ly;
    y_laser = odom.y(idx) + sin(theta)*lx + cos(theta)*ly;
    laserPose = [x_laser, y_laser, theta + laserYaw];

    ranges = double(scan.ranges{k}(:));
    angles = double(scan.angles{k}(:));

    ranges(ranges > maxRange) = maxRange;
    ranges(ranges < rangeMin) = NaN;

    N = min(numel(ranges), numel(angles));
    insertRay(map, laserPose, ranges(1:N), angles(1:N), maxRange);
end

map_real = [];

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%% 
% ----------------------------------------------
% ANÁLISIS DE RESULTADOS | GRÁFICAS REALIZADAS
% ----------------------------------------------
%%%%%%%%%%%

%%%%%%%%%%%
% Extracción, reconstrucción y gráfica del mapa real | Mapa del rosbag
%%%%%%%%%%%


if hasTopic('/map')
    bagMap = select(bag, 'Topic', '/map');
    mapMsgs = readMessages(bagMap, 'DataFormat', 'struct');

    m = mapMsgs{1};
    mapW = double(m.Info.Width);
    mapH = double(m.Info.Height);
    res = double(m.Info.Resolution);

    mapData = reshape(m.Data, mapW, mapH);
    mapData = flipud(mapData');

    mapDataNorm = double(mapData);
    mapDataNorm(mapData == -1) = NaN;
    mapDataNorm(mapData == 0) = 0;
    mapDataNorm(mapData == 100) = 1;

    map_real = occupancyMap(mapDataNorm, 1/res);
    map_real.GridLocationInWorld = [double(m.Info.Origin.Position.X), double(m.Info.Origin.Position.Y)];
end

figure('Name','Mapa generado por scan','NumberTitle','off');
show(map);
axis equal;
title('Mapa generado a partir de odometría + LiDAR');
xlabel('x [m]');
ylabel('y [m]');

if ~isempty(map_real)
    occ_gen = occupancyMatrix(map);
    occ_real = occupancyMatrix(map_real);
    occ_real_resized = imresize(occ_real, size(occ_gen), 'nearest');

    mask = isnan(occ_gen) | isnan(occ_real_resized);
    A = occ_gen;
    B = occ_real_resized;
    A(mask) = [];
    B(mask) = [];

    rmse_mapa = sqrt(mean((A(:) - B(:)).^2));
    A_bin = A > 0.65;
    B_bin = B > 0.65;
    coincidencia_celdas = sum(A_bin(:) == B_bin(:)) / numel(A_bin);

    disp(['Error entre mapas: ', num2str(rmse_mapa)]);
    disp(['Coincidencia de ocupación: ', num2str(coincidencia_celdas)]);
end

occ_gen = occupancyMatrix(map);
celdas_desconocidas = sum(occ_gen(:) > 0.45 & occ_gen(:) < 0.55) / numel(occ_gen);
disp(['Celdas desconocidas: ', num2str(celdas_desconocidas)]);

startWorld = [1.0, 2.0];
goalWorld = [7, -8];

if checkOccupancy(map, startWorld) > 0.65
    error('El punto inicial está en una celda ocupada.');
end

if checkOccupancy(map, goalWorld) > 0.65
    error('El punto meta está en una celda ocupada.');
end

startGrid = world2grid(map, startWorld);
goalGrid = world2grid(map, goalWorld);

planner = plannerAStarGrid(map);
pathGrid = plan(planner, startGrid, goalGrid);
pathWorld = grid2world(map, pathGrid);

figure('Name','Trayectoria A*','NumberTitle','off');
show(map);
hold on;
plot(pathWorld(:,1), pathWorld(:,2), 'r', 'LineWidth', 2);
plot(startWorld(1), startWorld(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(goalWorld(1), goalWorld(2), 'bx', 'MarkerSize', 10, 'LineWidth', 2);
legend('Ruta A*', 'Inicio', 'Meta');
title('Trayectoria generada con A*');
axis equal;
hold off;

mapDynamic = copy(map);
mapDynamic.OccupiedThreshold = 0.65;
mapDynamic.FreeThreshold = 0.2;

obsIdx = round(size(pathWorld,1)/2);
obstacleCenter = pathWorld(obsIdx,:);
obstacleRadius = 0.45;

occ = occupancyMatrix(mapDynamic);
[rows, cols] = size(occ);
[rowGrid, colGrid] = ndgrid(1:rows, 1:cols);
worldPts = grid2world(mapDynamic, [rowGrid(:), colGrid(:)]);

distToObs = vecnorm(worldPts - obstacleCenter, 2, 2);
insideObs = reshape(distToObs <= obstacleRadius, rows, cols);
occ(insideObs) = 0.85;

mapDynamic = occupancyMap(occ, mapResolution);
mapDynamic.GridLocationInWorld = map.GridLocationInWorld;
mapDynamic.OccupiedThreshold = 0.65;
mapDynamic.FreeThreshold = 0.2;

gridObs = world2grid(mapDynamic, obstacleCenter);
fprintf('Obstáculo artificial en mundo: [%.2f, %.2f]\n', obstacleCenter(1), obstacleCenter(2));
fprintf('Obstáculo artificial en grilla: fila %d, columna %d\n', gridObs(1), gridObs(2));
occDyn = occupancyMatrix(mapDynamic);
fprintf('Ocupación en el centro del obstáculo: %.3f\n', occDyn(gridObs(1), gridObs(2)));

pp = controllerPurePursuit;
pp.Waypoints = pathWorld;
pp.DesiredLinearVelocity = 0.15;
pp.MaxAngularVelocity = 1.0;
pp.LookaheadDistance = 0.5;

obstacleThreshold = 0.8;
vAvoid = 0.12;
kAvoid = 1.5;

dt = 0.1;
goalTol = 0.20;
maxSteps = 3000;

robotPose = [startWorld(1), startWorld(2), pi/2];
robotPath = robotPose(1:2);

lidarAngles = linspace(-pi/2,pi/2,181)';

figure('Name','Seguimiento de ruta con Pure Pursuit + VFH','NumberTitle','off');
show(mapDynamic);
hold on;
plot(pathWorld(:,1), pathWorld(:,2), 'r--', 'LineWidth', 1.5);
plot(startWorld(1), startWorld(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(goalWorld(1), goalWorld(2), 'bx', 'MarkerSize', 10, 'LineWidth', 2);
plot(obstacleCenter(1), obstacleCenter(2), 'mo', 'MarkerSize', 10, 'LineWidth', 2);
hRobot = plot(robotPose(1), robotPose(2), 'ko', 'MarkerSize', 8, 'LineWidth', 2);
hPath = plot(robotPath(:,1), robotPath(:,2), 'b', 'LineWidth', 2);
legend('Ruta A*', 'Inicio', 'Meta', 'Obstáculo', 'Robot', 'Trayectoria seguida');
title('Robot siguiendo ruta A* con Pure Pursuit + VFH');
axis equal;

avoidState = 0;

avoidStartPose = robotPose(1:2);

avoidStartTheta = robotPose(3);

routeReplanned = false;

for step = 1:maxSteps
    theta = robotPose(3);

    x_laser = robotPose(1) + cos(theta)*lx - sin(theta)*ly;
    y_laser = robotPose(2) + sin(theta)*lx + cos(theta)*ly;

    laserPoseSim = robotPose;%[x_laser, y_laser, theta];

    lidarRangesRaw = rayIntersection(mapDynamic, laserPoseSim, lidarAngles, maxRange);

    if size(lidarRangesRaw,2) > 1
        lidarRanges = min(lidarRangesRaw, [], 2, 'omitnan');
    else
        lidarRanges = lidarRangesRaw;
    end

    lidarRanges = double(lidarRanges(:));
    lidarAngles = double(lidarAngles(:));

    lidarRanges(isnan(lidarRanges)) = maxRange;
    lidarRanges(isinf(lidarRanges)) = maxRange;
    lidarRanges(lidarRanges < 0) = maxRange;
    lidarRanges(lidarRanges > maxRange) = maxRange;

    lidarRanges = lidarRanges(:);
    lidarAnglesUse = lidarAngles(:);

    frontSector = abs(lidarAnglesUse) <= deg2rad(45);
    minFrontRange = min(lidarRanges(frontSector));
    obstacleDetected = minFrontRange < obstacleThreshold;

    if minFrontRange > 2*obstacleThreshold
        routeReplanned = false;
    end

    if obstacleDetected && ~routeReplanned

        disp('Obstáculo detectado. Recalculando ruta...');
    
        currentGrid = world2grid(mapDynamic, robotPose(1:2));
    
        planner = plannerAStarGrid(mapDynamic);
    
        newPathGrid = plan(planner, currentGrid, goalGrid);
    
        if isempty(newPathGrid)
    
            disp('No existe ruta alternativa.');
    
            break;
    
        end
    
        pathWorld = grid2world(mapDynamic, newPathGrid);
    
        pp.Waypoints = pathWorld;
    
        routeReplanned = true;
    
        disp('Nueva ruta calculada.');
    
    end
    
    [v, omega] = pp(robotPose);

    robotPose(1) = robotPose(1) + v*cos(robotPose(3))*dt;
    robotPose(2) = robotPose(2) + v*sin(robotPose(3))*dt;
    robotPose(3) = wrapToPi(robotPose(3) + omega*dt);

    robotPath(end+1,:) = robotPose(1:2);

    set(hRobot, 'XData', robotPose(1), 'YData', robotPose(2));
    set(hPath, 'XData', robotPath(:,1), 'YData', robotPath(:,2));

    delete(findobj(gca, 'Tag', 'lidarDebug'));

    %plot(laserPoseSim(1), laserPoseSim(2), 'mo', 'MarkerSize', 6, 'LineWidth', 1.5, 'Tag', 'lidarDebug', 'HandleVisibility', 'off');

    % Dirección del robot
L = 1.0;
xRobotFront = robotPose(1) + L*cos(robotPose(3));
yRobotFront = robotPose(2) + L*sin(robotPose(3));

%plot([robotPose(1), xRobotFront], [robotPose(2), yRobotFront], 'g-', 'LineWidth', 2, 'Tag', 'lidarDebug', 'HandleVisibility', 'off');

% Dirección frontal del lidar
xLidarFront = laserPoseSim(1) + L*cos(laserPoseSim(3));
yLidarFront = laserPoseSim(2) + L*sin(laserPoseSim(3));
%{
plot([laserPoseSim(1), xLidarFront], ...
     [laserPoseSim(2), yLidarFront], ...
     'm-', 'LineWidth', 2, ...
     'Tag', 'lidarDebug', 'HandleVisibility', 'off');
%}
    L = 1.0;
    xFront = laserPoseSim(1) + L*cos(laserPoseSim(3));
    yFront = laserPoseSim(2) + L*sin(laserPoseSim(3));


    idxRays = 1:40:numel(lidarAnglesUse);
    for ii = idxRays
        r = lidarRanges(ii);
        xEnd = laserPoseSim(1) + r*cos(laserPoseSim(3) + lidarAnglesUse(ii));
        yEnd = laserPoseSim(2) + r*sin(laserPoseSim(3) + lidarAnglesUse(ii));
    end

    drawnow;

    if norm(robotPose(1:2) - goalWorld) < goalTol
        disp('Meta alcanzada.');
        break;
    end
end
