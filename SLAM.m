

function analyze_bag(bagFile)

%%%%%%%%%%% 
% ----------------------------------
% CARGA DE DATOS DEL ROSBAG | Robot MiR100 %
% ----------------------------------
%%%%%%%%%%%

    %----------------------------
    % - Cargar  el archivo rosbag 
    %----------------------------
    if nargin < 1
        [f, p] = uigetfile('*.bag', 'Selecciona un rosbag');
        if isequal(f,0)
            disp('No se ha seleccionado archivo.');
            return;
        end
        bagFile = fullfile(p, f);
    end

    fprintf('Cargando rosbag: %s\n', bagFile);
    bag = rosbag(bagFile);

    fprintf('\nTopics disponibles:\n');
    disp(bag.AvailableTopics.Properties.RowNames);

    % Helper para comprobar si existe un topic
    hasTopic = @(name) any(strcmp(name, bag.AvailableTopics.Properties.RowNames));

    % Helper para tiempo (segundos)
    timeFromHeader = @(h) double(h.Stamp.Sec) + double(h.Stamp.Nsec)*1e-9;

    % Helper para yaw desde cuaternión
    quatToYaw = @(q) atan2( ...
        2*(q.W*q.Z + q.X*q.Y), ...
        1 - 2*(q.Y^2 + q.Z^2));

    %----------------------------
    % 1) ODOMETRÍA ( /odom )
    %----------------------------
    odom = [];
    if hasTopic('/odom')
        bagOdom = select(bag, 'Topic', '/odom');
        odomMsgs = readMessages(bagOdom, 'DataFormat', 'struct');

        n = numel(odomMsgs);
        odom.t   = zeros(n,1);
        odom.x   = zeros(n,1);
        odom.y   = zeros(n,1);
        odom.yaw = zeros(n,1);

        for k = 1:n
            m = odomMsgs{k};
            odom.t(k) = timeFromHeader(m.Header);
            odom.x(k) = m.Pose.Pose.Position.X;
            odom.y(k) = m.Pose.Pose.Position.Y;

            q = m.Pose.Pose.Orientation;
            odom.yaw(k) = quatToYaw(q);
        end
        fprintf('Leídos %d mensajes de /odom\n', n);
    else
        warning('No se ha encontrado /odom en el bag.');
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
            scan.t(k) = timeFromHeader(m.Header);
            scan.ranges{k} = double(m.Ranges);
            scan.angles{k} = double(m.AngleMin) : ...
                             double(m.AngleIncrement) : ...
                             double(m.AngleMax);
        end
        fprintf('Leídos %d mensajes de /scan\n', n);
    else
        warning('No se ha encontrado /scan en el bag.');
    end
    
    
        %----------------------------
        % 3) IMU ( /imu_data )
        %----------------------------
        
     imu = [];
     if hasTopic('/imu_data')
        bagIMU = select(bag, 'Topic', '/imu_data');
        imuMsgs = readMessages(bagIMU, 'DataFormat', 'struct');
    
        n = numel(imuMsgs);
        imu.t   = zeros(n,1);
        imu.yaw = zeros(n,1);
    
        for k = 1:n
            m = imuMsgs{k};
            imu.t(k) = timeFromHeader(m.Header);
    
            q = m.Orientation;
            imu.yaw(k) = quatToYaw(q); % solo yaw
        end
        fprintf('Leídos %d mensajes de /imu_data\n', n);
     else
        warning('No se ha encontrado /imu_data en el bag.');
     end

 %% 

%%%%%%%%%%% 
% ----------------------------------------------
% SINCRONIZACIONES Y TRANSFORMACIONES DE DATOS 
% ----------------------------------------------
%%%%%%%%%%%
 
        %--------------------------------------------
        % Transformación LiDAR a frame de base_link 
        %--------------------------------------------
     
    % Dicha transformación se encuentra en /tf_static
    
    laser_tf = [];
    
    if hasTopic('/tf_static')
        bagTFs = select(bag, 'Topic', '/tf_static');
        tfMsgs = readMessages(bagTFs, 'DataFormat', 'struct');
    
        for k = 1:numel(tfMsgs)
            tfs = tfMsgs{k}.Transforms;
            for i = 1:numel(tfs)
                tf = tfs(i);
                if strcmp(tf.Header.FrameId,'base_link') && contains(tf.ChildFrameId,'laser')
    
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
      
    lx = laser_tf.Transform.Translation.X; % desplazamiento en eje x | frame base_link
    ly = laser_tf.Transform.Translation.Y; % desplazamiento en eje y | frame base_link
    
    q = laser_tf.Transform.Rotation; % rotación del láser | frame base_link
    
    laserYaw = atan2(2*(q.W*q.Z + q.X*q.Y), 1 - 2*(q.Y^2 + q.Z^2)); %conversión cuaternión a yaw 
    
    
        %-------------------------------
        % Sincronización de IMU con Odom
        %-------------------------------
    
    % Para la orientación del robot, vamos a utilizar la orientación del IMU
    % * Más precisa
    % * Menos error angular
    % * Menos deriva
    
    if ~isempty(imu)
        
        % Alineamos la IMU con odom, no tienen los mismos timestamps
    
        imu.yaw = double(imu.yaw); % 
        odom.yaw = interp1(imu.t, imu.yaw, odom.t, "nearest");
        fprintf("Sustituido yaw de odometría por IMU.\n");
    end
    
    % Con esto:  odometría = posición y la IMU = Orientación. 
    
    
        %----------------------------
        % Sincronización LiDAR /scan con odom /odom
        %----------------------------
    
    if ~isempty(scan) && ~isempty(odom)
    
        scan.odom_idx = zeros(length(scan.t),1);
    
        for k = 1:length(scan.t)
            [~, idx] = min(abs(odom.t - scan.t(k))); % obtenemos el valor mínimo encontrador
            % obviamos el valor, solo obtenemos la posición o índice del valor dentro del vector 
            scan.odom_idx(k) = idx;
        end
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

    margin = 2.0;  % metros de margen de seguridad
    
    % colocamos margenes de seguridad para evitar errores de hallar la pose
    % fuera del mapa 
    
    xmin = min(odom.x) - margin;
    xmax = max(odom.x) + margin;
    ymin = min(odom.y) - margin;
    ymax = max(odom.y) + margin;
    
    mapWidth  = xmax - xmin;
    mapHeight = ymax - ymin;
    
    fprintf('Límites del mapa:\n');
    fprintf('X: [%.2f, %.2f]\n', xmin, xmax);
    fprintf('Y: [%.2f, %.2f]\n', ymin, ymax);

%%%%%

   %%%%%%%%%%%
   % Creación del mapa de ocupación, definición de su resolución y
   % alineación del frame
   %%%%%%%%%%%


    mapResolution = 10;      % celdas por metro
    
    map = occupancyMap(mapWidth, mapHeight, mapResolution);
    
    % Colocar el origen del mapa según la odometría
    
    % Alineamos con esto el mapa con la odometría real. 
    
    map.GridLocationInWorld = [xmin, ymin];

   
    %%%%%%%%%%%
    % Mapeo | Insertar mediciones LIDAR en el mapa
    %%%%%%%%%%%

    for k = 1:length(scan.t)
    
        idx = scan.odom_idx(k);
    
        theta = odom.yaw(idx);
    
        % Posición real del láser en el mundo
        x_laser = odom.x(idx) + cos(theta)*lx - sin(theta)*ly;
        y_laser = odom.y(idx) + sin(theta)*lx + cos(theta)*ly;
    
        laserPose = [x_laser, y_laser, theta + laserYaw];
    
        ranges = scan.ranges{k};
        angles = scan.angles{k};
    
        % Limpiar rangos inválidos
        ranges = double(ranges);
        ranges(ranges > maxRange) = maxRange;
        ranges(ranges < rangeMin) = NaN;
    
        insertRay(map,laserPose, ranges, angles, maxRange);
    end

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

    m = mapMsgs{1};   % usar el primer mapa

    % Tamaño y resolución
    mapW = double(m.Info.Width);
    mapH = double(m.Info.Height);
    res  = double(m.Info.Resolution);   % [m/celda]

    % Convertir vector ROS a matriz 2D
    mapData = reshape(m.Data, mapW, mapH);
    mapData = flipud(mapData');  % ajustar orientación del mapa a MATLAB

    % Normalizar valores
    mapDataNorm = double(mapData);
    mapDataNorm(mapData == -1)  = NaN;   % desconocido
    mapDataNorm(mapData == 0)   = 0;     % libre
    mapDataNorm(mapData == 100) = 1;     % ocupado

    % Crear mapa MATLAB
    map_real = occupancyMap(mapDataNorm, 1/res);

end 

    figure('Name','Mapa del rosbag (/map)','NumberTitle','off');
    show(map_real);
    axis equal;
    title('Mapa real obtenido desde el rosbag');
    xlabel('x [m]');
    ylabel('y [m]');

    %%%%%%%%%%%
    % Gráfica de la trayectoria del robot
    %%%%%%%%%%%

    % Trayectoria XY
    figure('Name','Trayectoria','NumberTitle','off');
    hold on; grid on; axis equal;
    if ~isempty(odom)
        plot(odom.x, odom.y, 'b--', 'DisplayName', 'Odom');
    end
    xlabel('x [m]');
    ylabel('y [m]');
    title('Trayectorias 2D');
    legend('Location','best');
    hold off;

    tf_map_odom = [];

    %%%%%%%%%%%
    % Gráfica de la trayectoria en el mapa real
    %%%%%%%%%%%

    if hasTopic('/tf')
        bagTF = select(bag,'Topic','/tf');
        tfMsgs = readMessages(bagTF,'DataFormat','struct');
    
        for k = 1:numel(tfMsgs)
            tfs = tfMsgs{k}.Transforms;
            for i = 1:numel(tfs)
                tf = tfs(i);
                if strcmp(tf.Header.FrameId,'map') && strcmp(tf.ChildFrameId,'odom')
    
                    tf_map_odom = tf;
                    break;
                end
            end
            if ~isempty(tf_map_odom)
                break;
            end
        end
    end
        
        % Traslación
        tx = tf_map_odom.Transform.Translation.X;
        ty = tf_map_odom.Transform.Translation.Y;
        
        % Rotación (yaw)
        q = tf_map_odom.Transform.Rotation;

        % conversión de cuaternión a yaw 
        yaw = atan2( 2*(q.W*q.Z + q.X*q.Y),1 - 2*(q.Y^2 + q.Z^2));
        
        % Matriz de rotación 
        R = [cos(yaw) -sin(yaw);
             sin(yaw)  cos(yaw)];

        % Se aplica la transformación de la trayectoria 
        traj_map = R * [odom.x'; odom.y'];

        %Toma todas la filas de x y suma la traslación en x del frame /map
        odom_x_map = traj_map(1,:) + tx;    

        %Toma todas la filas de y y suma la traslación en y del frame /map
        odom_y_map = traj_map(2,:) + ty;

        
        figure('Name','Trayectoria sobre el mapa real','NumberTitle','off');
        show(map_real);
        hold on;
        plot(odom_x_map, odom_y_map, 'r', 'LineWidth', 1.5);
        axis equal;
        title('Odometría transformada al frame /map');
        xlabel('x [m]');
        ylabel('y [m]');
        hold off;


    %%%%%%%%%%%
    % Gráfica de los obstáculos detectado en el mapeo
    %%%%%%%%%%%a 
        
        occ = occupancyMatrix(map);
        obstacles = occ > 0.65;
        
        figure('Name','Obstáculos detectados','NumberTitle','off');
        imshow(obstacles);
        title('Celdas ocupadas (obstáculos)');

    %%%%%%%%%%%
    % Gráfica del mapa construido
    %%%%%%%%%%%a 

        figure('Name','Mapa generado por scan','NumberTitle','off');
        show(map);
        axis equal;
        title('Mapa generado a partir de odometría + láser');
        xlabel('x [m]');
        ylabel('y [m]');

    %%%%%%%%%%%
    % Gráfica de la trayectoria en el mapa construido con SLAM 
    %%%%%%%%%%%a 

    
    figure('Name','Mapa SLAM con trayectoria','NumberTitle','off');
    show(map);
    hold on;
    plot(odom.x, odom.y, 'r', 'LineWidth', 1.2);
    axis equal;
    title('Mapa generado por scan con trayectoria del robot');
    xlabel('x [m]');
    ylabel('y [m]');
    hold off;

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%% 
% ----------------------------------------------
% MÉTRICAS MAPA CONSTRUIDO  | Métricas de evaluación de calidad 
% ----------------------------------------------
%%%%%%%%%%%


    %%%%%%%%%%%
    %  ERROR | MAPA GENERADO VS  MAPA REAL 
    %%%%%%%%%%%

    % Convertir los mapas a matriz numérica 
    occ_gen = occupancyMatrix(map);
    occ_real = occupancyMatrix(map_real);

    % Redimensionamos mapas para que tengan mismo tamaño 
    occ_real_resized = imresize(occ_real, size(occ_gen), 'nearest');

    % Filtran celdas desconocidas 
    mask = isnan(occ_gen) | isnan(occ_real_resized); 
    
    % Eliminamos celdas inválidas 
    A = occ_gen;  A(mask)= [];
    B = occ_real_resized; B(mask)= [];
    
    % determinamos el error rmse 
   rmse_mapa = sqrt(mean((A(:) - B(:)).^2)); 
   disp(['Error entre mapas : ', num2str(rmse_mapa)]);

    %%%%%%%%%%%
    %  PORCENTAJE | COINCIDENCIA DE OCUPACIÓN 
    %%%%%%%%%%%

    Coincidencia_celdas = sum(A(:) == B(:)) / numel(A);
    disp(['Coincidencia de ocupación : ', num2str(Coincidencia_celdas)]);

    %%%%%%%%%%%
    %  PORCENTAJE | CELDAS DESCONOCIDAS
    %%%%%%%%%%%

    Celdas_desconocidas = sum(isnan(occ_gen), 'all') / numel(occ_gen);
    disp(['Celdas desconocidas: ', num2str(Celdas_desconocidas)]);

end

