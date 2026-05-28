%%% CÓDIGO PROYECTO INTEGRADOR 1 

%%% ASIGNATURA: NAVEGACIÓN Y VEHÍCULOS AUTÓNOMOS 
%%% INTEGRANTES: GRUPO 3 ECUADOR TEAM



function analyze_bag(bagFile)

    %----------------------------
    % CÓDIGO PARA CARGAR EL ROSBAG DEL ROBOT MiR100
    %----------------------------

    % Verifica si se ha cargado el MiR100 o se abre una ventana para
    % seleccionar 
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

    fprintf('\nTopics disponibles:\n');   % Imprime los topics del Rosbag
    disp(bag.AvailableTopics.Properties.RowNames);

    hasTopic = @(name) any(strcmp(name, bag.AvailableTopics.Properties.RowNames));

    timeFromHeader = @(h) double(h.Stamp.Sec) + double(h.Stamp.Nsec)*1e-9;  % Conversión del tiempo a sec.

   
    quatToYaw = @(q) atan2(2*(q.W*q.Z + q.X*q.Y),1-2*(q.Y^2 + q.Z^2));  % Conversión de cuaternión a Yaw. 

    %----------------------------
    % 1) ODOMETRÍA (/odom)
    %----------------------------

    % Comprueba la existencia del Topic /odom. Si existe entonces filtra el
    % rosbag para quedarse con los mensajes de odometria, y los convierte
    % en una lista (cell array). 

    odom = [];
    if hasTopic('/odom')
        bagOdom = select(bag, 'Topic', '/odom'); % Filtra mensajes /odom
        odomMsgs = readMessages(bagOdom, 'DataFormat', 'struct'); % convierte mensaje a arrays

        % Crea vectores 0 de: tiempo, x, y , yaw, del tamaño del número de mensajes. 
        n = numel(odomMsgs);
        odom.t   = zeros(n,1);
        odom.x   = zeros(n,1);
        odom.y   = zeros(n,1);
        odom.yaw = zeros(n,1);

        % Crea bucle for para extraer información de odometría de
        % x,y,tiempo y yaw

        for k = 1:n
            m = odomMsgs{k};
            odom.t(k) = timeFromHeader(m.Header); % Extrae tiempo sec
            odom.x(k) = m.Pose.Pose.Position.X;   % Extraer valores x 
            odom.y(k) = m.Pose.Pose.Position.Y;   % Extraer valores y

            q = m.Pose.Pose.Orientation;          % Extraer valores orientación (cuaternion)
            odom.yaw(k) = quatToYaw(q);           % Conversión de cuaternión a Yaw. (función línea 36)
        end
        fprintf('Leídos %d mensajes de /odom\n', n);
    else
        warning('No se ha encontrado /odom en el bag.');
    end

    %----------------------------
    % 2) GROUND TRUTH (/base_pose_ground_truth)
    %----------------------------

    % El mismo proceso para 1) ODOMETRÍA (/odom) se realiza para 2) GROUND TRUTH
    % Sigue el mismo flujo y proceso. 

    gt = [];
    if hasTopic('/base_pose_ground_truth')
        bagGT = select(bag, 'Topic', '/base_pose_ground_truth');
        gtMsgs = readMessages(bagGT, 'DataFormat', 'struct');
    
        n = numel(gtMsgs);
        gt.t   = zeros(n,1);
        gt.x   = zeros(n,1);
        gt.y   = zeros(n,1);
        gt.yaw = zeros(n,1);
    
        for k = 1:n
            m = gtMsgs{k};
            gt.t(k) = timeFromHeader(m.Header);
            gt.x(k) = m.Pose.Pose.Position.X;
            gt.y(k) = m.Pose.Pose.Position.Y;
    
            q = m.Pose.Pose.Orientation;
            gt.yaw(k) = quatToYaw(q);
        end
        fprintf('Leídos %d mensajes de /base_pose_ground_truth\n', n);
    else
        fprintf('No hay /base_pose_ground_truth (no se pintará GT).\n');
    end

%% ANALISIS %%

    %% ANÁLISIS 1) DE GROUND TRUTH VS ODOM 

    %----------------------------
    % GRÁFICAS TRAYECTORIAS 
    %----------------------------

    % Trayectoria XY
    figure('Name','Trayectorias Robot MiR100 de GT Y ODOMETRÍA','NumberTitle','off');
    hold on; grid on; axis equal;
    
    % ||||| TRAYECTORIA DEL DESPLAZAMIENTO/ MOVIMIENTO EN EL MAP ||

    % Se condicionan las gráficas para verificar que las vaiables no estén vacías. 

    if ~isempty(gt)
        plot(gt.x, gt.y, 'k-', 'DisplayName', 'Ground truth'); % Gráfica GT (color negro)
    end

    if ~isempty(odom)
        plot(odom.x, odom.y, 'b--', 'DisplayName', 'Odom'); % Gráfica Odom (color azúl)
    end
    
    xlabel('x [m]');
    ylabel('y [m]');
    title('Trayectorias GROUND TRUTH (NEGRO) VS ODOMETRÍA (AZÚL)');
    legend('Location','best');
    hold off;

    % ||||| ORIENTACIÓN DEL ROBOT CON RESPECTO AL TIEMPO | VARIACIÓN TEMPORAL ||

    % Yaw vs tiempo 

    if ~isempty(odom) || ~isempty(gt)
        figure('Name','Yaw vs tiempo','NumberTitle','off');
        hold on; grid on;
        if ~isempty(odom)
            plot(odom.t - odom.t(1), odom.yaw, 'b--', 'DisplayName', 'Odom');  % Normalización del tiempo
        end

        if ~isempty(gt)
            plot(gt.t - gt.t(1), gt.yaw, 'k-', 'DisplayName', 'GT');   % Normalización del tiempo
        end
        xlabel('t [s]');
        ylabel('yaw [rad]');
        title('Orientación (yaw) vs tiempo');
        legend('Location','best');
        hold off;
    end

%% ANÁLISIS 2) DE GROUND TRUTH VS AMCL

    % ANÁLISIS COMPARATIVO DE LA POSICIÓN REAL DEL ROBOT CON EL ALGORITMO
    % AMCL UTILIZADO PARA DETERMINAR LA LOCALIZACIÓN DEL ROBOT. 


    % Comprueba la existencia del Topic /odom. Si existe entonces filtra el
    % rosbag para quedarse con los mensajes de odometria, y los convierte
    % en una lista (cell array). 

    %---------------------------
    % 3) AMCL (/amcl_pose)
    %----------------------------
    amcl = [];
    if hasTopic('/amcl_pose')
        bagAmcl = select(bag, 'Topic', '/amcl_pose');     % Filtra mensajes /odom
        amclMsgs = readMessages(bagAmcl, 'DataFormat', 'struct'); % Convierte mensajes en arrays

        % Creas vector zeros del tamaño de la cantidad de mensajes para
        % tiempo, x,y, yaw

        n = numel(amclMsgs);
        amcl.t   = zeros(n,1); 
        amcl.x   = zeros(n,1);
        amcl.y   = zeros(n,1);
        amcl.yaw = zeros(n,1);

         % Crea bucle for para extraer información del amcl de
        % x,y,tiempo y yaw

        for k = 1:n
            m = amclMsgs{k};
            amcl.t(k) = timeFromHeader(m.Header); % Extrae información de tiempo sec
            amcl.x(k) = m.Pose.Pose.Position.X;   % Extrae información de Posiición en x
            amcl.y(k) = m.Pose.Pose.Position.Y;   % Extrae infromación de Posición en y 

            q = m.Pose.Pose.Orientation; % Extrae información de orientación del robot en cuaternión
            amcl.yaw(k) = quatToYaw(q);   % Convierte cuaternion a yaw      
        end
        fprintf('Leídos %d mensajes de /amcl_pose\n', n);
    else
        fprintf('No hay /amcl_pose.\n');
    end

  
% ANÁLISIS 2)  GRÁFICA |  DE GROUND TRUTH VS AMCL

    %----------------------------
    % PLOTS BÁSICOS PARA AMCL
    %----------------------------

    % ||||| TRAYECTORIA DEL DESPLAZAMIENTO/ MOVIMIENTO EN EL MAP ||

    % Trayectoria XY
    figure('Name','Trayectorias del Robot MiR100 GT Y AMCL','NumberTitle','off');
    hold on; grid on; axis equal;

    % Se condicionan las gráficas para verificar que las vaiables no estén vacías. 
    
    if ~isempty(gt)
        plot(gt.x, gt.y, 'k-', 'DisplayName', 'Ground truth');  % Gráfica GT (color NEGRO)
    end 

    if ~isempty(amcl)
        plot(amcl.x, amcl.y, 'r-', 'DisplayName', 'AMCL');   % Gráfica AMCL (color ROJO)
    end 

    xlabel('x [m]');
    ylabel('y [m]');
    title('Trayectorias 2D  GT Y AMCL');
    legend('Location','best');
    hold off;

    % ||||| ORIENTACIÓN DEL ROBOT CON RESPECTO AL TIEMPO | VARIACIÓN TEMPORAL ||

     % Yaw vs tiempo 

    if ~isempty(amcl) || ~isempty(gt)

        figure('Name','Gráfcia Temporal Yaw vs tiempo','NumberTitle','off');
        hold on; grid on;

        if ~isempty(amcl)
            plot(amcl.t - amcl.t(1), amcl.yaw, 'r-', 'DisplayName', 'AMCL');    % Normalización del tiempo
        end

        if ~isempty(gt)
            plot(gt.t - gt.t(1), gt.yaw, 'k-', 'DisplayName', 'GT');   % Normalización del tiempo
        end
        xlabel('t [s]');
        ylabel('yaw [rad]');
        title('Orientación (yaw) vs tiempo');
        legend('Location','best');
        hold off;
    end

%% ALINEACION TRAYECTORIA Y ORIENTACION AMCL CON GROUND TRUTH  

% LAS GRÁFICAS INICIALMENTE SE ENCUENTRAN CON UN DESAFASE EN CUANTO AL
% ORIGEN DE LA TRAYECTORIA. 

% Ground Truth empieza en coordenadas X,Y (0,0) mientras que el AMCL
% empieza en marcos de referencia globales diferentes. 

%%% Para trayectoria 

% Cálculo desplazamiento
dx = gt.x(1) - amcl.x(1);    % desfase en x 
dy = gt.y(1) - amcl.y(1);    % desfase en y 

% Traslación a AMCL 

amcl.x_aligned = amcl.x + dx;   % Alinear AMCL al GT en x 
amcl.y_aligned = amcl.y + dy;   % Alinaer AMCL al GT en y

%%% Para Orientación 

% De igual manera, existe un desfase temporal por parte del amcl con
% respceto al GT.

% Tiempo relativo
t_amcl = amcl.t;
t_gt   = gt.t;

% Yaw 

% unwrap es clave ya que permite que el robot gire los 360 grados elimina
% el límite de de rotación de [-pi, pi] que es 180 grados a -180 grados.
% Permite llegar a valores mayores a -pi o pi. 

% Convierte curva orientación en una cuerva suave y continua. 

gt_yaw_u   = unwrap(gt.yaw);  % Elimina los límites de rotación -pi a pi 
amcl_yaw_u = unwrap(amcl.yaw); % Elimina los límites de rotación -pi a pi 

% Interpolar GT al tiempo de AMCL
gt_yaw_interp_amcl = interp1(t_gt, gt_yaw_u, t_amcl, 'linear', 'extrap');

d_yaw = gt_yaw_interp_amcl(1) - amcl_yaw_u(1);
amcl_yaw_aligned = amcl_yaw_u + d_yaw;

%% GRÁFICA DE LAS TRAYECTORIAS ALINEADAS AMCL Y GT

%  Gráfica trayectoria 

figure('Name','Trayectorias del Robot MiR100 GT Y AMCL Alineado','NumberTitle','off');

hold on; grid on; axis equal;
plot(gt.x, gt.y, 'k', 'DisplayName','Ground Truth');
plot(amcl.x_aligned, amcl.y_aligned, 'r', 'DisplayName','AMCL alineado');
xlabel('x [m]');
ylabel('y [m]');
title('Trayectorias 2D Robot MiR100: GT vs AMCL alineado');
legend;
hold off;

%  Gráfica yaw vs tiempo 
figure('Name','Gráfica Temporal GT vs AMCL Alineado','NumberTitle','off');
hold on; grid on;
plot(t_amcl - t_amcl(1), wrapToPi(gt_yaw_interp_amcl), 'k', 'DisplayName','GT');
plot(t_amcl - t_amcl(1), wrapToPi(amcl_yaw_aligned), 'r', 'DisplayName','AMCL');
xlabel('Tiempo [s]');
ylabel('Yaw [rad]');
title('Orientación: GT vs AMCL sincronizado');
legend;
hold off;

%% ANÁLISIS ERRORES CUANTITATIVOS 

     %----------------------------
     % ERROR DE POSICIÓN
     %----------------------------

odom.x %posicion en x de la odom
odom.y %posición en y de la odom

gt.x % posición en x real del robot 
gt.y %posición en y real del robot    

% Interpolación de GT a los tiempos de odom
gt_x_interp_odom = interp1(gt.t, gt.x, odom.t);
gt_y_interp_odom= interp1(gt.t, gt.y, odom.t);
gt_yaw_interp_amcl = interp1(gt.t, unwrap(gt.yaw), odom.t);
     
%%%%%% PARA ODOM %%%%%%

% distancia euclidiana
epos_odom = sqrt((odom.x - gt_x_interp_odom).^2 + (odom.y - gt_y_interp_odom).^2) ; 

    %----------------------------
    % ERROR DE ORIENTACIÓN
    %----------------------------

yaw_odom = unwrap(odom.yaw);

% Error angular 

%%%%%% PARA ODOM %%%%%%
yaw_error_odom = yaw_odom - gt_yaw_interp_amcl ; 

    %----------------------------
    %           RMSE 
    %----------------------------

% RMSE en la posición

% Eliminación de los NaN en posición y orientación 
% generados por la interpolación de Odom con GT 


valid_odom = ~isnan(gt_x_interp_odom) & ~isnan(gt_y_interp_odom); %ignorar valores fuera de rango temporal 

valid_yaw_odom = ~isnan(gt_yaw_interp_amcl);

% Cálculo del error posición y orientación sin NaN
epos_odom2 = sqrt((odom.x(valid_odom) - gt_x_interp_odom(valid_odom)).^2 +(odom.y(valid_odom) - gt_y_interp_odom(valid_odom)).^2 );

yaw_error_odom2 = yaw_odom(valid_yaw_odom) - gt_yaw_interp_amcl(valid_yaw_odom);

%%%%%% PARA ODOM %%%%%%
rmse_pos_odom = sqrt(mean(epos_odom2.^2)); 

% RMSE en la orientación 

%%%%%% PARA ODOM %%%%%%
rmse_yaw_odom = sqrt(mean(yaw_error_odom2.^2));


%% GRÁFICA DE LOS ERRORES DE ODOM Y AMCL 

    %----------------------------
    % ERROR DE POSICIÓN
    %----------------------------

    figure('Name','Gráfica de Error de Posición de Odom ','NumberTitle','off');
    hold on;
    grid on;
    
    plot(odom.t - odom.t(1), epos_odom, 'b', 'DisplayName','Odom');

    xlabel('Tiempo [s]');
    ylabel('Error posición [m]');
    title('Error de posición Odmo vs Ground Truth');
    legend;
    hold off;


    %----------------------------
    % ERROR DE ORIENTACIÓN
    %----------------------------

    figure('Name','Gráfica de Error de Orientación de Odom','NumberTitle','off');
    hold on; 
    grid on; 

    plot(odom.t - odom.t(1), yaw_error_odom, 'b', 'DisplayName','Odom');
    
    xlabel('Tiempo [s]');
    ylabel('Error de orientación [rad]');
    title('Error de orientación: ODOM vs Ground Truth');


    %----------------------------
    %           RMSE
    %----------------------------

    % posicion
    fprintf('RMSE Posición ODOM: %.3f m\n', rmse_pos_odom);

    % orinetacion
    fprintf('RMSE Orientación ODOM: %.3f rad\n', rmse_yaw_odom);

end


