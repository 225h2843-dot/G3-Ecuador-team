%%% CÓDIGO PROYECTO INTEGRADOR 1 

%%% ASIGNATURA: NAVEGACIÓN Y VEHÍCULOS AUTÓNOMOS 
%%% INTEGRANTES: GRUPO 3 ECUADOR TEAM



function analyze_bag(bagFile)

% Script básico para analizar un rosbag de MIR
%
% Uso:
%   analyze_mir_bag('ruta/al/rosbag.bag')

    %----------------------------
    % 0) Cargar bag
    %----------------------------
    if nargin < 1
        [f, p] = uigetfile('*.bag', 'Selecciona un rosbag'); % Seleccionamos el archivo con el rosbag 
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
    % 1) ODOMETRÍA (/odom)
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
    % 2) GROUND TRUTH (/base_pose_ground_truth)
    %----------------------------
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
    
            % OJO: aquí es Pose, no pose
            gt.x(k) = m.Pose.Pose.Position.X;
            gt.y(k) = m.Pose.Pose.Position.Y;
    
            q = m.Pose.Pose.Orientation;
            gt.yaw(k) = quatToYaw(q);
        end
        fprintf('Leídos %d mensajes de /base_pose_ground_truth\n', n);
    else
        fprintf('No hay /base_pose_ground_truth (no se pintará GT).\n');
    end

%% ANÁLISIS 1) DE GROUND TRUTH VS ODOM 

    %----------------------------
    % PLOTS BÁSICOS
    %----------------------------

    % Trayectoria XY
    figure('Name','Trayectorias','NumberTitle','off');
    hold on; grid on; axis equal;
    if ~isempty(gt)
        plot(gt.x, gt.y, 'k-', 'DisplayName', 'Ground truth');
    end
    if ~isempty(odom)
        plot(odom.x, odom.y, 'b--', 'DisplayName', 'Odom');
    end
    xlabel('x [m]');
    ylabel('y [m]');
    title('Trayectorias 2D');
    legend('Location','best');
    hold off;

    % Yaw vs tiempo (si hay odom)

    if ~isempty(odom) || ~isempty(gt)
        figure('Name','Yaw vs tiempo','NumberTitle','off');
        hold on; grid on;
        if ~isempty(odom)
            plot(odom.t - odom.t(1), odom.yaw, 'b--', 'DisplayName', 'Odom');
        end

        if ~isempty(gt)
            plot(gt.t - gt.t(1), gt.yaw, 'k-', 'DisplayName', 'GT');
        end
        xlabel('t [s]');
        ylabel('yaw [rad]');
        title('Orientación (yaw) vs tiempo');
        legend('Location','best');
        hold off;
    end

    fprintf('\nListo. Revisa las figuras generadas.\n');

%% ANÁLISIS 2) DE GROUND TRUTH VS AMCL

    %---------------------------
    % 3) AMCL (/amcl_pose)
    %----------------------------
    amcl = [];
    if hasTopic('/amcl_pose')
        bagAmcl = select(bag, 'Topic', '/amcl_pose');
        amclMsgs = readMessages(bagAmcl, 'DataFormat', 'struct');

        n = numel(amclMsgs);
        amcl.t   = zeros(n,1);
        amcl.x   = zeros(n,1);
        amcl.y   = zeros(n,1);
        amcl.yaw = zeros(n,1);

        for k = 1:n
            m = amclMsgs{k};
            amcl.t(k) = timeFromHeader(m.Header);
            amcl.x(k) = m.Pose.Pose.Position.X;
            amcl.y(k) = m.Pose.Pose.Position.Y;

            q = m.Pose.Pose.Orientation;
            amcl.yaw(k) = quatToYaw(q);
        end
        fprintf('Leídos %d mensajes de /amcl_pose\n', n);
    else
        fprintf('No hay /amcl_pose (no se pintará AMCL).\n');
    end


%% ANÁLISIS 2) DE GROUND TRUTH VS AMCL

    %----------------------------
    % PLOTS BÁSICOS PARA AMCL
    %----------------------------

    % 4.1 Trayectoria XY
    figure('Name','Trayectorias','NumberTitle','off');
    hold on; grid on; axis equal;
    if ~isempty(gt)
        plot(gt.x, gt.y, 'k-', 'DisplayName', 'Ground truth');
    end
    if ~isempty(amcl)
        plot(amcl.x, amcl.y, 'r-', 'DisplayName', 'AMCL');
    end
    xlabel('x [m]');
    ylabel('y [m]');
    title('Trayectorias 2D');
    legend('Location','best');
    hold off;

  % 4.2 Yaw vs tiempo (si hay odom y/o amcl)
    if ~isempty(amcl) || ~isempty(gt)
        figure('Name','Yaw vs tiempo','NumberTitle','off');
        hold on; grid on;
        if ~isempty(amcl)
            plot(amcl.t - amcl.t(1), amcl.yaw, 'r-', 'DisplayName', 'AMCL');
        end
        if ~isempty(gt)
            plot(gt.t - gt.t(1), gt.yaw, 'k-', 'DisplayName', 'GT');
        end
        xlabel('t [s]');
        ylabel('yaw [rad]');
        title('Orientación (yaw) vs tiempo');
        legend('Location','best');
        hold off;
    end


    fprintf('\nListo. Revisa las figuras generadas.\n');

%% ALINEACION TRAYECTORIA Y ORIENTACION AMCL CON GROUND TRUTH  

%%% Para trayectoria 

% Cálculo desplazamiento
dx = gt.x(1) - amcl.x(1);  
dy = gt.y(1) - amcl.y(1);

% Traslación a AMCL 

amcl.x_aligned = amcl.x + dx;
amcl.y_aligned = amcl.y + dy;

%%% Para Orientacion 

% Tiempo relativo
t_amcl = amcl.t;
t_gt   = gt.t;

% Yaw 
gt_yaw_u   = unwrap(gt.yaw);
amcl_yaw_u = unwrap(amcl.yaw);

% Interpolar GT al tiempo de AMCL
gt_yaw_interp_odom = interp1(t_gt, gt_yaw_u, t_amcl, 'linear', 'extrap');

d_yaw = gt_yaw_interp_odom(1) - amcl_yaw_u(1);
amcl_yaw_aligned = amcl_yaw_u + d_yaw;

%% GRÁFICA DE LAS TRAYECTORIAS ALINEADAS AMCL Y GT

%  Gráfica trayectoria 

figure;
hold on; grid on; axis equal;
plot(gt.x, gt.y, 'k', 'DisplayName','Ground Truth');
plot(amcl.x_aligned, amcl.y_aligned, 'r', 'DisplayName','AMCL alineado');
xlabel('x [m]');
ylabel('y [m]');
title('Trayectorias 2D: GT vs AMCL alineado');
legend;
hold off;

%  Gráfica yaw vs tiempo 
figure;
hold on; grid on;
plot(t_amcl - t_amcl(1), wrapToPi(gt_yaw_interp_odom), 'k', 'DisplayName','GT');
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
gt_yaw_interp_odom = interp1(gt.t, unwrap(gt.yaw), odom.t);
     
%%%%%% PARA ODOM %%%%%%

% distancia euclidiana
epos_odom = sqrt((odom.x - gt_x_interp_odom).^2 + (odom.y - gt_y_interp_odom).^2) ; 

    %----------------------------
    % ERROR DE ORIENTACIÓN
    %----------------------------

yaw_odom = unwrap(odom.yaw);

% Error angular 

%%%%%% PARA ODOM %%%%%%
yaw_error_odom = yaw_odom - gt_yaw_interp_odom ; 

    %----------------------------
    %           RMSE 
    %----------------------------

% RMSE en la posición

% Eliminación de los NaN en posición y orientación 
% generados por la interpolación de Odom con GT 


valid_odom = ~isnan(gt_x_interp_odom) & ~isnan(gt_y_interp_odom); %ignorar valores fuera de rango temporal 

valid_yaw_odom = ~isnan(gt_yaw_interp_odom);

% Cálculo del error posición y orunetaciób sin NaN
epos_odom2 = sqrt((odom.x(valid_odom) - gt_x_interp_odom(valid_odom)).^2 +(odom.y(valid_odom) - gt_y_interp_odom(valid_odom)).^2 );

yaw_error_odom2 = yaw_odom(valid_yaw_odom) - gt_yaw_interp_odom(valid_yaw_odom);

%%%%%% PARA ODOM %%%%%%
rmse_pos_odom = sqrt(mean(epos_odom2.^2)); 

% RMSE en la orientación 

%%%%%% PARA ODOM %%%%%%
rmse_yaw_odom = sqrt(mean(yaw_error_odom2.^2));


%% GRÁFICA DE LOS ERRORES DE ODOM Y AMCL 

    %----------------------------
    % ERROR DE POSICIÓN
    %----------------------------

    figure;
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

    figure;
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


