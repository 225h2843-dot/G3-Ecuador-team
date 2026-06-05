# PROYECTO INTEGRADOR

##  Pipeline | Localización y Análisis de datos del Robot MiR100 

* Este archivo contiene la información del flujo de trabajo del programa de localización de un robot MiR100 para analizar la trayectoria estimada y su consistencia. Se utiliza datos de posición y orientación por odometría
* '/odom', posición real del robot '/base_pose_ground_truth' y estimación a partir del algoritmo AMCL '/amcl_pose'. 

* Dichos 'Topics' se encuentra dentro del archivo de rosbag 'mir_basics_20251210_114529.bag' para el robot MiR100

## REQUISITOS E INSTRUCCIONES DE USO !!!

**Prerrequisitos !!**
* MATLAB instalado (Versión R2021a o superior recomendada).
* **ROS Toolbox** instalado en MATLAB.
* El archivo de datos 'mir_basics_20251210_114529.bag' del robot MiR100 ubicado en una ruta accesible junto con el script 'Proyecto_Integrador_1.m'

**Ejecución del Script** 
1. Abre MATLAB y sitúa el directorio de trabajo en la carpeta que contiene el archivo `Proyecto_Integrador_1.m`.
2. Ejecuta la función desde la ventana de comandos llamándola por su nombre: 'Proyecto_Integrador_1'

## FLUJO DE TRABJO  
El script 'Proyecto_Integrador_1' se encuentra estructurado de manera secuencial y modular dentro de 'function analyze_bag(bagFile)' en donde se contiene y corre el programa. 

## * PASO 1: Inicialización y Carga de rosbag 
El programa prepara el entorno de Matlab para interpretar datos provinientes de un rosbag para el robot MiR100: 
  **Carga interactiva:** Si no se introduce una ruta por defecto, el código abre una ventana emergente (`uigetfile`) para seleccionar visualmente el archivo `.bag`.
  **Adquisición datos:** La función 'rosbag(bagFile)' se abre el archivo y se adquieren/ listan los topics de sensores disponibles 
  **Herramientas matemáticas:** Herramientas como 'timeFromHeader' convierte el tiempo fraccionado (segundos y nanosegundos) a un formato continuo en segundos ('double') de Matlab. 
                                 'quatToYaw' convierte orientación 3D X,Y,Z (cuaternión) a un ángulo Yaw en 2D/ 

## * PASO 2: Extracción y Preprocesamiento de Datos
Primero el programa crea vectores de cerros ('zeros') para optimización de rendimiento. Posterior, mediante bucles 'for' desempaqueta las estructuras y aisla la infromación en tres ariables: 
1. **Odometría ('odom'):** La posición estimada acumulada por las ruedas del MiR100 (`/odom`).
2. **Ground Truth ('gt'):** La posición real del robot MiR100, como fuente de referencia (`/base_pose_ground_truth`).
3. **AMCL ('amcl'):** La posición estimada por el algoritmo probabilístico AMCL (`/amcl_pose`).

## * PASO 3: Alineación Espacial y Temporal (Sincronización)
Dado que los datos provienen de diferentes fuentes, no se pueden comparar directamente debido a desfases geométricos y de frecuencia:
1. **Alineación espacial (X, Y, Yaw):** AMCL empieza en un marco global de coordenadas y el Ground Truth arranca en otro. El código calcula la distancia inicial (`dx`, `dy`, `d_yaw`) y traslada la trayectoria de AMCL para que ambas inicien exactamente en el mismo origen.
2. **Suavizado angular ('unwrap'):** Evita que las gráficas se rompan con saltos bruscos cuando el robot gira y pasa del límite de 'pi' a '-pi' (180 grados a -180 grados). 
3. **Interpolación temporal ('interp1'):** Como el Ground Truth y la Odometría registran datos a velocidades (frecuencias) distintas, se utiliza la interpolación lineal para calcular matemáticamente la posición real exacta en los mismos instantes de tiempo en los que midieron los sensores.

## * PASO 4: Análisis Cuantitativo de Errores
Con todas las trayectorias en el mismo instante de tiempo y espacio, el código calcula las desviaciones en cada instante:
1. **Error de posición ('epos_odom'):** Se calcula aplicando la distancia euclidiana entre la posición de la odometría y el Ground Truth interpolado: epos_odom = ' sqrt((odom.x - gt_x_interp_odom).^2 + (odom.y - gt_y_interp_odom).^2) '
2. **Error de orientación ('yaw_error_odom'):** Resta directa entre el ángulo calculado por las ruedas y el ángulo del algoritmo AMCL.
3. **Filtrado de Nulos (`~isnan`):** Se remueven los valores indeterminados (`NaN`) producidos en los extremos por la interpolación para evitar errores en los promedios globales

### * PASO 5: Cálculo de RMSE y Gráfica de resultados
Finalmente, se presentan los resultados y se grafican
1 **Métricas (RMSE):** Calcula e imprime el **Error Cuadrático Medio (RMSE)** de la posición y la orientación de la odometría para medir numéricamente la gravedad de la deriva (*drift*).
2 **Gráficas de Trayectorias:** Genera mapas en 2D utilizando 'figure' comparando rutas y su presición correspondeinte con la real.
* **Gráficas Temporales:** Muestra la evolución del error y del ángulo Yaw a lo largo del tiempo, normalizando el inicio para t = 0 segundos. 
