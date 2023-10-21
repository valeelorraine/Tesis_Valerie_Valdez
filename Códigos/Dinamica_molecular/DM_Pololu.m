%%                          COLICIONES EN 2D

%% Limpiar todo
clear all;
clc;

%% POLOLU Conexión con el servidor del Robotat y Pololu
PololuNumb = 2;                           % Pololu a utilizar
ObjNumb = 2;                              % Número de Marker 
Opti = robotat_connect(); % Conexión con el robotat
PI3 = robotat_3pi_connect(PololuNumb);    % Conectarse al pololu

%% Desconexion del agente
robotat_3pi_disconnect(PI3)

%% Probar si el Pololu está conectado
robotat_3pi_set_wheel_velocities(PI3,-50,50)

%% Stop de emergencia
robotat_3pi_force_stop(PI3)

%% Variables del Pololu
r = 32/(2*1000); % Radio de las ruedas en metros
l = 96/(2*1000); % Distancia entre ruedas en metros

%% Obtén la posición inicial del robot en el sistema OptiTrack
pose = robotat_get_pose(Opti, PololuNumb, 'eulzyx');
x_initial = pose(1);
y_initial = pose(2);

% Realiza la traslación para ajustar al centro del espacio
x_adjusted_initial = x_initial - 3 + 0.5; % Resta 3 para alinear el centro y agrega 0.5 para el desplazamiento
y_adjusted_initial = y_initial - 1 + 0.5; % Resta 1 para alinear el centro y agrega 0.5 para el desplazamiento

% Parámetros del entorno
ancho_espacio = 1; % Ancho del espacio en metros
largo_espacio = 1; % Largo del espacio en metros

% Configuración de control PID
kp = 10.0; % Ganancia proporcional
ki = 0.01; % Ganancia integral
kd = 0.0; % Ganancia derivativa

% Variables de control
target_x = 0.5; % Posición x deseada
target_y = 0.9; % Posición y deseada
target_heading = 0; % Orientación deseada
error_integral = 0;
error_previo = 0;

% Bucle principal
while true
    % Obtener la posición actual del robot
    pose = robotat_get_pose(Opti, PololuNumb, 'eulzyx');
    x = pose(1);
    y = pose(2);
    heading = pose(4);
    
    x_ajustada = x - 3 + 0.5; % Resta 3 para alinear el centro y agrega 0.5 para el desplazamiento
    y_ajustada = y - 1 + 0.5; % Resta 1 para alinear el centro y agrega 0.5 para el desplazamiento
   
    % Calcular errores de posición y orientación
    error_x = target_x - x_ajustada;
    error_y = target_y - y_ajustada;
    error_heading = target_heading - heading;

    % Calcular el error integral
    error_integral = error_integral + error_x;

    % Calcular la señal de control PID
    control_signal = kp * error_x + ki * error_integral + kd * (error_x - error_previo);

    % Actualizar velocidades de las ruedas
    v_left = control_signal + 0.5 * error_heading * l;
    v_right = control_signal - 0.5 * error_heading * l;

    % Limitar las velocidades para evitar movimientos bruscos
    max_speed = 0.1; % Máxima velocidad lineal permitida
    v_left = max(min(v_left, max_speed), -max_speed);
    v_right = max(min(v_right, max_speed), -max_speed);

    % Enviar las velocidades al robot
    robotat_3pi_set_wheel_velocities(PI3, v_left, v_right);

    % Verificar si el robot está cerca de una pared y hacerlo rebotar
    if x < 0 || x > ancho_espacio || y < 0 || y > largo_espacio
        % Cambiar la orientación en 180 grados
        target_heading = heading + pi;
        error_integral = 0;
    end

    % Actualizar el error previo
    error_previo = error_x;

    % Pausa para el bucle principal
    pause(0.01);
end

% Detener el robot al final
robotat_3pi_force_stop(PI3);
