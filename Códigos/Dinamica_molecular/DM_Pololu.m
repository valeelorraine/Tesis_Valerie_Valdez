%%                          COLICIONES EN 2D

%% Limpiar todo
clear all;
clc;

%% POLOLU Conexión con el servidor del Robotat y Pololu
PololuNumb = 4;                           % Pololu a utilizar
ObjNumb = 4;                              % Número de Marker 
Opti = robotat_connect();                 % Conexión con el robotat
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

%% Ángulo de bearing
% Se encuentra el offset en base a la posición del marcador (angulo de rotación)
clc;
tempBear = robotat_get_pose(Opti,PololuNumb,'eulzyx')
offsetB =  134.8168;                     % Calibrar orientación pololu
bearing = tempBear(4)-offsetB            % ángulo, lo mas cercano a 0

%% Variables de control del sistema
%           PID orientación
kpO = 10; 
kiO = 0.001; 
kdO = 0;
EO = 0;
eO_1 = 0;

% Acercamiento exponencial
v0 = 6; 
alpha = 0.95;

%% Controlador
P = 1;

while(1)
    tempBear = robotat_get_pose(Opti,PololuNumb,'eulzyx'); % Coor x y Y
    temp2 = robotat_get_pose(Opti, 2, 'eulzyx')
    bearing = deg2rad(tempBear(4)-offsetB);                % -offset orientarlo bien para el control
    xg = 0;                                         % Coordenada x
    yg = 0;                                         % Coordenada Y
                                                           % el punto cambia conforme se acerca
    x = tempBear(1);
    y = tempBear(2);
    e = [xg-x;yg-y];                                       % Error
    thetag = atan2(e(2), e(1));                            % Angulo entre puntos deseados
    
    eP = norm(e);                                          % Dist, entre pinicial y meta
    eO = thetag - bearing; 
    eO = atan2(sin(eO), cos(eO));                          % Error orientacion
    
   % if(P < length(xpath))                                % Cada vez que la dist < 0.1, p aumenta y cambia de punto
   %    if(eP<0.1)
   %      P = P+1;
   %    end
   %  end
    
    % Control de velocidad lineal exponencial
    kP = v0 * (1-exp(-alpha*eP^2)) / eP;
    v = kP*eP;
    
    % Control de velocidad angular
    eO_D = eO - eO_1;
    EO = EO + eO;
    w = kpO*eO + kiO*EO + kdO*eO_D;
    eO_1 = eO;
    
    % Se combinan los controladores
    u = [v; w];
 
    v_rigth_wheel = (u(1) + u(2) *l)/r 
    v_left_wheel = (u(1) - u(2) *l)/r 

    robotat_3pi_set_wheel_velocities(PI3,v_left_wheel,v_rigth_wheel); 
end

