% Valerie Valdez                                        Carné: 19659
% Universidad del Valle de Guatemala                    Sección: 20

%                            Avances tesis
%                     Rapidly exploring Random Tree

%% Pruebas iniciales para el control del Pololu 3Pi+ utilizando RRT
%        Conexión con el servidor del Robotat y el Pololu
clear all;
clc;

PololuNumb = 4;                           % Pololu a utilizar
ObjNumb = 7;                              % Número de Marker 
Opti = robotat_connect();                 % Conexión con el robotat
PI3 = robotat_3pi_connect(PololuNumb);    % Conectarse al pololu


%% Variables del Pololu
r = 32/(2*1000);                          % radio de las llantas [m]
l = 96/(2*1000);                          % Dist. llantas desde el centro [m]

%% Ángulo de bearing
% Se encuentra el offset en base a la posición del marcador (angulo de
% rotación)
clc;
tempBear = robotat_get_pose(Opti,PololuNumb,'eulzyx')
offsetB =  -89.9307;    
% Calibrar orientación pololu
bearing = tempBear(4)-offsetB             % ángulo, lo mas cercano a 0

%% Obteniendo los puntos para los obstáculos
i = 1;
OBS = robotat_get_pose(Opti,7,'eulzyx');
%OBS(:,:,end+1) = robotat_get_pose(Opti,4,'eulzyx');
%OBS(:,:,end+1) = robotat_get_pose(Opti,11,'eulzyx');
% OBS(:,:,end+1) = robotat_get_pose(Opti,14,'eulzyx');
% OBS(:,:,end+1) = robotat_get_pose(Opti,17,'eulzyx');
% OBS(:,:,end+1) = robotat_get_pose(Opti,19,'eulzyx');
% OBS(:,:,end+1) = robotat_get_pose(Opti,20,'eulzyx');
% OBS(:,:,end+1) = robotat_get_pose(Opti,21,'eulzyx');
% OBS(:,:,end+1) = robotat_get_pose(Opti,22,'eulzyx');
OBSCoords = zeros(2,1);

for i=1:size(OBS,3)
    OBSCoords(:,end+1) = [OBS(1,1,i);OBS(1,2,i)]; % Coord. obstáculos
end

%% Mapa con los obstáculos
width = 3.8;                    % Ancho
height = 4.8;                   % Alto
resolution = 100;               % Centímetros

map = binaryOccupancyMap(width,height,resolution); % Mapa 2D, representar y visualizar
% Colocar los puntos referenciados al origen del mapa binario
x = OBSCoords(1,2:end)'+ 3.8/2;     % Mapeo para valores positivos 
y = OBSCoords(2,2:end)'+ 4.8/2;
setOccupancy(map, [x y], ones(size(x,2),1)) % Valores en x y y 
inflate(map,0.15);                          % Para inflar los obstaculos
show(map)                                   % Mostrar el mapa creado
%% Trayectoria utilizando RRT
% Espacios de estado que considera dirección y posicion para mover el carro
%            X        Y       Rotación 
bounds = [[0 3.8]; [0 4.8]; [-pi pi]];
ss = stateSpaceDubins(bounds); % Guarda parámetros y estados compuestos por vec. de estados
ss.MinTurningRadius = 0.2;     % Radio de giro (puede dar vueltas en rango 20cm)

% Crear mapa para validar
stateValidator = validatorOccupancyMap(ss); %Valida estados y discretiza mov. basados en el mapa
stateValidator.Map = map;                   % Conjunto de valores = mapa binario
stateValidator.ValidationDistance = 0.05;

% Planer con espacio de estados (generar un estado random)
planner = plannerRRT(ss,stateValidator);    % Árbol para conectar salida con goal
planner.MaxConnectionDistance = 0.5;        % Magnitud maxima para moverse
planner.MaxIterations = 100000;
planner.GoalReachedFcn = @exampleHelperCheckIfGoal; % checks if a feasible path reaches the goal within a set threshold. 

start_temp = robotat_get_pose(Opti,PololuNumb,'eulzyx'); % Posicion pololu inicial
start_x = start_temp(1)+3.8/2;                           % Coor. + desfase
start_y = start_temp(2)+4.8/2;
start = [start_x,start_y,deg2rad(start_temp(4)-offsetB)];

goal_temp = robotat_get_pose(Opti,ObjNumb,'eulzyx');     % Posición de la meta
goal_x = goal_temp(1)+3.8/2;
goal_y = goal_temp(2)+4.8/2;
goal = [goal_x,goal_y+0.1,-pi/2];

%% Generar la trayectoria
clc
rng default                                              % para resultado repetible
[pthObj, solnInfo] = plan(planner,start,goal);           % Calcular trayectoria factible
solnInfo                                                 % Estructura

%% Remapeando los valores al OptiTrack
% Mapeo inverso
interpolate(pthObj,100)               % Interpolar val. obtener + puntos en el camino
xpath = pthObj.States(:,1)-3.8/2;
ypath = pthObj.States(:,2)-4.8/2;
Dpath = [xpath,ypath];
%% Resultados
show(map)
hold on
% Trazar árbol de búsqueda
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-');

% Interpolar y trazar la trayectoria
plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2)
plot(start(1),start(2),'ro')          % Mostrar inicio y meta en el mapa
plot(goal(1),goal(2),'mo')
hold off

%% Variables de control del sistema
%           PID orientación
kpO = 10; 
kiO = 0.001; 
kdO = 0;
EO = 0;
eO_1 = 0;

% Acercamiento exponencial
v0 = 10; 
alpha = 0.95;

%% Failsafe
robotat_3pi_force_stop(PI3);       % Pararlo
pause(1);

%% Controlando la posición
P = 1;

while(P<size(xpath,2))
    tempBear = robotat_get_pose(Opti,PololuNumb,'eulzyx'); % Coor x y Y
    bearing = deg2rad(tempBear(4)-offsetB);                % -offset orientarlo bien para el control
    xg = 0                                       % punto de la trayectoria p
    yg = 0                                       % Arreglo de la trayectoria
                                                           % el punto cambia conforme se acerca
    x = tempBear(1);
    y = tempBear(2);
    e = [xg-x;yg-y];                                       % Error
    thetag = atan2(e(2), e(1));                            % Angulo entre puntos deseados
    
    eP = norm(e);                                          % Dist, entre pinicial y meta
    eO = thetag - bearing; 
    eO = atan2(sin(eO), cos(eO));                          % Error orientacion
    
%      if(P < length(xpath))                                  % Cada vez que la dist < 0.1, p aumenta y cambia de punto
%        if(eP<0.1)
%          P = P+1;
%        end
%      end
    
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

robotat_3pi_force_stop(PI3);
pause(1);

%% Desconectar el servidor
robotat_disconnect(Opti);
robotat_3pi_disconnect(PI3);