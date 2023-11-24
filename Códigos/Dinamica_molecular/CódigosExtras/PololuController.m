%% Controlador para el movimiento del Pololu 3Pi+
%   Jose Alvarez
%   Valerie Lorraine

%% Conexion con el servidor del Robotat y el carrito
clear all;
clc;

%% Conexiónal pololu
PololuNumb = 2;                           % Pololu a utilizar
ObjNumb = 2;                              % Número de Marker 
Opti = robotat_connect(); % Conexión con el robotat
PI3 = robotat_3pi_connect(PololuNumb);    % Conectarse al pololu

%% Variables del carrito
r = 32/(2*1000); %radio de las llantas en m
l = 96/(2*1000); %distancia de las llantas desde el centro en m

%% Ángulo de bearing
% Se encuentra el offset en base a la posición del marcador (angulo de
% rotación)
clc;
tempBear = robotat_get_pose(Opti,PololuNumb,'eulzyx')
offsetB =  40.7762 ;    
% Calibrar orientación pololu
bearing = tempBear(4)-offsetB             % ángulo, lo mas cercano a 0


%% Obteniendo los puntos para los obstaculos
i = 1;
OBS = robotat_get_pose(Opti,3,'eulzyx');
OBSCoords = zeros(2,1);

for i=1:size(OBS,3)
    OBSCoords(:,end+1) = [OBS(1,1,i);OBS(1,2,i)];
end

%% Creando el mapa para trabajar
width = 3.8;
height = 4.8;
resolution = 10;
map = binaryOccupancyMap(width,height,resolution);
x = OBSCoords(1,:)'+ 3.8/2;
y = OBSCoords(2,:)'+ 4.8/2;
setOccupancy(map, [x y], ones(6,1))
inflate(map,0.15);                          % Para inflar los obstaculos
show(map);

%% variables de control del sistema

% PID posición
kpP = 1;
kiP = 0.0001; 
kdP = 0.5;
EP = 0;
eP_1 = 0;

% PID orientación
kpO = 5; %2.5
kiO = 0.0001; 
kdO = 0;
EO = 0;
eO_1 = 0;

% Acercamiento exponencial
v0 = 3;
alpha = 0.7;

%% Failsafe
robotat_3pi_force_stop(PI3);

%% Controlando la posicion

while(1)
    tempBear = robotat_get_pose(Opti,PololuNumb,'eulzyx');
    bearing = tempBear(4)+5;  
    
    goal = robotat_get_pose(Opti,3,'eulzyx');
    xg = goal(1);
    yg = goal(2);
    
    x = tempBear(1);
    y = tempBear(2);
    e = [xg-x;yg-y];
    thetag = atan2(e(2), e(1));
    
    eP = norm(e);
    eO = thetag - bearing;
    eO = atan2(sin(eO), cos(eO));

    % Control de velocidad lineal
    kP = v0 * (1-exp(-alpha*eP^2)) / eP;
    v = kP*eP*0;
    
    % Control de velocidad angular
    eO_D = eO - eO_1;
    EO = EO + eO;
    w = kpO*eO + kiO*EO + kdO*eO_D;
    eO_1 = eO;
    
    % Se combinan los controladores
    u = [v; w];
 
	v_rigth_wheel = (u(1) + u(2) *l)/r ;
	v_left_wheel = (u(1) - u(2) *l)/r ;
%    if (v_rigth_wheel > 50 )
%        v_rigth_wheel = 50;
%    end 
%    if (v_left_wheel > 50 )
%        v_right_wheel = 50;
%    end 

    robotat_3pi_set_wheel_velocities(PI3,v_left_wheel,v_rigth_wheel); 
end
%% Desconexion con el servidor
robotat_disconnect(PI3);
clear all;
