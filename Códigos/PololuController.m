%% Controlador para el movimiento del Pololu 3Pi+
%   Jose Alvarez
%   Valerie Lorraine

%% Conexion con el servidor del Robotat y el carrito
clear all;
clc;
Opti = robotat_connect('192.168.50.200');
PI3 = robotat_3pi_connect(1);

%% Variables del carrito
r = 32/(2*1000); %radio de las llantas en m
l = 96/(2*1000); %distancia de las llantas desde el centro en m

%% Obteniendo los puntos para los obstaculos
i = 1;

%OBS = robotat_get_pose(Opti,1,'eulzyx');
OBS(:,:,end+1) = robotat_get_pose(Opti,1,'eulzyx');
%OBS(:,:,end+1) = robotat_get_pose(Opti,20,'eulzyx');
%OBS(:,:,end+1) = robotat_get_pose(Opti,21,'eulzyx');
%OBS(:,:,end+1) = robotat_get_pose(Opti,22,'eulzyx');

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
show(map);

%% Arreglando el angulo de bearing
clc;
tempBear = robotat_get_pose(Opti,3,'eulzyx')
bearing = tempBear(4)+5;

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
    tempBear = robotat_get_pose(Opti,1,'eulzyx');
    bearing = tempBear(4)+5;  
    
    goal = robotat_get_pose(Opti,1,'eulzyx');
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

   robotat_3pi_set_wheel_velocities(PI3,v_left_wheel,v_rigth_wheel); 
end
%% Desconexion con el servidor
robotat_disconnect(Rport);
clear all;
