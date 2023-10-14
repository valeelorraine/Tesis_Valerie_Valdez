%%                          COLICIONES EN 2D
% Parámetros de la simulación
% num_particulas = 10;
% tiempo_simulacion = 8; % Tiempo de simulación en segundos (30 segundos en este ejemplo)
% paso_tiempo = 0.01;      % Tamaño del paso de tiempo en segundos
% masa_particula = 1.0;    % Masa de cada partícula
% espacio = 10.0;           % Tamaño del espacio a utilizar
% 
% % Distribuir las partículas equidistantemente en el espacio
% num_particulas_por_lado = round(sqrt(num_particulas));
% num_particulas = num_particulas_por_lado^2;
% espaciado_entre_particulas = espacio / num_particulas_por_lado;
% 
% % Inicialización de posiciones y velocidades aleatorias
% posiciones = rand(num_particulas, 2) * espacio;
% velocidades = randn(num_particulas, 2);
% aceleraciones = zeros(num_particulas, 2);
% 
% %% Distribuir las partículas en el espacio
% contador = 1;
% for x = 1:num_particulas_por_lado
%     for y = 1:num_particulas_por_lado
%         posiciones(contador, :) = [(x - 0.5) * espaciado_entre_particulas, ...
%                                      (y - 0.5) * espaciado_entre_particulas];
%         contador = contador + 1;
%     end
% end
% 
% posiciones_i = posiciones;
% % Matrices para almacenar las coordenadas de posición de cada partícula a lo largo del tiempo
% trayectorias_x_acum = zeros(num_particulas, tiempo_simulacion / paso_tiempo);
% trayectorias_y_acum = zeros(num_particulas, tiempo_simulacion / paso_tiempo);
% 
% % Parámetros del potencial de Lennard-Jones
% epsilon = 1.0;
% sigma = 1.0;
% 
% % Diámetro de colisión (asume que todas las partículas tienen el mismo diámetro)
% diametro_colision = 0.1; % Ajusta este valor según sea necesario
% 
% % Bucle de simulación para calcular trayectorias
% num_pasos = tiempo_simulacion / paso_tiempo;
% posiciones_ant = posiciones;
% 
% for paso = 1:num_pasos
%     fuerzas = calcular_fuerza(posiciones_ant, epsilon, sigma);
%     aceleraciones = fuerzas / masa_particula;
%     velocidades = velocidades + (aceleraciones * paso_tiempo);
%     
%     % Manejar colisiones con los límites del espacio (colisiones elásticas)
%     for i = 1:num_particulas
%         for dim = 1:2 % Solo dos dimensiones (x y y)
%             if posiciones(i, dim) < 0 || posiciones(i, dim) > espacio
%                 % La partícula choca con un límite en la dirección dim
%                 % Reflejar la velocidad en esa dirección (colisión elástica)
%                 velocidades(i, dim) = -velocidades(i, dim);
%             end
%         end
%     end
%     
%     posiciones = posiciones + (velocidades * paso_tiempo) + (0.5 * aceleraciones * paso_tiempo^2);
%     posiciones_ant = posiciones;
%     
%     % Almacenar las coordenadas de posición de cada partícula en este paso de tiempo
%     trayectorias_x_acum(:, paso) = posiciones(:, 1);
%     trayectorias_y_acum(:, paso) = posiciones(:, 2);
% end
% 
% % Trazar las trayectorias de todas las partículas
% figure;
% xlabel('X');
% ylabel('Y');
% title('Simulación de Trayectorias de Partículas (2D)');
% grid on;
% axis([0, espacio, 0, espacio]);                           % Delimitar el espacio
% 
% hold on;
% for i = 1:num_particulas
%     plot(posiciones_i(i, 1), posiciones_i(i, 2), 'ro');   % Posición inicial
%     plot(trayectorias_x_acum(i, :), trayectorias_y_acum(i, :));
%     plot(posiciones(i, 1), posiciones(i, 2), 'yp');       % Última posición
% end
% hold off;
% 
% % Función para calcular las fuerzas entre partículas (utilizando el potencial de Lennard-Jones)
% function fuerzas = calcular_fuerza(posiciones, epsilon, sigma)
%     num_particulas = size(posiciones, 1);
%     dimension = size(posiciones, 2);
%     fuerzas = zeros(num_particulas, dimension);
%     
%     for i = 1:num_particulas
%         for j = i+1:num_particulas
%             r_ij = posiciones(i, :) - posiciones(j, :);
%             r = norm(r_ij);
%             
%             % Calcular la fuerza
%             fuerza_ij = 24 * epsilon * (2 * (sigma / r)^12 - (sigma / r)^6) * r_ij / r^2;
%             
%             % Aplicar la fuerza a ambas partículas (ley de acción y reacción)
%             fuerzas(i, :) = fuerzas(i, :) + fuerza_ij;
%             fuerzas(j, :) = fuerzas(j, :) - fuerza_ij;
%         end
%     end
% end

%% POLOLU Conexión con el servidor del Robotat y Pololu
clear all;
clc;

PololuNumb = 5;                           % Pololu a utilizar
ObjNumb = 5;                              % Número de Marker 
Opti = robotat_connect('192.168.50.200'); % Conexión con el robotat
PI3 = robotat_3pi_connect(PololuNumb);    % Conectarse al pololu

%% Probar si el Pololu está conectado
%robotat_3pi_set_wheel_velocities(PI3,-50,50)
robotat_3pi_force_stop(PI3)

%% Variables del Pololu
r = 32*1000/(2*1000);                          % radio de las llantas [m]
l = 96*1000/(2*1000);                          % Dist. llantas desde el centro [m]

%% Ángulo de bearing
% Se encuentra el offset en base a la posición del marcador (angulo de rotación)
clc;
tempBear = robotat_get_pose(Opti,PololuNumb,'eulzyx')
offsetB =  -88.3631;                       % Calibrar orientación pololu
bearing = tempBear(4)-offsetB             % ángulo, lo mas cercano a 0

%% Obteniendo los puntos para los obstáculos
i = 1;
OBS = robotat_get_pose(Opti,6,'eulzyx');  % Obtener la pose del obstáculo (marker)
OBSCoords = zeros(2,1);

for i=1:size(OBS,3)                       % Iteración para obtener poses
    OBSCoords(:,end+1) = [OBS(1,1,i);OBS(1,2,i)]; % Coord. obstáculos
end

%% Mapa con los obstáculos
% Dimensiones de la plataforma
width = 3.8;                                % Ancho
height = 4.8;                               % Alto
resolution = 100;                           % Centímetros

map = binaryOccupancyMap(width,height,resolution); % Mapa 2D, representar y visualizar

% Colocar los puntos referenciados al origen del mapa binario
x = OBSCoords(1,2:end)'+ 3.8/2;             % Mapeo para valores positivos 
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
xpath = transpose(trayectorias_x_acum);
ypath = transpose(trayectorias_y_acum);
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
%          PID orientación
kpO = 10;                                   % Proporcional
kiO = 0.01;                                % Integral
kdO = 0;                                    % Derivativo
EO = 0;                                     % Error
eO_1 = 0;

% Acercamiento exponencial
v0 = 10;                                    % Velocidad
alpha = 0.9;                               % Factor para reducir velocidad al acercarse

%% Failsafe
robotat_3pi_force_stop(PI3);  

% Pararlo por si loquea
pause(1);

%% Controlando la posición
P = 1;

while(1)
    tempBear = robotat_get_pose(Opti,PololuNumb,'eulzyx'); % Coor x y Y
    temp2 = robotat_get_pose(Opti, 4, 'eulzyx')
    bearing = deg2rad(tempBear(4)-offsetB);                % -offset orientarlo bien para el control
    xg = temp2(1);                                         % Coordenada x
    yg = temp2(2);                                         % Coordenada Y
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

robotat_3pi_force_stop(PI3);
pause(1);

%% Trayectoria con dinamica molecular

clear all;
clc;

% Parámetros de la simulación
num_particulas = 10;
tiempo_simulacion = 5;   % Tiempo de simulación en segundos)
paso_tiempo = 0.01;      % Tamaño del paso de tiempo en segundos
masa_particula = 1.0;    % Masa de cada partícula
espacio = 10.0;          % Tamaño del espacio a utilizar

% Distribuir las partículas equidistantemente en el espacio
num_particulas_por_lado = round(num_particulas^(1/2)); % Raíz cuadrada del número de partículas por lado
num_particulas = num_particulas_por_lado^2; % cuadrado de partículas
espaciado_entre_particulas = espacio / num_particulas_por_lado;

% Inicialización de posiciones y velocidades aleatorias
posiciones = rand(num_particulas, 2) * espacio;   % Posiciones iniciales aleatorias en el espacio 2D
velocidades = randn(num_particulas, 2); % Velocidades iniciales aleatorias en 2D
aceleraciones = zeros(num_particulas, 2); % Inicializar aceleraciones a cero en 2D
posiciones_i = posiciones;

% Parámetros del potencial de Lennard-Jones
epsilon = 1.0;
sigma = 1.0;

% Bucle de simulación para calcular trayectorias
num_pasos = tiempo_simulacion / paso_tiempo;
posiciones_ant = posiciones;

for paso = 1:num_pasos
    % Calcular fuerzas entre partículas (utilizando el potencial de Lennard-Jones)
    fuerzas = calcular_fuerza(posiciones_ant, epsilon, sigma);
    
    % Calcular aceleraciones
    aceleraciones = fuerzas / masa_particula;
    
    % Actualizar velocidades utilizando el algoritmo de Verlet
    velocidades = velocidades + (aceleraciones * paso_tiempo);
    
    % Actualizar posiciones utilizando el algoritmo de Verlet
    posiciones = posiciones + (velocidades * paso_tiempo) + (0.5 * aceleraciones * paso_tiempo^2);
    
    % Verificar colisiones con las paredes y aplicar rebote
    for i = 1:num_particulas
        for dim = 1:2
            if posiciones(i, dim) < 0 || posiciones(i, dim) > espacio
                % Invertir la velocidad en la dimensión correspondiente
                velocidades(i, dim) = -velocidades(i, dim);
            end
        end
    end
    
    % Almacenar las coordenadas de posición de cada partícula en este paso de tiempo
    trayectorias_x_acum(:, paso) = posiciones(:, 1);
    trayectorias_y_acum(:, paso) = posiciones(:, 2);
    
    % Actualizar posiciones y aceleraciones para el siguiente paso de tiempo
    posiciones_ant = posiciones;
   
    % Pausa para permitir que MATLAB actualice la figura
    pause(0.01);
end

xlabel('X(m)');
ylabel('Y(m)');
title('Simulación de Trayectorias de Partículas (2D)');
grid on;
axis([0, espacio, 0, espacio]);

% Trazar las trayectorias de todas las partículas en 2D
hold on;
for i = 1:num_particulas
    plot(posiciones_i(i, 1), posiciones_i(i, 2),'o', 'MarkerEdgeColor', 'black', MarkerSize = 4); % Posicion inicial
    plot(trayectorias_x_acum(i, 2:end-1), trayectorias_y_acum(i, 2:end-1));
    plot(posiciones(i, 1), posiciones(i, 2),'pentagram', 'MarkerEdgeColor', 'black', MarkerSize = 4);
end
hold off;


% Función para calcular las fuerzas entre partículas (utilizando el potencial de Lennard-Jones)
function fuerzas = calcular_fuerza(posiciones, epsilon, sigma)
    num_particulas = size(posiciones, 1);
    dimension = size(posiciones, 2);
    fuerzas = zeros(num_particulas, dimension);
    
    for i = 1:num_particulas
        for j = 1:num_particulas
            if i ~= j 

                r_ij = posiciones(i, :) - posiciones(j, :);
                r = norm(r_ij);
                
                % Calcular la fuerza
                fuerza_ij = 24 * epsilon * (2 * (sigma / r)^12 - (sigma / r)^6) * r_ij / r^2;
                
                % Aplicar la fuerza a la partícula i
                fuerzas(i, :) = fuerzas(i, :) + fuerza_ij;
            end
        end
    end
end




% % posiciones = rand(num_particulas, 3) * espacio;   % Posiciones iniciales aleatorias en el espacio
% % velocidades = randn(num_particulas, 3); % Velocidades iniciales aleatorias
% % aceleraciones = zeros(num_particulas, 3); % Inicializar aceleraciones a cero
% 
% % % Matrices para almacenar las coordenadas de posición de cada partícula a lo largo del tiempo
% % trayectorias_x_acum = zeros(num_particulas, tiempo_simulacion / paso_tiempo);
% % trayectorias_y_acum = zeros(num_particulas, tiempo_simulacion / paso_tiempo);
% % trayectorias_z_acum = zeros(num_particulas, tiempo_simulacion / paso_tiempo);
% 
% % % Parámetros del potencial de Lennard-Jones
% % epsilon = 1.0; % Parámetro epsilon del potencial de Lennard-Jones (intensidad)
% % sigma = 1.0;   % Parámetro sigma del potencial de Lennard-Jones (Alcance)
% % 
% %%% Bucle de simulación con el algoritmo de Verlet
% % num_pasos = tiempo_simulacion / paso_tiempo;
% % posiciones_ant = posiciones;
% 
% for paso = 1:num_pasos
%     % Calcular fuerzas entre partículas (utilizando el potencial de Lennard-Jones)
%     fuerzas = calcular_fuerza(posiciones_ant, epsilon, sigma);
%     % Calcular aceleraciones
%     aceleraciones = fuerzas / masa_particula;
%     % Actualizar velocidades utilizando el algoritmo de Verlet
%     %velocidades = velocidades + (aceleraciones * paso_tiempo);
%     %velocidades = posiciones/tiempo;
%     
%     % Actualizar posiciones utilizando el algoritmo de Verlet
%     posiciones = posiciones + (velocidades * paso_tiempo) + (0.5 * aceleraciones * paso_tiempo^2);
%     
%     % Verificar colisiones con las paredes y aplicar rebote
%     % MODIFICAR ESPACIO PLATAFORMA
%     for i = 1:num_particulas
%         for dim = 1:2
%             if posiciones(i, dim) < 0 || posiciones(i, dim) > espacio
%                 % Invertir la velocidad en la dimensión correspondiente
%                 velocidades(i, dim) = -velocidades(i, dim);
%             end
%         end
%     end
%     
%     % Almacenar las coordenadas de posición de cada partícula en este paso de tiempo
%     trayectorias_x_acum(:, paso) = posiciones(:, 1);
%     trayectorias_y_acum(:, paso) = posiciones(:, 2);
%     
%     % Actualizar posiciones y aceleraciones para el siguiente paso de tiempo
%     posiciones_ant = posiciones;
%     
%     % Pausa para permitir que MATLAB actualice la figura
%     pause(0.01);
% end
% 
% %     % Dibujar las partículas en la figura actualizada
% %     scatter(posiciones(:, 1), posiciones(:, 2), 10, 'filled', 'MarkerFaceAlpha', 0.5);
% %     xlabel('X');
% %     ylabel('Y');
% %     title('Simulación de Trayectorias de Partículas (2D)');
% %     grid on;
% %     axis([0, espacio, 0, espacio]);
% 
% % % Función para calcular las fuerzas entre partículas (utilizando el potencial de Lennard-Jones)
% % function fuerzas = calcular_fuerza(posiciones, epsilon, sigma)
% %     num_particulas = size(posiciones, 1);
% %     dimension = size(posiciones, 2);
% %     fuerzas = zeros(num_particulas, dimension);
% %     
% %     for i = 1:num_particulas
% %         for j = i+1:num_particulas
% %             r_ij = posiciones(i, :) - posiciones(j, :);
% %             r = norm(r_ij);       % Calcular la distancia entre particulas
% %             
% %             % Calcular la fuerza
% %             fuerza_ij = 24 * epsilon * (2 * (sigma / r)^12 - (sigma / r)^6) * r_ij / r^2;
% %             
% %             % Aplicar la fuerza a ambas partículas (ley de acción y reacción)
% %             fuerzas(i, :) = fuerzas(i, :) + fuerza_ij;
% %             fuerzas(j, :) = fuerzas(j, :) - fuerza_ij;
% %         end
% %     end
% % end

% %% Desconectar el servidor
% robotat_disconnect(Opti);
% robotat_3pi_disconnect(PI3);