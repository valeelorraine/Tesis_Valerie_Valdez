%             Código de experimentación para pruebas físicas
%                      Algoritmo de física granular

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
r = 32*1000/(2*1000);                    % radio de las llantas [m]
l = 96*1000/(2*1000);                    % Dist. llantas desde el centro [m]

%% Ángulo de bearing
% Se encuentra el offset en base a la posición del marcador (angulo de rotación)
clc;
tempBear = robotat_get_pose(Opti,PololuNumb,'eulzyx')
offsetB =  46.6145;                     % Calibrar orientación pololu
bearing = tempBear(4)-offsetB            % ángulo, lo mas cercano a 0

%% Obteniendo los puntos para los obstáculos
i = 1;
OBS = robotat_get_pose(Opti,2,'eulzyx');  % Obtener la pose del obstáculo (marker)
OBSCoords = zeros(2,1);

for i=1:size(OBS,3)                               % Iteración para obtener poses
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


%% Variables de control del sistema
%          PID orientación
kpO = 10;                                  % Proporcional
kiO = 0.01;                                % Integral
kdO = 0;                                   % Derivativo
EO = 0;                                    % Error
eO_1 = 0;

% Acercamiento exponencial
v0 = 10;                                   % Velocidad
alpha = 0.9;                               % Factor para reducir velocidad al acercarse

%% Failsafe
robotat_3pi_force_stop(PI3);  

% Pararlo por si loquea
pause(1);

%% Controlando la posición
P = 1;

while(1)
    tempBear = robotat_get_pose(Opti,PololuNumb,'eulzyx'); % Coor x y Y
    temp2 = robotat_get_pose(Opti, 2, 'eulzyx')
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

% %% Obtención de posición
% t = 10;
% X1Coords = [];
% vel = [];
% robotat_3pi_set_wheel_velocities(PI3,50,50)
% 
% for i=1:t                                   % Iteración para obtener poses
%     X1 = robotat_get_pose(Opti,6,'eulzyx'); % Obtener la pose de partícula 1 (marker)
%     X1Coords(:,end+1) = [X1(1,1);X1(1,2)];  % Coord. obstáculos
% 
%     if i > 1
%         deltaPos = X1Coords(:, end) - X1Coords(:, end-1);
%         deltaT = 1;                         % Cambio en tiempo,
%         velocidad = deltaPos / deltaT;
%         vel = [vel, velocidad];
%     end
%     pause(1);
% end
% 
% robotat_3pi_force_stop(PI3)

%% Trayectoria CAMBIANDO VALORES
% ESTE NO MUY FUNCIONO
% Parámetros de la simulación
% num_particulas = 1;
% tiempo_simulacion = 10;  % Tiempo de simulación en segundos (30 segundos en este ejemplo)
% paso_tiempo = 0.01;      % Tamaño del paso de tiempo en segundos
% masa_particula = 1.0;    % Masa de cada partícula
% espacio = 3.8;          % Tamaño del espacio a utilizar
% 
% % width = 3.8;                                % Ancho
% % height = 4.8;                               % Alto
% % resolution = 100;                           % Centímetros
% % map = binaryOccupancyMap(width,height,resolution); % Mapa 2D, representar y visualizar
% 
% % Inicialización de posiciones y velocidades aleatorias
% posiciones = robotat_get_pose(Opti,6,'eulzyx');  % Posiciones iniciales aleatorias en el espacio 2D
% velocidades = robotat_3pi_set_wheel_velocities(PI3,-50,50); % Velocidades iniciales aleatorias en 2D
% aceleraciones = zeros(num_particulas, 2); % Inicializar aceleraciones a cero en 2D
% posiciones_i = posiciones;
% 
% % Parámetros del potencial de Lennard-Jones
% epsilon = 1.0;
% sigma = 1.0;
% 
% % Bucle de simulación para calcular trayectorias
% num_pasos = tiempo_simulacion / paso_tiempo;
% posiciones_ant = posiciones;
% 
% for paso = 1:num_pasos
%     % Calcular fuerzas entre partículas (utilizando el potencial de Lennard-Jones)
%     fuerzas = calcular_fuerza(posiciones_ant, epsilon, sigma);
%     
%     % Calcular aceleraciones
%     aceleraciones = fuerzas / masa_particula;
%     
%     % Actualizar velocidades utilizando el algoritmo de Verlet
%     velocidades = velocidades + (aceleraciones * paso_tiempo);
%     
%     % Actualizar posiciones utilizando el algoritmo de Verlet
%     posiciones = robotat_get_pose(Opti,6,'eulzyx');
%     
%     % Verificar colisiones con las paredes y aplicar rebote
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
% xlabel('X');
% ylabel('Y');
% title('Simulación de Trayectorias de Partículas (2D)');
% grid on;
% axis([0, espacio, 0, espacio]);
% 
% % Trazar las trayectorias de todas las partículas en 2D
% hold on;
% for i = 1:num_particulas
%     plot(posiciones_i(i, 1), posiciones_i(i, 2),'o', 'MarkerEdgeColor', 'black', MarkerSize = 4); % Posicion inicial
%     plot(trayectorias_x_acum(i, 2:end-1), trayectorias_y_acum(i, 2:end-1));
%     plot(posiciones(i, 1), posiciones(i, 2),'pentagram', 'MarkerEdgeColor', 'black', MarkerSize = 4);
% end
% hold off;
% 
% 
% % Función para calcular las fuerzas entre partículas (utilizando el potencial de Lennard-Jones)
% function fuerzas = calcular_fuerza(posiciones, epsilon, sigma)
%     num_particulas = size(posiciones, 1);
%     dimension = size(posiciones, 2);
%     fuerzas = zeros(num_particulas, dimension);
%     
%     for i = 1:num_particulas
%         for j = 1:num_particulas
%             if i ~= j 
% 
%                 r_ij = posiciones(i, :) - posiciones(j, :);
%                 r = norm(r_ij);
%                 
%                 % Calcular la fuerza
%                 fuerza_ij = 24 * epsilon * (2 * (sigma / r)^12 - (sigma / r)^6) * r_ij / r^2;
%                 
%                 % Aplicar la fuerza a la partícula i
%                 fuerzas(i, :) = fuerzas(i, :) + fuerza_ij;
%             end
%         end
%     end
% end


%% TRAYECTORIA
%Codigo de mi amigo virtual
% Parámetros de la simulación
% tiempo_simulacion = 10;     % Tiempo de simulación en segundos
% paso_tiempo = 0.01;         % Tamaño del paso de tiempo en segundos
% espacio = 3.8;              % Tamaño del espacio a utilizar
% 
% % Inicialización de posiciones y velocidades
% num_particulas = 1;
% posiciones = robotat_get_pose(Opti,6,'eulzyx'); % Inicializar posiciones a cero en 2D
% robotat_3pi_set_wheel_velocities(PI3,30,30); % Inicializar velocidades a cero en 2D
% 
% % Establecer la velocidad inicial del robot 3pi+
% robotat_3pi_set_wheel_velocities(PI3, 30, 30);
% 
% 
% % Parámetros del potencial de Lennard-Jones
% epsilon = 1.0;
% sigma = 1.0;
% 
% % Bucle de simulación para calcular trayectorias
% num_pasos = tiempo_simulacion / paso_tiempo;
% 
% for paso = 1:num_pasos
%     % Obtener la posición actual del robot desde OptiTrack
%     posiciones_robot = robotat_get_pose(Opti, PololuNumb, 'eulzyx');
%     
%     % Calcular la fuerza entre el robot y la partícula (utilizando el potencial de Lennard-Jones)
%     fuerza = calcular_fuerza(posiciones, posiciones_robot, epsilon, sigma);
%     
%     % Calcular la aceleración de la partícula
%     aceleracion = fuerza / masa_particula;
%     
%     % Actualizar la velocidad de la partícula
%     velocidad = velocidad + (aceleracion * paso_tiempo);
%     
%     % Actualizar la posición de la partícula
%     posicion = posicion + (velocidad * paso_tiempo);
%     
%     % Enviar comandos de control al robot Pololu 3pi+ para seguir la posición de la partícula
%     robotat_3pi_set_target_position(PI3, posicion(1), posicion(2));
%     
%     % Pausa para permitir que el robot se mueva y que MATLAB actualice la figura
%     pause(paso_tiempo);
% end
% 
% % Función para calcular la fuerza entre el robot y la partícula (utilizando el potencial de Lennard-Jones)
% function fuerza = calcular_fuerza(posicion_particula, posicion_robot, epsilon, sigma)
%     r_ij = posicion_particula - posicion_robot;
%     r = norm(r_ij);
%     
%     % Calcular la fuerza
%     fuerza = 24 * epsilon * (2 * (sigma / r)^12 - (sigma / r)^6) * r_ij / r^2;
% end

%% Trayectoria con graficación
% Parámetros de la simulación
num_particulas = 1;
tiempo_simulacion = 1;      % Tiempo de simulación en segundos
paso_tiempo = 0.01;         % Tamaño del paso de tiempo en segundos
espacio = 1;                % Tamaño del espacio a utilizar
masa_particula = 1;

% Inicialización de posiciones y velocidades
posicion = robotat_get_pose(Opti,2,'eulzyx'); % Inicializar posiciones a cero en 2D
robotat_3pi_set_wheel_velocities(PI3,30,30);  % Inicializar velocidades a cero en 2D

% Parámetros del potencial de Lennard-Jones
epsilon = 1.0;
sigma = 1.0;

% Bucle de simulación para calcular trayectorias
num_pasos = tiempo_simulacion / paso_tiempo;

% Inicializar matrices para almacenar la trayectoria del robot
trayectoria_x = zeros(1, num_pasos);
trayectoria_y = zeros(1, num_pasos);

% Crear el mapa con obstáculos
width = 3.8;            % Ancho
height = 4.8;           % Alto
resolution = 100;       % Centímetros
map = binaryOccupancyMap(width, height, resolution);

% Obtener los puntos de los obstáculos desde OptiTrack
i = 1;
OBS = robotat_get_pose(Opti, 3, 'eulzyx'); % Obtener la pose del obstáculo (marker)
OBSCoords = zeros(2, size(OBS, 3));

for i = 1:size(OBS, 3) % Iteración para obtener poses
    OBSCoords(:, i) = [OBS(1, 1, i); OBS(1, 2, i)]; % Coord. obstáculos
end

% Colocar los puntos referenciados al origen del mapa binario
x = OBSCoords(1, :) + 3.8 / 2; % Mapeo para valores positivos
y = OBSCoords(2, :) + 4.8 / 2;
setOccupancy(map, [x; y]', ones(size(x, 2), 1)); % Valores en x y y
inflate(map, 0.15); % Para inflar los obstáculos

% Crear una figura para el mapa
figure;
show(map); % Mostrar el mapa con los obstáculos inflados

% Bucle de simulación para calcular trayectorias
for paso = 1:num_pasos
    % Obtener la posición actual del robot desde OptiTrack
    posicion = robotat_get_pose(Opti, PololuNumb, 'eulzyx');
    
    % Calcular la fuerza entre el robot y la partícula (utilizando el potencial de Lennard-Jones)
    fuerza = calcular_fuerza(posicion, epsilon, sigma);
    
    % Calcular la aceleración del robot
    aceleracion = fuerza / masa_particula;
    
    % Actualizar la velocidad del robot
    velocidad = velocidad + (aceleracion * paso_tiempo);
    
    % Actualizar la posición del robot
    posicion = posicion + (velocidad * paso_tiempo);
    
    % Verificar colisiones con las paredes y aplicar rebote
    for i = 1:num_particulas
        for dim = 1:2
            if posicion(i, dim) < 0 || posicion(i, dim) > espacio
                % Invertir la velocidad en la dimensión correspondiente
                velocidad(i, dim) = -velocidad(i, dim);
            end
        end
    end
   
    % Guardar la posición actual en la trayectoria
    trayectoria_x(paso) = posicion(1);
    trayectoria_y(paso) = posicion(2);
    posicion_ant = trayectoria_x(paso) + trayectoria_y(paso);
    
    % Actualizar la visualización de la trayectoria
    scatter(trayectoria_x, trayectoria_y, 'filled'); % Graficar la trayectoria en azul
    xlabel('X');
    ylabel('Y');
    title('Validación del Algoritmo de física granular');
    grid on;
    drawnow; % Actualizar la figura
end

robotat_3pi_force_stop(PI3)    % Parar el robot

% Función para calcular las fuerzas entre partículas (utilizando el potencial de Lennard-Jones)
function fuerzas = calcular_fuerza(posicion, epsilon, sigma)
    num_particulas = size(posicion, 1);
    dimension = size(posicion, 2);
    fuerzas = zeros(num_particulas, dimension);
    
    for i = 1:num_particulas
        for j = 1:num_particulas
            if i ~= j 

                r_ij = posicion(i, :) - posicion(j, :);
                r = norm(r_ij);
                
                % Calcular la fuerza
                fuerza_ij = 24 * epsilon * (2 * (sigma / r)^12 - (sigma / r)^6) * r_ij / r^2;
                
                % Aplicar la fuerza a la partícula i
                fuerzas(i, :) = fuerzas(i, :) + fuerza_ij;
            end
        end
    end
end
%% Trayectoria con dinamica molecular
% clear all;
% clc;
% 
% % Parámetros de la simulación
% num_particulas = 10;
% tiempo_simulacion = 5;  % Tiempo de simulación en segundos (30 segundos en este ejemplo)
% paso_tiempo = 0.01;      % Tamaño del paso de tiempo en segundos
% masa_particula = 1.0;    % Masa de cada partícula
% espacio = 10.0;          % Tamaño del espacio a utilizar
% 
% % Distribuir las partículas equidistantemente en el espacio
% num_particulas_por_lado = round(num_particulas^(1/2)); % Raíz cuadrada del número de partículas por lado
% num_particulas = num_particulas_por_lado^2; % cuadrado de partículas
% espaciado_entre_particulas = espacio / num_particulas_por_lado;
% 
% % Inicialización de posiciones y velocidades aleatorias
% posiciones = rand(num_particulas, 2) * espacio;   % Posiciones iniciales aleatorias en el espacio 2D
% velocidades = randn(num_particulas, 2); % Velocidades iniciales aleatorias en 2D
% aceleraciones = zeros(num_particulas, 2); % Inicializar aceleraciones a cero en 2D
% posiciones_i = posiciones;
% 
% % Parámetros del potencial de Lennard-Jones
% epsilon = 1.0;
% sigma = 1.0;
% 
% % Bucle de simulación para calcular trayectorias
% num_pasos = tiempo_simulacion / paso_tiempo;
% posiciones_ant = posiciones;
% 
% for paso = 1:num_pasos
%     % Calcular fuerzas entre partículas (utilizando el potencial de Lennard-Jones)
%     fuerzas = calcular_fuerza(posiciones_ant, epsilon, sigma);
%     
%     % Calcular aceleraciones
%     aceleraciones = fuerzas / masa_particula;
%     
%     % Actualizar velocidades utilizando el algoritmo de Verlet
%     velocidades = velocidades + (aceleraciones * paso_tiempo);
%     
%     % Actualizar posiciones utilizando el algoritmo de Verlet
%     posiciones = posiciones + (velocidades * paso_tiempo) + (0.5 * aceleraciones * paso_tiempo^2);
%     
%     % Verificar colisiones con las paredes y aplicar rebote
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
% xlabel('X');
% ylabel('Y');
% title('Simulación de Trayectorias de Partículas (2D)');
% grid on;
% axis([0, espacio, 0, espacio]);
% 
% % Trazar las trayectorias de todas las partículas en 2D
% hold on;
% for i = 1:num_particulas
%     plot(posiciones_i(i, 1), posiciones_i(i, 2),'o', 'MarkerEdgeColor', 'black', MarkerSize = 4); % Posicion inicial
%     plot(trayectorias_x_acum(i, 2:end-1), trayectorias_y_acum(i, 2:end-1));
%     plot(posiciones(i, 1), posiciones(i, 2),'pentagram', 'MarkerEdgeColor', 'black', MarkerSize = 4);
% end
% hold off;
% 
% 
% % Función para calcular las fuerzas entre partículas (utilizando el potencial de Lennard-Jones)
% function fuerzas = calcular_fuerza(posiciones, epsilon, sigma)
%     num_particulas = size(posiciones, 1);
%     dimension = size(posiciones, 2);
%     fuerzas = zeros(num_particulas, dimension);
%     
%     for i = 1:num_particulas
%         for j = 1:num_particulas
%             if i ~= j 
% 
%                 r_ij = posiciones(i, :) - posiciones(j, :);
%                 r = norm(r_ij);
%                 
%                 % Calcular la fuerza
%                 fuerza_ij = 24 * epsilon * (2 * (sigma / r)^12 - (sigma / r)^6) * r_ij / r^2;
%                 
%                 % Aplicar la fuerza a la partícula i
%                 fuerzas(i, :) = fuerzas(i, :) + fuerza_ij;
%             end
%         end
%     end
% end
% 
% 
% 
% 
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

%% Desconexion del agente
%robotat_3pi_disconnect(PI3)
