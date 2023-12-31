%%                          COLICIONES EN 2D
% Funcion de dinámica molecular para una partícula

%% Limpiar todo
clear all;
clc;

%% POLOLU Conexión con el servidor del Robotat y Pololu
PololuNumb = 3;                             % # de Pololu a utilizar
ObjNumb = 3;                                  % # de Marker colocado en el Pololu
Opti = robotat_connect();                   % Conectar con el robotat
pause(2)
PI3 = robotat_3pi_connect(PololuNumb);      % Conectarse al pololu


%% Desconexion del agente
robotat_3pi_disconnect(PI3)                 % Desconectarme del robot

%% Probar si el Pololu está conectado
robotat_3pi_set_wheel_velocities(PI3,-50,50)
% Girar sobre sí mismo

%% Ir en linea recta
robotat_3pi_set_wheel_velocities(PI3,50,50) % Avanzar 
%% Stop de emergencia
robotat_3pi_force_stop(PI3)                  

%% Variables del Pololu
r = 32/(2*1000);                            % Radio de las ruedas en metros
l = 96/(2*1000);                            % Distancia entre ruedas en metros

%% Ángulo de bearing
% Se encuentra el offset en base a la posición del marcador (angulo de rotación)
clc;
tempBear = robotat_get_pose(Opti,PololuNumb,'eulzyx')
offsetB =  177.3402  %-137.4042 ; %177.3402;                       % Calibrar orientación pololu
bearing = tempBear(4)-offsetB               % Ángulo, lo mas cercano a 0

%% Variables de control del sistema
%          PID orientación
kpO = 12;  %1
kiO = 0.002; 
kdO = 0;
EO = 0;
eO_1 = 0;               % Error de orientación

% Acercamiento exponencial
v0 = 15 % 30;                % estaba en 10, Velocidad inicial
alpha = 0.95;           % 95

%% Parámetros del potencial de Lennard-Jones
epsilon = 1.0;          
sigma = 1.0;
num_particulas = 1; 
tiempo_simulacion = 10;  % Tiempo de simulación en segundos (30 segundos en este ejemplo)
paso_tiempo = 0.01;      % Tamaño del paso de tiempo en segundos
masa_particula = 1.0;    % Masa de cada partícula
espacio = 1.0;           % Tamaño del espacio a utilizar


% Bucle de simulación para calcular trayectorias
num_pasos = tiempo_simulacion / paso_tiempo;

%% Trayectoria con dinamica molecular
%posiciones = zeros(1,2);
%posiciones = rand(num  _particulas, 2)*espacio;
posiciones = [0.5,0.5] 
posiciones0 = posiciones;
velocidades = [2,1];                      % Velocidades iniciales aleatorias en 2D
aceleraciones = zeros(num_particulas, 2); % Inicializar aceleraciones a cero en 2D
posiciones_ant = posiciones;
x_path = [];
y_path = [];
trayectoria = [];

for paso = 1:230
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
            if posiciones(i, dim) < 0 || posiciones(i, dim) >= espacio
                % Invertir la velocidad en la dimensión correspondiente
                velocidades(i, dim) = -velocidades(i, dim);
            end
        end
    end

    % Actualizar posiciones y aceleraciones para el siguiente paso de tiempo
    posiciones_ant = posiciones;
    
    % Dibujar las partículas en la figura actualizada
    scatter(posiciones(:, 1), posiciones(:, 2), 10, 'filled', 'MarkerFaceAlpha', 0.5);
    x_path(end+1) = posiciones(:,1);
    y_path(end+1) = posiciones(:,2);
    trayectoria(end+1,:) = posiciones
    xlabel('X (m)');
    ylabel('Y (m)');    
    title('Simulación de Trayectorias de Partículas (2D)');
    grid on;
    axis([0, espacio, 0, espacio]);
    
    % Pausa para permitir que MATLAB actualice la figura
    pause(paso_tiempo);
end

%% Suavizado y reducción de puntos
largo = length(trayectoria)/8; % longitud de datos dividido 4
xy_puntos = [];

for j = 1:largo
    xy_puntos(end+1,:) = trayectoria(8*j,:);
end
xy_puntos(end+1,:) = trayectoria(end,:);

% interpolación mediante b-splines y generando puntos cada 0.25 en x para mayor fineza
%trayec = bsplinepolytraj(xy_puntos',[0,length(xy_puntos)-1],0:0.25:length(xy_puntos)-1);
trayec = bsplinepolytraj(xy_puntos',[0,length(xy_puntos)],0:0.08:length(xy_puntos));
trayec = trayec';

% 1:10 son 300, 1:8 son 375, 1:5 sonnnnn 600 puntos, 1:3 son 1000 puntos, 1:2 son 1500
short_trac = [0,0];
for i = 1:8:3000
    short_trac(end+1,:) = trayec(:,i);
end 

% trayec = trayec';

short_trac = short_trac(2:end,:);

%% Graficar la trayectoria CON TRAYEC NADA MAS
%scatter(x(1,:),x(2,:), 'green','filled');
figure(2);
hold on;
grid on;
title('Suavizado en la simulación de Trayectorias de Partículas (2D)');
plot(x_path,y_path,'green')
xlabel('X(m)');
ylabel('Y(m)');
axis([0, espacio, 0, espacio]);
drawnow

for i = 1:num_particulas
    plot(posiciones0(i, 1), posiciones0(i, 2),'o', 'MarkerEdgeColor', 'red', 'MarkerSize', 4); % Posicion inicial
    plot(trayec(:,1), trayec(:,2));
    plot(trayec(i, 1), trayec(i, 2),'pentagram', 'MarkerEdgeColor', 'black');
end
pbaspect([1 1 1])
hold off;

%%  Graficar 
figure(2);
hold on;
grid on;
title('Suavizado en la simulación de Trayectorias de Partículas (2D)');
plot(x_path,y_path,'green')
xlabel('X(m)');
ylabel('Y(m)');
axis([0, espacio, 0, espacio]);
drawnow

for i = 1:num_particulas
    plot(posiciones0(i, 1), posiciones0(i, 2),'o', 'MarkerEdgeColor', 'red', 'MarkerSize', 4); % Posicion inicial
    plot(short_trac(:,1), short_trac(:,2));
    plot(short_trac(end,1), short_trac(end,2),'pentagram', 'MarkerEdgeColor', 'black');
   % plot(posiciones(i, 1), posiciones(i, 2),'pentagram', 'MarkerEdgeColor', 'black');
end
pbaspect([1 1 1])
hold off;
%% Trayectoria para probar mapas
xpath = [];
ypath = [];

%% Crear un gif
% % Propiedades del archivo .GIF que se va a generar
% % Nombre
% filename = 'DMolecularFisico.gif';
% % Tiempo en segundos que dura cada plot en el GIF
% DelayTime = 0.1; 
% f = figure(2)                         % Crear una figura


%% Colocarlo en la posicion inicial
v0 = 5;
tempBear = robotat_get_pose(Opti,PololuNumb,'eulzyx'); % Coor x y Y
x = tempBear(1);
y = tempBear(2);  
xg = posiciones0(1);                                       % Coordenada x
yg = posiciones0(2);    
e = [xg-x;yg-y];
eP = norm(e);  

while(eP > 1)
    tempBear = robotat_get_pose(Opti,PololuNumb,'eulzyx'); % Coor x y Y
    bearing = deg2rad(tempBear(4)-offsetB);                % -offset orientarlo bien para el control

    xg = posiciones0(1);                                   % Coordenada x
    yg = posiciones0(2);                                
    
    % Coordenada Y                                                         % el punto cambia conforme se acerca
    x = tempBear(1);
    y = tempBear(2);    
    e = [xg-x;yg-y];                                       % Error
    thetag = atan2(e(2), e(1));                            % Angulo entre puntos deseados
    
    eP = norm(e);                                          % Dist, entre pinicial y meta
    eO = thetag - bearing; 
    eO = atan2(sin(eO), cos(eO));                          % Error orientacion
    
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
    
%     if v_rigth_wheel > 100
%         v_rigth_wheel = 100;
%     end 
%     if v_rigth_wheel < -100
%         v_rigth_wheel = -100;
%     end  
%     if v_left_wheel > 100
%         v_left_wheel = 100;
%     end 
%     if v_left_wheel < -100
%         v_left_wheel = -100;
%     end 

    robotat_3pi_set_wheel_velocities(PI3,v_left_wheel,v_rigth_wheel); 
    posicionx = tempBear(1);
    posiciony = tempBear(2); 
    posiciones = [posicionx,posiciony];
    posiciones_ant = posiciones;
end

robotat_3pi_force_stop(PI3)

%          PID orientación
kpO = 17;  %1
kiO = 0.002; 
kdO = 0;
EO = 0;
eO_1 = 0;               % Error de orientación

% Acercamiento exponencial
v0 = 20;                % estaba en 10, Velocidad inicial
alpha = 0.95;           % 95

%%  Colocar en la orientación correcta
kpO = 16;  %1
tempBear = robotat_get_pose(Opti,PololuNumb,'eulzyx'); % Coor x y Y
x = tempBear(1);
y = tempBear(2);  
xg = posiciones0(1);                                   % Coordenada x
yg = posiciones0(2);    
e = [xg-x;yg-y]; 
thetag = atan2(e(2), e(1));
eO = thetag - bearing; 
eO = atan2(sin(eO), cos(eO));  

while(abs(eO) > 0.01)
    tempBear = robotat_get_pose(Opti,PololuNumb,'eulzyx'); % Coor x y Y
    bearing = deg2rad(tempBear(4)-offsetB);                % -offset orientarlo bien para el control
    xg = 1.5;                                      % Coordenada x
    yg = 1.5;                                
    
    % Coordenada Y                                                      % el punto cambia conforme se acerca
    x = tempBear(1);
    y = tempBear(2);    
    e = [xg-x;yg-y];                                       % Error
    thetag = atan2(e(2), e(1));                            % Angulo entre puntos deseados
    
    eP = norm(e);                                          % Dist, entre pinicial y meta
    eO = thetag - bearing; 
    eO = atan2(sin(eO), cos(eO))                          % Error orientacion
    
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
    v_rigth_wheel = (u(1)*0 + u(2) *l)/r 
    v_left_wheel = (u(1)*0 - u(2) *l)/r 
    
    robotat_3pi_set_wheel_velocities(PI3,v_left_wheel,v_rigth_wheel); 
    posicionx = tempBear(1);
    posiciony = tempBear(2); 
    posiciones = [posicionx,posiciony];
    posiciones_ant = posiciones;
end

robotat_3pi_force_stop(PI3)

%          PID orientación
kpO = 15;  %1
kiO = 0.002; 
kdO = 0;
EO = 0;
eO_1 = 0;               % Error de orientación

% Acercamiento exponencial
v0 = 20;                % estaba en 10, Velocidad inicial
alpha = 0.95;           % 95

%% Controlador
P = 1;

while(1)
    tempBear = robotat_get_pose(Opti,PololuNumb,'eulzyx'); % Coor x y Y
    %temp2 = robotat_get_pose(Opti, 1, 'eulzyx')
    bearing = deg2rad(tempBear(4)-offsetB);                % -offset orientarlo bien para el control
%     xg = short_trac(P,1);                                        % Coordenada x
%     yg = short_trac(P,2);      
    xg = trayec(P,1);                                        % Coordenada x
    yg = trayec(P,2);      
    % Coordenada Y
                                                           % el punto cambia conforme se acerca
    x = tempBear(1);
    y = tempBear(2);    
    e = [xg-x;yg-y];                                       % Error
    thetag = atan2(e(2), e(1));                            % Angulo entre puntos deseados
    
    eP = norm(e);                                          % Dist, entre pinicial y meta
    eO = thetag - bearing; 
    eO = atan2(sin(eO), cos(eO));                          % Error orientacion
    
   if(P < length(xpath))                                   % Cada vez que la dist < 0.1, p aumenta y cambia de punto
      if(eP<0.5)
        P = P+1;
      end
    end
   
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
    posicionx = tempBear(1);
    posiciony = tempBear(2) ;
    posiciones = [posicionx,posiciony];
    posiciones_ant = posiciones;
    
    hold on;
    scatter(posiciones(:, 1), posiciones(:, 2), 10, 'filled', 'MarkerFaceAlpha', 0.5);
    xpath(end+1) = posiciones(:,1);
    ypath(end+1) = posiciones(:,2);
    xlabel('X (m)');
    ylabel('Y (m)');    
    title('Trayectoria simulada y física (2D)');
    grid on;
    axis([0, espacio, 0, espacio]);   
end

robotat_3pi_force_stop(PI3)

%% Stop de emergencia
robotat_3pi_force_stop(PI3)

%% Función para calcular las fuerzas entre partículas (utilizando el potencial de Lennard-Jones)
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


%writecell({xpath;ypath; x_path;y_path;short_trac(:,1);short_trac(:,2)}, 'AgenteCiclo8puntos_Error1.xlsx')
% Velocidad para 8 puntos = 63
% Velocidad para 5 puntos = 44
% Velocidad para 3 puntos = 25
% Velocidad para 2 puntos = 16
    
% ERROR 0.5
% Velocidad para 8 puntos = 62
% Velocidad para 5 puntos = 38
% Velocidad para 3 puntos = 19
% Velocidad para 2 puntos = 13
