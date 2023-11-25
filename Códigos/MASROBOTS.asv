%% Dos pololus o mas
%% Limpiar todo
clear all;
clc;

%% POLOLU Conexión con el servidor del Robotat y Pololu
% Primer pololu
PololuNumb = 2;                            % # de Pololu a utilizar
ObjNumb = 2;                               % # de Marker colocado en el Pololu
Opti = robotat_connect();                  % Conectar con el robotat
PI31 = robotat_3pi_connect(PololuNumb);    % Conectarse al pololu

PololuNumb2 = 3;                           % # de Pololu a utilizar
ObjNumb2 = 3;                              % # de Marker colocado en el Pololu
PI32 = robotat_3pi_connect(PololuNumb2);   % Conectarse al pololu

% PololuNumb3 = 3;                         % # de Pololu a utilizar
% ObjNumb3 = 3;                            % # de Marker colocado en el Pololu
% PI33 = robotat_3pi_connect(PololuNumb3); % Conectarse al pololu 
 
%% Desconexion del agente
robotat_3pi_disconnect(PI31)                % Desconectarme del robot
robotat_3pi_disconnect(PI32)                % Desconectarme del robot
%robotat_3pi_disconnect(PI33)               % Desconectarme del robot

%% Probar si el Pololu está conectado
robotat_3pi_set_wheel_velocities(PI31,-50,50)% Girar sobre sí mismo
robotat_3pi_set_wheel_velocities(PI32,-50,50)% Girar sobre sí mismo
%robotat_3pi_set_wheel_velocities(PI33,-50,50)% Girar sobre sí mismo

%% Ir en linea recta
robotat_3pi_set_wheel_velocities(PI31,50,50) % Avanzar 
robotat_3pi_set_wheel_velocities(PI32,50,50) % Avanzar 
%robotat_3pi_set_wheel_velocities(PI33,50,50) % Avanzar 

%% Stop de emergencia
robotat_3pi_force_stop(PI31)                  
robotat_3pi_force_stop(PI32)     
%robotat_3pi_force_stop(PI33) 

%% Variables del Pololu
r = 32/(2*1000);                            % Radio de las ruedas en metros
l = 96/(2*1000);                            % Distancia entre ruedas en metros

%% Ángulo de bearing
% Se encuentra el offset en base a la posición del marcador (angulo de rotación)
clc;
% Robot 2
tempBear = robotat_get_pose(Opti,PololuNumb,'eulzyx')
offsetB = -139.3762;                         % Calibrar orientación pololu
bearing = tempBear(4)-offsetB                % ángulo, lo mas cercano a 0

% Robot 3
tempBear2 = robotat_get_pose(Opti,PololuNumb2,'eulzyx')
offsetB2 =  177.3168;                          % Calibrar orientación pololu
bearing2 = tempBear2(4)-offsetB2               % ángulo, lo mas cercano a 0

%% Variables de control del sistema
%          PID orientación
kpO = 12;  %1
kiO = 0.002; 
kdO = 0;
EO = 0;
eO_1 = 0;               % Error de orientación
EO2 = 0;
eO_12 = 0;  

% Acercamiento exponencial
v0 = 12;                % estaba en 10, Velocidad inicial
alpha = 0.88;           % 95

%% Parámetros del potencial de Lennard-Jones
epsilon = 1.0;          
sigma = 1.0;
num_particulas = 2;
tiempo_simulacion = 6   ;  % Tiempo de simulación en segundos (30 segundos en este ejemplo)
paso_tiempo = 0.01;      % Tamaño del paso de tiempo en segundos
masa_particula = 1.0;   % Masa de cada partícula
espacio = 1.5;          % Tamaño del espacio a utilizar

% Bucle de simulación para calcular trayectorias
num_pasos = tiempo_simulacion / paso_tiempo;


%% Trayectoria con dinamica molecular
%posiciones = zeros(1,2);
%posiciones = [1,1] 
% num_particulas_por_lado = round(num_particulas^(1/2)); % Raíz cuadrada del número de partículas por lado
% espaciado_entre_particulas = espacio / num_particulas_por_lado;
% posiciones = rand(num_particulas, 2)*espaciado_entre_particulas;
%posiciones = rand(num_particulas, 2)*espacio;
posiciones = [0,0;0.2,1.1]
posiciones0 = posiciones;z
velocidades = [2.5,1]; % Velocidades iniciales aleatorias en 2D
aceleraciones = zeros(num_particulas, 2); % Inicializar aceleraciones a cero en 2D
posiciones_ant = posiciones;
x_path = [];
y_path = [];

for paso = 1:100
    
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

    % Actualizar posiciones y aceleraciones para el siguiente paso de tiempo
    posiciones_ant = posiciones;
    
    % Almacenar las coordenadas de posición de cada partícula en este paso de tiempo
    x_path(:, paso) = posiciones(:, 1);
    y_path(:, paso) = posiciones(:, 2);
    
    % Dibujar las partículas en la figura actualizada
    scatter(posiciones(:, 1), posiciones(:, 2), 10, 'filled', 'MarkerFaceAlpha', 0.5);
    xlabel('X (m)');
    ylabel('Y (m)');    
    title('Simulación de Trayectorias de Partículas (2D)');
    grid on;
    axis([0, espacio, 0, espacio]);
    
    % Pausa para permitir que MATLAB actualice la figura
    pause(paso_tiempo);
end

% Trazar las trayectorias de todas las partículas en 2D
hold on;
for i = 1:num_particulas
    plot(posiciones0(i, 1), posiciones0(i, 2),'o', 'MarkerEdgeColor', 'black', MarkerSize = 4); % Posicion inicial
    plot(x_path(i, 2:end-1), y_path(i, 2:end-1));
    plot(posiciones(i, 1), posiciones(i, 2),'pentagram', 'MarkerEdgeColor', 'black', MarkerSize = 4);
end
hold off;

%% Graficar la trayectoria
hold on;
plot(x_path(1, :), y_path(1, :));    % Trayectoria 1

plot(x_path(2, :), y_path(2, :));    % Trayectoria 2
pbaspect([1 1 1])
hold off;



%x_path = spline(x_path,y_path)
%% Trayectoria para probar mapas
xpath = [];
ypath = [];
xpath2 = [];
ypath2 = [];

%% Crear un gif
% % Propiedades del archivo .GIF que se va a generar
% % Nombre
% filename = 'DMolecularFisico.gif';
% % Tiempo en segundos que dura cada plot en el GIF
% DelayTime = 0.1; 
% f = figure(2)                         % Crear una figura

%% Colocarlo en la posicion inicial
v0 = 8;
tempBear = robotat_get_pose(Opti,PololuNumb,'eulzyx'); % Coor x y Y
x = tempBear(1);
y = tempBear(2);  
xg = posiciones0(1);                                       % Coordenada x
yg = posiciones0(1,end);    
e = [xg-x;yg-y];
eP = norm(e);  

while(eP > 0.1) % ep>1 && P<NUMERO DE LA TRAYECTORIAS
    tempBear = robotat_get_pose(Opti,PololuNumb,'eulzyx'); % Coor x y Y
    bearing = deg2rad(tempBear(4)-offsetB);                % -offset orientarlo bien para el control

    
    xg = posiciones0(1);                                   % Coordenada x
    yg = posiciones0(1,end);                                
    
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

    robotat_3pi_set_wheel_velocities(PI31,v_left_wheel,v_rigth_wheel); 
    posicionx = tempBear(1);
    posiciony = tempBear(2); 
    posiciones = [posicionx,posiciony];
    posiciones_ant = posiciones;
end

robotat_3pi_force_stop(PI31)

kpO = 12;  %1
kiO = 0.002; 
kdO = 0;
EO = 0;
eO_1 = 0;               % Error de orientación

% Acercamiento exponencial
v0 = 8;                % estaba en 10, Velocidad inicial
alpha = 0.95;           % 95

EO2 = 0;
eO_12 = 0;               % Error de orientación

%% Colocarlo en la posicion inicial
v0 = 5;
tempBear2 = robotat_get_pose(Opti,PololuNumb2,'eulzyx'); % Coor x y Y
x2 = tempBear2(1);
y2 = tempBear2(2);  
xg2 = posiciones0(2);                                       % Coordenada x
yg2 = posiciones0(2,end);    
e2 = [xg2-x2;yg2-y2];
eP2 = norm(e2);  

while(eP2 > 0.01)
    tempBear2 = robotat_get_pose(Opti,PololuNumb2,'eulzyx'); % Coor x y Y
    bearing2 = deg2rad(tempBear2(4)-offsetB2);                % -offset orientarlo bien para el control

    xg2 = posiciones0(2);                                   % Coordenada x
    yg2 = posiciones0(2,end);                                
    
    % Coordenada Y                                                         % el punto cambia conforme se acerca
    x2 = tempBear2(1);
    y2 = tempBear2(2);    
    e2 = [xg2-x2;yg2-y2];                                       % Error
    thetag2 = atan2(e2(2), e2(1));                            % Angulo entre puntos deseados
    
    eP2 = norm(e2);                                          % Dist, entre pinicial y meta
    eO2 = thetag2 - bearing2; 
    eO2 = atan2(sin(eO2), cos(eO2));                          % Error orientacion
    
    % Control de velocidad lineal exponencial
    kP2 = v0 * (1-exp(-alpha*eP2^2)) / eP2;
    v2 = kP2*eP2;
    
    % Control de velocidad angular
    eO_D2 = eO2 - eO_12;
    EO2 = EO2 + eO2;
    w2 = kpO*eO2 + kiO*EO2 + kdO*eO_D2;
    eO_12 = eO2;
    
    % Se combinan los controladores
    u2 = [v2; w2];
    v_rigth_wheel2 = (u2(1) + u2(2) *l)/r
    v_left_wheel2 = (u2(1) - u2(2) *l)/r 
    
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

    robotat_3pi_set_wheel_velocities(PI32,v_left_wheel2,v_rigth_wheel2); 
    posicionx2 = tempBear2(1);
    posiciony2 = tempBear2(2); 
    posiciones2 = [posicionx2,posiciony2];
    posiciones_ant2 = posiciones2;
end

robotat_3pi_force_stop(PI32)

kpO = 12;  %1
kiO = 0.002; 
kdO = 0;
EO = 0;
eO_1 = 0;               % Error de orientación
EO2 = 0;
eO_12 = 0; 
% Acercamiento exponencial
v0 = 8;                % estaba en 10, Velocidad inicial
alpha = 0.95;        

%% Controlador
P = 1;
P2 = 1;
while(1)
    tempBear = robotat_get_pose(Opti,PololuNumb,'eulzyx'); % Coor x y Y
    bearing = deg2rad(tempBear(4)-offsetB);                % -offset orientarlo bien para el control
    
    tempBear2 = robotat_get_pose(Opti,PololuNumb2,'eulzyx'); % Coor x y Y
    bearing2 = deg2rad(tempBear2(4)-offsetB2);                % -offset orientarlo bien para el control

    xg = x_path(1, P);                                        % Coordenada x
    yg = y_path(1, P);   
    xg2 = x_path(2, P2);                                        % Coordenada x
    yg2 = y_path(2, P2);                                
                                                           % el punto cambia conforme se acerca
    x = tempBear(1);
    y = tempBear(2);    
    e = [xg-x;yg-y];                                       % Error
    thetag = atan2(e(2), e(1));                            % Angulo entre puntos deseados
    x2 = tempBear2(1);
    y2 = tempBear2(2);    
    e2 = [xg2-x2;yg2-y2];                                       % Error
    thetag2 = atan2(e2(2), e2(1));                            % Angulo entre puntos deseados
    
    eP = norm(e);                                          % Dist, entre pinicial y meta
    eO = thetag - bearing; 
    eO = atan2(sin(eO), cos(eO));                          % Error orientacion
    eP2 = norm(e2);                                          % Dist, entre pinicial y meta
    eO2 = thetag2 - bearing2; 
    eO2 = atan2(sin(eO2), cos(eO2));                          % Error orientacion
    
    if(P < length(xpath))                                   % Cada vez que la dist < 0.1, p aumenta y cambia de punto
      if(eP<0.2)
        P = P+1;
      end
    end

    if(P2 < length(xpath2))                                   % Cada vez que la dist < 0.1, p aumenta y cambia de punto
      if(eP2<0.2)
        P2 = P2+1;
      end
    end
        
    % Control de velocidad lineal exponencial
    kP = v0 * (1-exp(-alpha*eP^2)) / eP;
    v = kP*eP;
    % ROBOT2
    kP2 = v0 * (1-exp(-alpha*eP2^2)) / eP2;
    v2 = kP*eP2;
    
    % Control de velocidad angular
    eO_D = eO - eO_1;
    EO = EO + eO;
    w = kpO*eO + kiO*EO + kdO*eO_D;
    eO_1 = eO;
    % ROBOT2
    eO_D2 = eO2 - eO_12;
    EO2 = EO2 + eO2;
    w2 = kpO*eO2 + kiO*EO2 + kdO*eO_D2;
    eO_12 = eO2;
    
    % Se combinan los controladores
    u = [v; w];
    v_rigth_wheel = (u(1) + u(2) *l)/r 
    v_left_wheel = (u(1) - u(2) *l)/r 
    % RObot 2
    u2 = [v2; w2];
    v_rigth_wheel2 = (u2(1) + u2(2) *l)/r 
    v_left_wheel2 = (u2(1) - u2(2) *l)/r 

    robotat_3pi_set_wheel_velocities(PI31,v_left_wheel,v_rigth_wheel); 
    posicionx = tempBear(1)
    posiciony = tempBear(2) 
    posiciones = [posicionx,posiciony]
    posiciones_ant = posiciones;
    robotat_3pi_set_wheel_velocities(PI32,v_left_wheel2,v_rigth_wheel2); 
    posicionx2 = tempBear2(1)
    posiciony2 = tempBear2(2) 
    posiciones2 = [posicionx2,posiciony2]
    posiciones_ant2 = posiciones2;
    
    hold on;
    scatter(posiciones(:, 1), posiciones(:, 2), 10, 'filled', 'MarkerFaceAlpha', 0.5);
    scatter(posiciones2(:, 1), posiciones2(:, 2), 10, 'filled', 'MarkerFaceAlpha', 0.5);
    %scatter(posiciones2(:, 1), posiciones2(:, 2), 10, 'filled', 'MarkerFaceAlpha', 0.5);
    
    xpath(end+1) = posiciones(:,1);
    ypath(end+1) = posiciones(:,2);
    xpath2(end+1) = posiciones2(:,1);
    ypath2(end+1) = posiciones2(:,2);
%     xpath2(end+1) = posiciones2(:,1);
%     ypath2(end+1) = posiciones2(:,2);
    xlabel('X (m)');
    ylabel('Y (m)');    
    title('Trayectoria simulada y física (2D)');
    grid on;
    axis([0, espacio, 0, espacio]); 
    hold on;
end


% %% Controlador
% P2 = 1;
% while(1)
%     tempBear2 = robotat_get_pose(Opti,PololuNumb2,'eulzyx'); % Coor x y Y
%     %temp2 = robotat_get_pose(Opti, 1, 'eulzyx')
%     bearing2 = deg2rad(tempBear2(4)-offsetB2);                % -offset orientarlo bien para el control
% 
%     xg2 = x_path(2, P2);                                        % Coordenada x
%     yg2 = y_path(2, P2);                                
%     % Coordenada Y
%                                                            % el punto cambia conforme se acerca
%     x2 = tempBear2(1);
%     y2 = tempBear2(2);    
%     e2 = [xg2-x2;yg2-y2];                                       % Error
%     thetag2 = atan2(e2(2), e2(1));                            % Angulo entre puntos deseados
%     
%     eP2 = norm(e2);                                          % Dist, entre pinicial y meta
%     eO2 = thetag2 - bearing2; 
%     eO2 = atan2(sin(eO2), cos(eO2));                          % Error orientacion
%     
%    if(P2 < length(xpath2))                                   % Cada vez que la dist < 0.1, p aumenta y cambia de punto
%       if(eP2<0.2)
%         P2 = P2+1;
%       end
%     end
%    
%     % Control de velocidad lineal exponencial
%     kP2 = v0 * (1-exp(-alpha*eP2^2)) / eP2;
%     v2 = kP*eP2;
%     
%     % Control de velocidad angular
%     eO_D2 = eO2 - eO_12;
%     EO2 = EO2 + eO2;
%     w2 = kpO*eO2 + kiO*EO2 + kdO*eO_D2;
%     eO_12 = eO2;
%     
%     % Se combinan los controladores
%     u2 = [v2; w2];
%     v_rigth_wheel2 = (u2(1) + u2(2) *l)/r 
%     v_left_wheel2 = (u2(1) - u2(2) *l)/r 
% 
%     robotat_3pi_set_wheel_velocities(PI32,v_left_wheel2,v_rigth_wheel2); 
%     posicionx2 = tempBear2(1)
%     posiciony2 = tempBear2(2) 
%     posiciones2 = [posicionx2,posiciony2]
%     posiciones_ant2 = posiciones2;
%     
%     hold on;
%     scatter(posiciones2(:, 1), posiciones2(:, 2), 10, 'filled', 'MarkerFaceAlpha', 0.5);
%     %scatter(posiciones2(:, 1), posiciones2(:, 2), 10, 'filled', 'MarkerFaceAlpha', 0.5);
%     
%     xpath2(end+1) = posiciones2(:,1);
%     ypath2(end+1) = posiciones2(:,2);
% %     xpath2(end+1) = posiciones2(:,1);
% %     ypath2(end+1) = posiciones2(:,2);
%     xlabel('X (m)');
%     ylabel('Y (m)');    
%     title('Trayectoria simulada y física (2D)');
%     grid on;
%     axis([0, espacio, 0, espacio]); 
%       
% end

%% Stop de emergencia
robotat_3pi_force_stop(PI31)
robotat_3pi_force_stop(PI32)


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