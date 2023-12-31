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

PololuNumb2 = 3;                            % # de Pololu a utilizar
ObjNumb2 = 3;                               % # de Marker colocado en el Pololu
PI32 = robotat_3pi_connect(PololuNumb2);     % Conectarse al pololu

% PololuNumb3 = 3;                            % # de Pololu a utilizar
% ObjNumb3 = 3;                               % # de Marker colocado en el Pololu
% PI33 = robotat_3pi_connect(PololuNumb3);     % Conectarse al pololu
% 


%% Desconexion del agente
robotat_3pi_disconnect(PI31)                % Desconectarme del robot
robotat_3pi_disconnect(PI32)                % Desconectarme del robot
%robotat_3pi_disconnect(PI33)                % Desconectarme del robot

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
tempBear = robotat_get_pose(Opti,PololuNumb,'eulzyx')
offsetB = -139.3762;                         % Calibrar orientación pololu
bearing = tempBear(4)-offsetB               % ángulo, lo mas cercano a 0

tempBear2 = robotat_get_pose(Opti,PololuNumb2,'eulzyx')
offsetB2 =  177.3168;                         % Calibrar orientación pololu
bearing2 = tempBear2(4)-offsetB2               % ángulo, lo mas cercano a 0

%% Variables de control del sistema
%          PID orientación
kpO = 12;  %1
kiO = 0.002; 
kdO = 0;
EO = 0;
eO_1 = 0;               % Error de orientación

% Acercamiento exponencial
v0 = 12;                % estaba en 10, Velocidad inicial
alpha = 0.88;           % 95

%% Parámetros del potencial de Lennard-Jones
epsilon = 1.0;          
sigma = 1.0;
num_particulas = 2;
tiempo_simulacion = 7   ;  % Tiempo de simulación en segundos (30 segundos en este ejemplo)
paso_tiempo = 0.1;      % Tamaño del paso de tiempo en segundos
masa_particula = 1.0;   % Masa de cada partícula
espacio = 1.5;          % Tamaño del espacio a utilizar

% Bucle de simulación para calcular trayectorias
num_pasos = tiempo_simulacion / paso_tiempo;


%% Trayectoria con dinamica molecular
%posiciones = zeros(1,2);
%posiciones = [1,1] 
num_particulas_por_lado = round(num_particulas^(1/2)); % Raíz cuadrada del número de partículas por lado
espaciado_entre_particulas = espacio / num_particulas_por_lado;
posiciones = rand(num_particulas, 2)*espaciado_entre_particulas;
%posiciones = rand(num_particulas, 2)*espacio;
posiciones0 = posiciones;
velocidades = [2,1]; % Velocidades iniciales aleatorias en 2D
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
    axis([-2, espacio, -2, espacio]);
    
    % Pausa para permitir que MATLAB actualice la figura
    pause(paso_tiempo);
end

% Trazar las trayectorias de todas las partículas en 2D
hold on;
for i = 1:num_particulas
    plot(posiciones_ant(i, 1), posiciones_ant(i, 2),'o', 'MarkerEdgeColor', 'black', MarkerSize = 4); % Posicion inicial
    plot(x_path(i, 2:end-1), y_path(i, 2:end-1));
    plot(posiciones(i, 1), posiciones(i, 2),'pentagram', 'MarkerEdgeColor', 'black', MarkerSize = 4);
end
hold off;


%% Graficar la trayectoria
plot(x_path,y_path)

%x_path = spline(x_path,y_path)
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
tempBear = robotat_get_pose(Opti,PololuNumb,'eulzyx'); % Coor x y Y
x = tempBear(1);
y = tempBear(2);  
xg = posiciones0(1);                                   % Coordenada x
yg = posiciones0(2);    
e = [xg-x;yg-y];
eP = norm(e);  

while(eP > 0.1)
    tempBear = robotat_get_pose(Opti,PololuNumb,'eulzyx'); % Coor x y Y
    bearing = deg2rad(tempBear(4)-offsetB);                % -offset orientarlo bien para el control

    xg = posiciones0(1);                                   % Coordenada x
    yg = posiciones0(2);                                
    % Coordenada Y
                                                           % el punto cambia conforme se acerca
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

    robotat_3pi_set_wheel_velocities(PI3,v_left_wheel,v_rigth_wheel); 
    posicionx = tempBear(1)
    posiciony = tempBear(2) 
    posiciones = [posicionx,posiciony]
    posiciones_ant = posiciones;
end

robotat_3pi_force_stop(PI3)

%% Controlador
P = 1;

while(1)
    tempBear = robotat_get_pose(Opti,PololuNumb,'eulzyx'); % Coor x y Y
    temp2 = robotat_get_pose(Opti, 1, 'eulzyx')
    bearing = deg2rad(tempBear(4)-offsetB);                % -offset orientarlo bien para el control

    xg = x_path(P);                                        % Coordenada x
    yg = y_path(P);                                
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
      if(eP<0.2)
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
    posicionx = tempBear(1)
    posiciony = tempBear(2) 
    posiciones = [posicionx,posiciony]
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
    
    
%     frame = getframe(f);
%     % Convertir la trama en una imagen RGB (3 dimensiones) 
%     im = frame2im(frame);
%     
%      % Transformar muestras de RGB a 1 dimensión con un mapa de color "cm" 
%     [imind,cm] = rgb2ind(im , 256); 
%     if paso == 1;
%         %Crear el archivo GIF
%         imwrite(imind,cm,filename,'gif','DelayTime', DelayTime , 'LoopCount' , Inf  );
%     else
%           % Añadir al GIF cada nuevo plot
%         imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime', DelayTime );
%     end
%     
end

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