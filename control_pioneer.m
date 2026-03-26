    clear all
    close all
    clc
    %% Inicio de conexion
    sim=remApi('remoteApi'); % usando el prototipo de función (remoteApiProto.m)
    sim.simxFinish(-1); % Cerrar las conexiones anteriores en caso de que exista una
    clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);
    %% Verificación de la conexión
    if (clientID>-1)
        disp('Conexión con Coppelia iniciada');
        %% Codigo de control
       
        % Preparación
        L = 0.365; %distancia entre ruedas
        r = 0.1; %radio llanta
        %estados iniciales
        x=0;
        y=0;
        theta = 0;
        %definición de ganacias (translacion y rotación repectivamente)
        kpt = 1.3;
        kpr = 5;
        i = 1;
        %velocidades de saturación
        wmax = 10;
        vmax = 0.3;
        
        %Instancias de objetos primitivos de coppelia a objetos de Matlab
        %Para referenciar el objeto y sus atributos (modo bloqueantes para
        %que se ejecute el código y luego se refleje el cambio en coppelia)
        [returnCode, left_motor]=sim.simxGetObjectHandle(clientID, ...
        'Pioneer_p3dx_leftMotor',sim.simx_opmode_blocking);%Motor izq
    
        [returnCode, right_motor]=sim.simxGetObjectHandle(clientID, ...
        'Pioneer_p3dx_rightMotor',sim.simx_opmode_blocking);%Motor der
    
        % Discos para referenciar posiciones deseadas
        [~, target1]=sim.simxGetObjectHandle(clientID, ...
        'Disc',sim.simx_opmode_blocking);
    
        [~, target2]=sim.simxGetObjectHandle(clientID, ...
        'Disc0',sim.simx_opmode_blocking);
    
        [~, target3]=sim.simxGetObjectHandle(clientID, ...
        'Disc1',sim.simx_opmode_blocking);
    
        %Instancia del AGV
        [returnCode, pioneer_act]=sim.simxGetObjectHandle(clientID, ...
        'Pioneer_p3dx',sim.simx_opmode_blocking);
    
        
    
    
        % Acciones
        %seteo de velocidades
        [returnCode] = sim.simxSetJointTargetVelocity(clientID,left_motor, 0, ...
        sim.simx_opmode_blocking);
        [returnCode] = sim.simxSetJointTargetVelocity(clientID,right_motor, 0, ...
        sim.simx_opmode_blocking);
    
    
        % Solicitar el envío continuo de los datos de posición y
        % orientación
        [~, pos_act]=sim. simxGetObjectPosition(clientID, pioneer_act,-1, ...
        sim.simx_opmode_streaming);%posición del agv 
        %posición de los objetivos
        [~, target1_pos]=sim.simxGetObjectPosition(clientID, target1, -1, ...
        sim.simx_opmode_streaming);
        [~, target2_pos]=sim.simxGetObjectPosition(clientID, target2, -1, ...
        sim.simx_opmode_streaming);
        [~, target3_pos]=sim.simxGetObjectPosition(clientID, target3, -1, ...
        sim.simx_opmode_streaming);
    
        %orientación del AGV
        [~, ori_act]=sim.simxGetObjectOrientation(clientID, pioneer_act, -1, ...
        sim.simx_opmode_streaming);
        
        for j = 1:100
            %Obtención de la posición y orientación actual del buffer del flujo
            %de datos
            [~, pos_act]=sim.simxGetObjectPosition(clientID, pioneer_act, -1, ...
            sim.simx_opmode_buffer);
            [~, target1_pos]=sim. simxGetObjectPosition(clientID, target1, -1, ...
            sim.simx_opmode_buffer);
            [~, target2_pos]=sim. simxGetObjectPosition(clientID, target2, -1, ...
            sim.simx_opmode_buffer);
            [~, target3_pos]=sim. simxGetObjectPosition(clientID, target3, -1, ...
            sim.simx_opmode_buffer);
            [~, ori_act]=sim.simxGetObjectOrientation(clientID, pioneer_act, -1, ...
            sim.simx_opmode_buffer);
            
            %definir vector de posiciones deseadas
            posd=[target1_pos(1) target1_pos(2); target2_pos(1) target2_pos(2); target3_pos(1) target3_pos(2)];
            
            %definir estados del robot en base a los datos de coppelia
            x=pos_act(1);
            y=pos_act(2);
            theta = ori_act(3);
            %ley de control
            d=sqrt(((posd(i,1)-x)^2)+((posd(i,2)-y)^2));
            thetad=atan2((posd(i,2)-y),(posd(i,1)-x));
            thetae=theta-thetad;
            %Saturar el error de -pi a 2pi
            if thetae>pi
                thetae = thetae-pi*2;
            elseif thetae<-pi
                thetae = thetae+pi*2;
            end
            
            % Definición de nueva posición cuando el error de posición
            %queda definido dentro de un círculo de 15cm de radio
            if abs(d)<0.15 && j>2
                if i == 3 %cuando llega al último punto, para los motores
                    disp("termina");
                    [returnCode] = sim.simxSetJointTargetVelocity(clientID,left_motor, 0, ...
                    sim.simx_opmode_blocking);
            
                    [returnCode] = sim.simxSetJointTargetVelocity(clientID,right_motor, 0, ...
                    sim.simx_opmode_blocking);
                    break;
                end
                i=i+1;
            end
            
            %control propuesto
            V = vmax*tanh(kpt*d/vmax);
            w = -kpr*tanh(thetae/wmax);
        
            %vel ang de ruedas
            wl = (V-L*w/2)/r;
            wr = (V+L*w/2)/r;
            
            %envíar las velocidades
            [returnCode] = sim.simxSetJointTargetVelocity(clientID,left_motor, wl, ...
            sim.simx_opmode_blocking);
    
            [returnCode] = sim.simxSetJointTargetVelocity(clientID,right_motor, wr, ...
            sim.simx_opmode_blocking);
    
            pause(0.1)%delay de 10ms
        end
        
        %termina la conexión al terminar la simulación
        [returnCode] = sim.simxSetJointTargetVelocity(clientID,left_motor, 0, ...
        sim.simx_opmode_blocking);
        [returnCode] = sim.simxSetJointTargetVelocity(clientID,right_motor, 0, ...
        sim.simx_opmode_blocking);
        disp('Conexion con Coppelia Terminada');
        sim.simxFinish(clientID);
    end
    
    sim.delete(); % Llamar al destructor!

