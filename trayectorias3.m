clear all;
clc
%% Construccion Phantom

l = [0.137 0.105 0.105 0.110];

L0 = 0.137; L1 = 0.105; L2 = 0.105; L3 = 0.110; 
q1 = 0; q2 = 0; q3 = 0; q4 = 0;

%         dh(thetai,      di,      ai-1,  alpha-1,   sigma, offset)
dh1(1,:)= [ q1,        L0,       0,         0,         0,      0];
dh1(2,:)= [ q2,        0,        0,        pi/2,       0,      pi/2];
dh1(3,:)= [ q3,        0,        L1,        0,         0,      0];
dh1(4,:)= [ q4,        0,        L2,        0,         0,      0];

% Eslabones++
L(1) = Link(dh1(1,:), 'modified','qlim',[-1.5*pi 1.5*pi]);
L(2) = Link(dh1(2,:), 'modified','qlim',[-2/3*pi 2/3*pi]);
L(3) = Link(dh1(3,:), 'modified','qlim',[-1.5*pi 1.5*pi]);
L(4) = Link(dh1(4,:), 'modified','qlim',[-1.5*pi 1.5*pi]);

%Representación
Phantom = SerialLink(L,'name','Phantom');
Phantom.tool = transl([L3 0 0])*round(trotz(pi/2))*round(trotx(pi/2));

figure(1)
trplot(eye(4),'length',0.08,'rgb','frame','0');
Phantom.plot([0 -pi/4 -pi/2 pi/4],'scale',0.8,'jvec');
Phantom.teach
%% Puntos pick and place

entrada = [0 -0.25];
pose_entrada = [90 0 90]; % 90 90 por ubicarse en el cuarto cuadrante 

salida1 = [0.175 0.25];
pose_salida1 = [-90 atan2(salida1(2),salida1(1))*180/pi -90]; % 90 90 por ubicarse en el primer cuadrante X-Y 

salida2 = [0.09 0.25];
pose_salida2 = [-90 atan2(salida2(2),salida2(1))*180/pi -90]; % 90 90 por ubicarse en el primer cuadrante X-Y 

salida3 = [0.02 0.25];
pose_salida3 = [-90 atan2(salida3(2),salida3(1))*180/pi -90]; % 90 90 por ubicarse en el primer cuadrante X-Y 

up = 0.137;
down = 0.03; %Altura mínima

% Griper
open = 0.02;
close = 0.005;
%% Coordenadas pick and place

home = [0.25 0 0.137 -90 89.9 -90];

entrada_up = [entrada up pose_entrada];
salida1_up = [salida1 up pose_salida1];
salida2_up = [salida2 up pose_salida2];
salida3_up = [salida3 up pose_salida3];

entrada_down = [entrada down pose_entrada];
salida1_down = [salida1 down pose_salida1];
salida2_down = [salida2 down pose_salida2];
salida3_down = [salida3 down pose_salida3];
%% Matrices de transformacion homogenea
T_home = transl(home(1:3))*rpy2tr(home(4:6),'deg');
T_entrada_up = transl(entrada_up(1:3))*rpy2tr(entrada_up(4:6),'deg');
T_salida1_up = transl(salida1_up(1:3))*rpy2tr(salida1_up(4:6),'deg');
T_salida2_up = transl(salida2_up(1:3))*rpy2tr(salida2_up(4:6),'deg');
T_salida3_up = transl(salida3_up(1:3))*rpy2tr(salida3_up(4:6),'deg');

T_entrada_down = transl(entrada_down(1:3))*rpy2tr(entrada_down(4:6),'deg');
T_salida1_down = transl(salida1_down(1:3))*rpy2tr(salida1_down(4:6),'deg');
T_salida2_down = transl(salida2_down(1:3))*rpy2tr(salida2_down(4:6),'deg');
T_salida3_down = transl(salida3_down(1:3))*rpy2tr(salida3_down(4:6),'deg');
%%  Cinematica inversa
ik_home = Phantom.ikunc(T_home);
ik_entrada_up = Phantom.ikunc(T_entrada_up,ik_home);
ik_salida1_up = Phantom.ikunc(T_salida1_up,ik_entrada_up);
ik_salida2_up = Phantom.ikunc(T_salida2_up,ik_salida1_up);
ik_salida3_up = Phantom.ikunc(T_salida3_up,ik_entrada_up);

ik_entrada_down = Phantom.ikunc(T_entrada_down,ik_entrada_up);
ik_salida1_down = Phantom.ikunc(T_salida1_down,ik_salida1_up);
ik_salida2_down = Phantom.ikunc(T_salida2_down,ik_salida2_up);
ik_salida3_down = Phantom.ikunc(T_salida3_down,ik_salida3_up);
%% Trayectorias

t = (0:0.01:0.2);  
home_2_entrada_up = jtraj(ik_home,ik_entrada_up,t);
entrada_up_2_home = jtraj(ik_entrada_up,ik_home,t);

entrada_up_2_entrada_down = jtraj(ik_entrada_up,ik_entrada_down,t);
entrada_down_2_entrada_up = jtraj(ik_entrada_down,ik_entrada_up,t);

entrada_up_2_salida1_up = jtraj(ik_entrada_up,ik_salida1_up,t);
salida1_up_2_salida1_down = jtraj(ik_salida1_up,ik_salida1_down,t);
salida1_down_2_salida1_up = jtraj(ik_salida1_down,ik_salida1_up,t);
salida1_up_2_home = jtraj(ik_salida1_up,ik_home,t);

entrada_up_2_salida2_up = jtraj(ik_entrada_up,ik_salida2_up,t);
salida2_up_2_salida2_down = jtraj(ik_salida2_up,ik_salida2_down,t);
salida2_down_2_salida2_up = jtraj(ik_salida2_down,ik_salida2_up,t);
salida2_up_2_home = jtraj(ik_salida2_up,ik_home,t);

entrada_up_2_salida3_up = jtraj(ik_entrada_up,ik_salida3_up,t);
salida3_up_2_salida3_down = jtraj(ik_salida3_up,ik_salida3_down,t);
salida3_down_2_salida3_up = jtraj(ik_salida3_down,ik_salida3_up,t);
salida3_up_2_home = jtraj(ik_salida3_up,ik_home,t);

salida1_up_2_salida2_up = jtraj(ik_salida1_up,ik_salida2_up,t);
salida2_up_2_salida1_up = jtraj(ik_salida2_up,ik_salida1_up,t);
salida1_up_2_salida3_up = jtraj(ik_salida1_up,ik_salida3_up,t);
salida3_up_2_salida1_up = jtraj(ik_salida3_up,ik_salida1_up,t);

%% Animacion

% Trayectoria Salida 1
t = 0; %Tiempo de espera en la animación de cada movimiento
pause(t)
Phantom.animate(home_2_entrada_up)
pause(t)
Phantom.animate(entrada_up_2_entrada_down)
pause(t)
Phantom.animate(entrada_up_2_salida1_up)
pause(t)
Phantom.animate(salida1_up_2_salida1_down)
pause(t)
Phantom.animate(salida1_down_2_salida1_up)
pause(t)
Phantom.animate(salida1_up_2_home)

%% Trayectoria Salida 2
 
t = 0; %Tiempo de espera en la animación de cada movimiento
pause(t)
Phantom.animate(home_2_entrada_up)
pause(t)
Phantom.animate(entrada_up_2_entrada_down)
pause(t)
Phantom.animate(entrada_down_2_entrada_up)
pause(t)
Phantom.animate(entrada_up_2_salida2_up)
pause(t)
Phantom.animate(salida2_up_2_salida2_down)
pause(t)
Phantom.animate(salida2_down_2_salida2_up)
pause(t)
Phantom.animate(salida2_up_2_home)

%% Trayectoria Salida 3

t = 0; %Tiempo de espera en la animación de cada movimiento
pause(t)
Phantom.animate(home_2_entrada_up)
pause(t)
Phantom.animate(entrada_up_2_entrada_down)
pause(t)
Phantom.animate(entrada_down_2_entrada_up)
pause(t)
Phantom.animate(entrada_up_2_salida3_up)
pause(t)
Phantom.animate(salida3_up_2_salida3_down)
pause(t)
Phantom.animate(salida3_down_2_salida3_up)
pause(t)
Phantom.animate(salida3_up_2_home)


%% Conexion Matlab y Ros
%rosinit('192.168.39.147')

%% Creación publicadores

publicadores(1) = rospublisher('/Phantom_sim/joint1_position_controller/command','std_msgs/Float64');
publicadores(2) = rospublisher('/Phantom_sim/joint2_position_controller/command','std_msgs/Float64');
publicadores(3) = rospublisher('/Phantom_sim/joint3_position_controller/command','std_msgs/Float64');
publicadores(4) = rospublisher('/Phantom_sim/joint4_position_controller/command','std_msgs/Float64');
publicadores(5) = rospublisher('/Phantom_sim/joint5_position_controller/command','std_msgs/Float64');
publicadores(6) = rospublisher('/Phantom_sim/joint6_position_controller/command','std_msgs/Float64');

%% Trayectoria 1 en ROS

msg= rosmessage('std_msgs/Float64');

for a=1:20
    angulos = home_2_entrada_up(a,:);
    for i=1:4 
        msg.Data = angulos(i);
        send(publicadores(i),msg)
    end
    pause(0.2)
end

for a=1:20
    angulos = [home_2_entrada_up(20,:) open open];
    for i=1:6 
        msg.Data = angulos(i);
        send(publicadores(i),msg)
    end
    pause(0.05)
end

for a=1:20
    angulos = entrada_up_2_entrada_down(a,:);
    for i=1:4 
        msg.Data = angulos(i);
        send(publicadores(i),msg)
    end
    pause(0.2)
end

for a=1:20
    angulos = [entrada_up_2_entrada_down(20,:) close close];
    for i=1:6 
        msg.Data = angulos(i);
        send(publicadores(i),msg)
    end
    pause(0.05)
end

for a=1:20
    angulos = entrada_down_2_entrada_up(a,:);
    for i=1:4 
        msg.Data = angulos(i);
        send(publicadores(i),msg)
    end
    pause(0.2)
end

for a=1:20
    angulos = entrada_up_2_salida1_up(a,:);
    for i=1:4 
        msg.Data = angulos(i);
        send(publicadores(i),msg)
    end
    pause(0.2)
end

for a=1:20
    angulos = salida1_up_2_salida1_down(a,:);
    for i=1:4 
        msg.Data = angulos(i);
        send(publicadores(i),msg)
    end
    pause(0.2)
end

for a=1:20
    angulos = [salida1_up_2_salida1_down(20,:) open open];
    for i=1:6 
        msg.Data = angulos(i);
        send(publicadores(i),msg)
    end
    pause(0.05)
end

for a=1:20
    angulos = salida1_down_2_salida1_up(a,:);
    for i=1:4 
        msg.Data = angulos(i);
        send(publicadores(i),msg)
    end
    pause(0.2)
end

for a=1:20
    angulos = [salida1_down_2_salida1_up(20,:) close close];
    for i=1:6 
        msg.Data = angulos(i);
        send(publicadores(i),msg)
    end
    pause(0.05)
end

for a=1:20
    angulos = salida1_up_2_home(a,:);
    for i=1:4 
        msg.Data = angulos(i);
        send(publicadores(i),msg)
    end
    pause(0.2)
end


%% Trayectoria 2 en ROS
 
msg= rosmessage('std_msgs/Float64');

for a=1:20
    angulos = home_2_entrada_up(a,:);
    for i=1:4 
        msg.Data = angulos(i);
        send(publicadores(i),msg)
    end
    pause(0.2)
end

for a=1:20
    angulos = [home_2_entrada_up(20,:) open open];
    for i=1:6 
        msg.Data = angulos(i);
        send(publicadores(i),msg)
    end
    pause(0.05)
end

for a=1:20
    angulos = entrada_up_2_entrada_down(a,:);
    for i=1:4 
        msg.Data = angulos(i);
        send(publicadores(i),msg)
    end
    pause(0.2)
end

for a=1:20
    angulos = [entrada_up_2_entrada_down(20,:) close close];
    for i=1:6 
        msg.Data = angulos(i);
        send(publicadores(i),msg)
    end
    pause(0.05)
end

for a=1:20
    angulos = entrada_down_2_entrada_up(a,:);
    for i=1:4 
        msg.Data = angulos(i);
        send(publicadores(i),msg)
    end
    pause(0.2)
end

for a=1:20
    angulos = entrada_up_2_salida2_up(a,:);
    for i=1:4 
        msg.Data = angulos(i);
        send(publicadores(i),msg)
    end
    pause(0.2)
end

for a=1:20
    angulos = salida2_up_2_salida2_down(a,:);
    for i=1:4 
        msg.Data = angulos(i);
        send(publicadores(i),msg)
    end
    pause(0.2)
end

for a=1:20
    angulos = [salida2_up_2_salida2_down(20,:) open open];
    for i=1:6 
        msg.Data = angulos(i);
        send(publicadores(i),msg)
    end
    pause(0.05)
end

for a=1:20
    angulos = salida2_down_2_salida2_up(a,:);
    for i=1:4 
        msg.Data = angulos(i);
        send(publicadores(i),msg)
    end
    pause(0.2)
end

for a=1:20
    angulos = [salida2_down_2_salida2_up(20,:) close close];
    for i=1:6 
        msg.Data = angulos(i);
        send(publicadores(i),msg)
    end
    pause(0.05)
end

for a=1:20
    angulos = salida2_up_2_home(a,:);
    for i=1:4 
        msg.Data = angulos(i);
        send(publicadores(i),msg)
    end
    pause(0.2)
end

%% Trayectoria 3 en ROS
 
msg= rosmessage('std_msgs/Float64');

for a=1:20
    angulos = home_2_entrada_up(a,:);
    for i=1:4 
        msg.Data = angulos(i);
        send(publicadores(i),msg)
    end
    pause(0.2)
end

for a=1:20
    angulos = [home_2_entrada_up(20,:) open open];
    for i=1:6 
        msg.Data = angulos(i);
        send(publicadores(i),msg)
    end
    pause(0.05)
end

for a=1:20
    angulos = entrada_up_2_entrada_down(a,:);
    for i=1:4 
        msg.Data = angulos(i);
        send(publicadores(i),msg)
    end
    pause(0.2)
end

for a=1:20
    angulos = [entrada_up_2_entrada_down(20,:) close close];
    for i=1:6 
        msg.Data = angulos(i);
        send(publicadores(i),msg)
    end
    pause(0.05)
end

for a=1:20
    angulos = entrada_down_2_entrada_up(a,:);
    for i=1:4 
        msg.Data = angulos(i);
        send(publicadores(i),msg)
    end
    pause(0.2)
end

for a=1:20
    angulos = entrada_up_2_salida3_up(a,:);
    for i=1:4 
        msg.Data = angulos(i);
        send(publicadores(i),msg)
    end
    pause(0.2)
end

for a=1:20
    angulos = salida3_up_2_salida3_down(a,:);
    for i=1:4 
        msg.Data = angulos(i);
        send(publicadores(i),msg)
    end
    pause(0.2)
end

for a=1:20
    angulos = [salida3_up_2_salida3_down(20,:) open open];
    for i=1:6 
        msg.Data = angulos(i);
        send(publicadores(i),msg)
    end
    pause(0.05)
end

for a=1:20
    angulos = salida3_down_2_salida3_up(a,:);
    for i=1:4 
        msg.Data = angulos(i);
        send(publicadores(i),msg)
    end
    pause(0.2)
end

for a=1:20
    angulos = [salida3_down_2_salida3_up(20,:) close close];
    for i=1:6 
        msg.Data = angulos(i);
        send(publicadores(i),msg)
    end
    pause(0.05)
end

for a=1:20
    angulos = salida3_up_2_home(a,:);
    for i=1:4 
        msg.Data = angulos(i);
        send(publicadores(i),msg)
    end
    pause(0.2)
end

%% Trayectorias  en ROS

joy =  vrjoystick(1);

while true
    but = button(joy)
    
    if but(1) == 1
        
        msg= rosmessage('std_msgs/Float64');
        
        for a=1:20
            angulos = home_2_entrada_up(a,:);
            for i=1:4
                msg.Data = angulos(i);
                send(publicadores(i),msg)
            end
            pause(0.2)
        end
        
        for a=1:20
            angulos = [home_2_entrada_up(20,:) open open];
            for i=1:6
                msg.Data = angulos(i);
                send(publicadores(i),msg)
            end
            pause(0.05)
        end
        
        for a=1:20
            angulos = entrada_up_2_entrada_down(a,:);
            for i=1:4
                msg.Data = angulos(i);
                send(publicadores(i),msg)
            end
            pause(0.2)
        end
        
        for a=1:20
            angulos = [entrada_up_2_entrada_down(20,:) close close];
            for i=1:6
                msg.Data = angulos(i);
                send(publicadores(i),msg)
            end
            pause(0.05)
        end
        
        for a=1:20
            angulos = entrada_down_2_entrada_up(a,:);
            for i=1:4
                msg.Data = angulos(i);
                send(publicadores(i),msg)
            end
            pause(0.2)
        end
        
        for a=1:20
            angulos = entrada_up_2_salida1_up(a,:);
            for i=1:4
                msg.Data = angulos(i);
                send(publicadores(i),msg)
            end
            pause(0.2)
        end
        
        for a=1:20
            angulos = salida1_up_2_salida1_down(a,:);
            for i=1:4
                msg.Data = angulos(i);
                send(publicadores(i),msg)
            end
            pause(0.2)
        end
        
        for a=1:20
            angulos = [salida1_up_2_salida1_down(20,:) open open];
            for i=1:6
                msg.Data = angulos(i);
                send(publicadores(i),msg)
            end
            pause(0.05)
        end
        
        for a=1:20
            angulos = salida1_down_2_salida1_up(a,:);
            for i=1:4
                msg.Data = angulos(i);
                send(publicadores(i),msg)
            end
            pause(0.2)
        end
        
        for a=1:20
            angulos = [salida1_down_2_salida1_up(20,:) close close];
            for i=1:6
                msg.Data = angulos(i);
                send(publicadores(i),msg)
            end
            pause(0.05)
        end
        
        for a=1:20
            angulos = salida1_up_2_home(a,:);
            for i=1:4
                msg.Data = angulos(i);
                send(publicadores(i),msg)
            end
            pause(0.2)
        end
    end
    if but(2) == 1
        
        msg= rosmessage('std_msgs/Float64');
        
        for a=1:20
            angulos = home_2_entrada_up(a,:);
            for i=1:4
                msg.Data = angulos(i);
                send(publicadores(i),msg)
            end
            pause(0.2)
        end
        
        for a=1:20
            angulos = [home_2_entrada_up(20,:) open open];
            for i=1:6
                msg.Data = angulos(i);
                send(publicadores(i),msg)
            end
            pause(0.05)
        end
        
        for a=1:20
            angulos = entrada_up_2_entrada_down(a,:);
            for i=1:4
                msg.Data = angulos(i);
                send(publicadores(i),msg)
            end
            pause(0.2)
        end
        
        for a=1:20
            angulos = [entrada_up_2_entrada_down(20,:) close close];
            for i=1:6
                msg.Data = angulos(i);
                send(publicadores(i),msg)
            end
            pause(0.05)
        end
        
        for a=1:20
            angulos = entrada_down_2_entrada_up(a,:);
            for i=1:4
                msg.Data = angulos(i);
                send(publicadores(i),msg)
            end
            pause(0.2)
        end
        
        for a=1:20
            angulos = entrada_up_2_salida2_up(a,:);
            for i=1:4
                msg.Data = angulos(i);
                send(publicadores(i),msg)
            end
            pause(0.2)
        end
        
        for a=1:20
            angulos = salida2_up_2_salida2_down(a,:);
            for i=1:4
                msg.Data = angulos(i);
                send(publicadores(i),msg)
            end
            pause(0.2)
        end
        
        for a=1:20
            angulos = [salida2_up_2_salida2_down(20,:) open open];
            for i=1:6
                msg.Data = angulos(i);
                send(publicadores(i),msg)
            end
            pause(0.05)
        end
        
        for a=1:20
            angulos = salida2_down_2_salida2_up(a,:);
            for i=1:4
                msg.Data = angulos(i);
                send(publicadores(i),msg)
            end
            pause(0.2)
        end
        
        for a=1:20
            angulos = [salida2_down_2_salida2_up(20,:) close close];
            for i=1:6
                msg.Data = angulos(i);
                send(publicadores(i),msg)
            end
            pause(0.05)
        end
        
        for a=1:20
            angulos = salida2_up_2_home(a,:);
            for i=1:4
                msg.Data = angulos(i);
                send(publicadores(i),msg)
            end
            pause(0.2)
        end
        
        
    end
    if but(3) == 1
        
        msg= rosmessage('std_msgs/Float64');
        
        for a=1:20
            angulos = home_2_entrada_up(a,:);
            for i=1:4
                msg.Data = angulos(i);
                send(publicadores(i),msg)
            end
            pause(0.2)
        end
        
        for a=1:20
            angulos = [home_2_entrada_up(20,:) open open];
            for i=1:6
                msg.Data = angulos(i);
                send(publicadores(i),msg)
            end
            pause(0.05)
        end
        
        for a=1:20
            angulos = entrada_up_2_entrada_down(a,:);
            for i=1:4
                msg.Data = angulos(i);
                send(publicadores(i),msg)
            end
            pause(0.2)
        end
        
        for a=1:20
            angulos = [entrada_up_2_entrada_down(20,:) close close];
            for i=1:6
                msg.Data = angulos(i);
                send(publicadores(i),msg)
            end
            pause(0.05)
        end
        
        for a=1:20
            angulos = entrada_down_2_entrada_up(a,:);
            for i=1:4
                msg.Data = angulos(i);
                send(publicadores(i),msg)
            end
            pause(0.2)
        end
        
        for a=1:20
            angulos = entrada_up_2_salida3_up(a,:);
            for i=1:4
                msg.Data = angulos(i);
                send(publicadores(i),msg)
            end
            pause(0.2)
        end
        
        for a=1:20
            angulos = salida3_up_2_salida3_down(a,:);
            for i=1:4
                msg.Data = angulos(i);
                send(publicadores(i),msg)
            end
            pause(0.2)
        end
        
        for a=1:20
            angulos = [salida3_up_2_salida3_down(20,:) open open];
            for i=1:6
                msg.Data = angulos(i);
                send(publicadores(i),msg)
            end
            pause(0.05)
        end
        
        for a=1:20
            angulos = salida3_down_2_salida3_up(a,:);
            for i=1:4
                msg.Data = angulos(i);
                send(publicadores(i),msg)
            end
            pause(0.2)
        end
        
        for a=1:20
            angulos = [salida3_down_2_salida3_up(20,:) close close];
            for i=1:6
                msg.Data = angulos(i);
                send(publicadores(i),msg)
            end
            pause(0.05)
        end
        
        for a=1:20
            angulos = salida3_up_2_home(a,:);
            for i=1:4
                msg.Data = angulos(i);
                send(publicadores(i),msg)
            end
            pause(0.2)
        end
        
    end
    if but(5) == 1
        break
    end
end


%% Desconexión
 rosshutdown
 
 
 

 
 
