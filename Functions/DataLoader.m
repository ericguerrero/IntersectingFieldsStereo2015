% Convert .pgm images
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% -las imágenes están en formato .pgm, Matlab no debería tener ningún
% problema 

% -la cámara 01 suele estar a la derecha y es la que normalmente lleva la
% IMU incorporada, en este experimento no hay IMU, asi que tampoco importa

% -cada imagen lleva en el nombre un timestamp de segundos desde epoch con
% resolución hasta microsegundo 

% -el fichero camarasquietasconmarker tiene listadas todas las imagenes,
% con su timesptamp y la camara a la que corresponde, se genera para
% comprobar más que nada  

% -cam01AR contiene (con timestamps) la transformacion de la camara 01
% hacia el tag AR(el cuadrado en la hoja de papel), y lo mismo con cam02AR
% y la otra cámara. El formato de estos ficheros es:  
% timestamp(segundos) timestamp(microseg) posicion_x(m) posicion_y(m)
% posicion_z(m) quaternion_x quaternion_y quaternion_z quaternion_w  
% 
% -con estas dos transformaciones se deberia poder estimar la pose entre
% cámaras. Sigue en principio las convenciones de ROS. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [t1, t2,tr_1, tr_2, tr_marker]=DataLoader(Secuencia)

load([Secuencia '/data.mat']);
%% TIME (s)
t1=round([cam01AR(:,1), cam01AR(:,2)*10^-3]);
t2=round([cam02AR(:,1), cam02AR(:,2)*10^-3]);


%% POSITION 
t_1=cam01AR(:,3:5);
t_2=cam02AR(:,3:5);

%% ORIENTATION
q1=cam01AR(:,6:9);
q2=cam02AR(:,6:9);
% Quaternion ROS to MATLAB
q1=[q1(:,4) q1(:,1:3)];
q2=[q2(:,4) q2(:,1:3)];
% Quaternion to rotation
len=length(q1);
r_1=zeros(4,4,len);
r_2=zeros(4,4,len);
for i=1:len
    r_1(:,:,i) = q2tr(q1(i,:));
    r_2(:,:,i) = q2tr(q2(i,:));
end

%% HT
% tr_1=zeros(4,4,len);
% tr_2=zeros(4,4,len);
% tr_marker = transl(0,0,0);
% for i=1:len
%     tr_1(:,:,i) = tr_marker/(transl(t_1(i,:))*r_1(:,:,i));
%     tr_2(:,:,i) = tr_marker/(transl(t_2(i,:))*r_2(:,:,i)); 
% end

% 
% tr_1=transl(0,0,0);
% tr_2=zeros(4,4,len);
% tr_marker = zeros(4,4,len);
% for i=1:len
%     tr_marker(:,:,i) = [r_1(1:3,1:3,i)', -r_1(1:3,1:3,i)'*t_1(i,:)';0 0 0 1];
%     tr_2(:,:,i) = tr_marker(:,:,i)*[r_2(1:3,1:3,i), t_2(i,:)';0 0 0 1]; 
% end
tr_1=transl([0 0 0]');
tr_2=zeros(4,4,len);
tr_marker = zeros(4,4,len);
for i=1:len
    tr_marker(:,:,i) = (transl(t_1(i,:))*r_1(:,:,i));
    tr_2(:,:,i) = tr_marker(:,:,i)/(transl(t_2(i,:))*r_2(:,:,i)); 
end

end


