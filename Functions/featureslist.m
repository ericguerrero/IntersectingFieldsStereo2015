function [features, positions, sigma] = featureslist(time1,time2,frame,cam1,cam2,ht1,ht2,dmax,Secuencia)
% Read images
[I1, I2] = readimages(time1(frame,:), time2(frame,:),Secuencia);

% Match features
[match1, match2, blobs1] = matchfeatures(I1,I2);

% Compute depth
f = length(match1);   % Number of matches
features = zeros(f,3);
for i=1:f
    % Feature Position
    p1 = ray(cam1, ht1, dmax, match1(i,:),'r',false); %Feature Position 1
    p2 = ray(cam2, ht2, dmax, match2(i,:),'g',false); %Feature Position 2
    t1=transl(ht1);t2=transl(ht2);
    res = intersectLines(t1,p1,t2,p2);          %Feature Position (Mid Point)
    features(i,:) = ((res(1:3)+res(4:6))/2)';   %Feature Position Array
%     plot3(features(i,1),features(i,2),features(i,3),'*b')
%     plot3(res(1:3:4),res(2:3:5),res(3:3:6),'b')
end
positions = match1;

% Noise Insertion
Var = 0.05; 
sigma = abs(mvnrnd(0,Var^2,size(features,1)));
for i=1:f
    margin = mvnrnd(0,sigma(i),1);
    noise = features(i,:)*margin;
    features(i,:) = features(i,:)+noise;
end

end