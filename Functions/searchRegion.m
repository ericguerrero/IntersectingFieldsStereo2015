function [SRextremes] = searchRegion(listedfeaturesIN, Inter, Points, cam2,ht2,sigma,plots)



% Variance Insertion
sRegA=zeros(size(listedfeaturesIN,1),6);
for i=1:size(listedfeaturesIN,1)
    noise = listedfeaturesIN(i,:)*sigma(i)*1.96; % 95% Confidence interval
    depth1 = listedfeaturesIN(i,:)-noise;
    depth2 = listedfeaturesIN(i,:)+noise;
    sRegA(i,:)=[depth1 depth2];
end

% Polytope intersection
sRegB=zeros(size(listedfeaturesIN,1),6);
for i = 1:size(listedfeaturesIN,1)
    L = [0 0 0; 10*listedfeaturesIN(i,:)]';
    
%     figure(1);subplot(223);hold on
%     plot3([L(1,1),L(1,2)],[L(2,1),L(2,2)],[L(3,1),L(3,2)],'r');
    
    II=[];
    for j = 1:size(Inter,1)
        pn = Inter(j,:);
        P = [Points(pn(1),:); Points(pn(2),:); Points(pn(3),:)]';
        
        [I,check,t,u,v]=triangle_line_intersect(P,L);
        if check==1
            II=[II, I'];
        end
    end
    if size(II,2)==6;
        sRegB(i,:)=II;
    end
end
% Compare
sReg=sRegA;
for i = 1:size(listedfeaturesIN,1)
    if sRegA(i,3)<sRegB(i,3)
        sReg(i,1:3)=sRegB(i,1:3);
    end
    if sRegA(i,6)>sRegB(i,6)
        sReg(i,4:6)=sRegB(i,4:6);
    end
end



% Estimated points in image2
K2 = cam2.P(1:3,1:3);
SRextremes = zeros(size(sReg,1),4);
for i=1:size(listedfeaturesIN,1)
    p_world = transl(sReg(i,1:3)');
    p_camera = transl(inv(ht2)*p_world);
    V = K2 * p_camera;
    SRextremes(i,1:2)=V(1:2)/V(3);
    
    p_world = transl(sReg(i,4:6)');
    p_camera = transl(inv(ht2)*p_world);
    V = K2 * p_camera;
    SRextremes(i,3:4)=V(1:2)/V(3);
end

%% Plot
if plots
figure(1);subplot(223);hold on
for i=1:size(sReg,1)
    plot3([sReg(i,1) sReg(i,4)],[sReg(i,2) sReg(i,5)],[sReg(i,3) sReg(i,6)],'*k')
end

figure(2);
imshow(I2);hold on
for i=1:size(SRextremes,1)
    plot([SRextremes(i,1) SRextremes(i,3)],[SRextremes(i,2) SRextremes(i,4)],'r')
    plot([SRextremes(i,1) SRextremes(i,3)],[SRextremes(i,2) SRextremes(i,4)],'*r')
end
    
K2 = cam2.P(1:3,1:3);
sPointI2 = zeros(size(sReg,1),4);
for i=1:size(listedfeaturesIN,1)
    p_world = transl(listedfeaturesIN(i,1:3)');
    p_camera = transl(inv(ht2)*p_world);
    V = K2 * p_camera;
    sPointI2(i,1:2)=V(1:2)/V(3);
end
for i=1:size(SRextremes,1)
    plot([sPointI2(i,1)],[sPointI2(i,2)],'*g')
end
end
