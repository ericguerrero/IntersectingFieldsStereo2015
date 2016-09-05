function [match1, match2, costmin, indexlist] = BRIEFcommonfeatures(I1,I2,cam2,ht2,positionsIN,listedfeaturesIN,SRextremes,plots)

%% Parameters to tune
S=32; % patch size
s=S/2;
thrcost = 65;
precision = 2;

%% Image positions
% Features in image1
 match1 = positionsIN;

if plots
    figure(1);
    subplot(121);
    imshow(I1);hold on;
    plot(match1(:,1),match1(:,2),'+r');
end

% Features in image2
K2 = cam2.P(1:3,1:3);
points2 = zeros(size(listedfeaturesIN,1),2);
for i=1:size(listedfeaturesIN,1)
    p_world = transl(listedfeaturesIN(i,:)');
    p_camera = transl(inv(ht2)*p_world);
    V = K2 * p_camera;
    points2(i,:)=V(1:2)/V(3);
end

if plots
    subplot(122);
    imshow(I2);hold on;
    plot(points2(:,1),points2(:,2),'+g');
end
    
%% Filtering
h = fspecial('gaussian', 9, 1.5);
I1_f = imfilter(I1,h); 
I1_f = padarray(I1_f,[s s]);
I2_f = imfilter(I2,h); 
I2_f = padarray(I2_f,[s s]);

%% Spatial Arrengement
% Image 1
mu = [0 0];
sigma = [S^2/10,0;0,S^2/10];
pairs = 256;
X = round(mvnrnd(mu,sigma,pairs)+s); 
Y = round(mvnrnd(mu,sigma,pairs)+s);
I = find(X<1);X(I)=1;
I = find(X>S);X(I)=S-1;
I = find(Y<1);Y(I)=1;
I = find(Y>S);Y(I)=S-1; 


%% Pair rotation
% Image 2
Xr=X;
Yr=Y;
% Xr=zeros(size(X));
% Yr=zeros(size(Y));
% for i=1:size(X,1)
%     u =[X(i,:)-s 1]*ht2(1:3,1:3);
%     Xr(i,:)= round(u(1:2)+s);
%     v =[Y(i,:)-s 1]*ht2(1:3,1:3);
%     Yr(i,:)= round(v(1:2)+s);
% end
% I = find(Xr<1);Xr(I)=1;
% I = find(Xr>S);Xr(I)=S-1;
% I = find(Yr<1);Yr(I)=1;
% I = find(Yr>S);Yr(I)=S-1; 



if plots
    figure(4);
    imshow(I2);hold on
end

%% Descriptor
match2 = zeros(length(match1),2);
costmin = zeros(length(match1),1);
indexlist =[];
for i=1:length(match1)
    tau2=[];
    u = round(match1(i,1));
    v = round(match1(i,2));
    patch1 = I1_f(v:(v+S),u:(u+S)); %Get patch from padded image
    for j=1:pairs
        px = patch1(X(j,1),X(j,2));
        py = patch1(Y(j,1),Y(j,2));
        if px<py
            tau1(j) = 1;
        else
            tau1(j) = 0;
        end
    end
    
    if plots
        figure(1);
        subplot(121);rectangle('Position',[u-s,v-s,S,S],'EdgeColor','y')
    end
    
    %% Pixels arround the serch line

    SR = round([SRextremes(i,1:2);SRextremes(i,3:4)]);
    SRmax = [max(SR(:,1)) max(SR(:,2))];
    SRmin = [min(SR(:,1)) min(SR(:,2))]; 
    % Avoid rounding issues
    if SRmin(1)<=0
        SRmin(1)=1;
    end
    if SRmin(2)<=0
        SRmin(2)=1;
    end
    if SRmax(1)>cam2.w
        SRmax(1)=cam2.w;
    end
    if SRmax(2)>cam2.h
        SRmax(2)=cam2.h;
    end
    
    % Line
    M = -((SRmax(2)-SRmin(2))/(SRmax(1)-SRmin(1)));
    N = -SRmax(2)-M*SRmax(1);
    
    if plots
        figure(4);hold on;
        plot(SR(:,1),SR(:,2),'r')
    end
    
   
    Pixel = [];
    trials = 1;
    for n=SRmin(1):precision:SRmax(1)
        for m=SRmin(2):precision:SRmax(2)
            if (abs(M*n+N+m)<1)
                Pixel=[Pixel;n m];
                patch2 = I2_f(m:(m+S),n:(n+S)); %Get patch from padded image
                
%                 if plots
%                     figure(4);hold on
%                     plot(n,m,'*w')
%                 end
                
                for j=1:pairs
                    px = patch2(Xr(j,1),Xr(j,2));
                    py = patch2(Yr(j,1),Yr(j,2));
                    if px<py
                        tau2(trials,j) = 1;
                    else
                        tau2(trials,j) = 0;
                    end
                end
                trials = trials+1;
            end
        end
    end
    
                    
    %% Hamming Distance
    if ~isempty(Pixel)
        cost=zeros(size(tau2,1),1);
        for k=1:size(tau2,1)
            for j=1:size(tau2,2)
                cost(k)=cost(k)+abs(tau1(1,j)-tau2(k,j));
            end
        end
        [mincost,Index]=min(cost);
        costmin(i)=mincost;
        if mincost<=thrcost
            match2(i,:) = Pixel(Index,:);
            indexlist = [indexlist;i];
        end
    end
end

match1=match1(indexlist,:);
match2=match2(indexlist,:);

%% Plot
if plots
    % Pairs 1
    figure(3);subplot(121);
    imshow(patch1);hold on
    plot(X(:,1),X(:,2),'.r')
    plot(Y(:,1),Y(:,2),'.g')
    for k=1:pairs
        plot([X(k,1); Y(k,1)],[X(k,2); Y(k,2)],'b');
    end
    % Pairs 2
    figure(3);subplot(122);
    imshow(patch2);hold on
    plot(Xr(:,1),Xr(:,2),'.r')
    plot(Yr(:,1),Yr(:,2),'.g')
    for k=1:pairs
        plot([Xr(k,1); Yr(k,1)],[Xr(k,2); Yr(k,2)],'b');
    end


    % Match 2
    figure(4);hold on
    plot(match2(:,1), match2(:,2),'ob','MarkerSize',10)
    plot(points2(:,1), points2(:,2),'+g')
    for i=1:length(match2)
        plot([points2(indexlist(i),1) match2(i,1)], [points2(indexlist(i),2) match2(i,2)],'b')
    end
end