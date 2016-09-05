
tic
%% Parameters to tune
plot=false
S=64; % patch size
s=S/2;
thrcost = 20;

%% Image positions
% Features in image1
 match1 = positions;

if plot
    figure(1);
    subplot(121);
    imshow(I1);hold on;
    plot(match1(:,1),match1(:,2),'+r');
end

% Features in image2
K2 = cam2.P(1:3,1:3);
points2 = zeros(size(listedfeatures,1),2);
for i=1:size(listedfeatures,1)
    p_world = transl(listedfeatures(i,:)');
    p_camera = transl(inv(ht2)*p_world);
    V = K2 * p_camera;
    points2(i,:)=V(1:2)/V(3);
end

if plot
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

%% Descriptor
match2 = zeros(length(match1),2);
costmin = zeros(length(match1),1);
indexlist =[];
for i=1:length(match1)
    tau2=[];
    u = round(match1(i,1));
    v = round(match1(i,2));
    patch1 = I1_f(v:(v+S),u:(u+S)); %Get patch from padded image
    descr1 = vl_covdet(im2single(patch1));
    
    if plot
        figure(1);
        subplot(121);rectangle('Position',[u-s,v-s,S,S],'EdgeColor','y')
    end
    
    %% Pixels arround the serch line
    SR = round([SRextremes(i,1:2);SRextremes(i,3:4)]);
    SRmax = [max(SR(:,1)) max(SR(:,2))];
    SRmin = [min(SR(:,1)) min(SR(:,2))]; 
    % Avoid rounding issues
    if SRmin(1)==0
        SRmin(1)=1;
    end
    if SRmin(2)==0
        SRmin(2)=1;
    end
    if SRmax(1)>cam2.w
        SRmax(1)=cam2.w;
    end
    if SRmax(2)>cam2.w
        SRmax(2)=cam2.w;
    end
    
    % Line
    M = -((SRmax(2)-SRmin(2))/(SRmax(1)-SRmin(1)));
    N = -SRmax(2)-M*SRmax(1);
    
    if plot
        figure(4);hold on;
        plot(SR(:,1),SR(:,2),'g')
    end
    
    tic
    Pixel = [];
    
    descr2=[];
    q=[];
    d=[];
    for n=SRmin(1):2:SRmax(1)
        for m=SRmin(2):2:SRmax(2)
            if (abs(M*n+N+m)<1)
                Pixel=[Pixel;n m];
                patch2 = I2_f(m:(m+S),n:(n+S)); %Get patch from padded image
                
                d = vl_covdet(im2single(patch2))
                descr2 = [descr2 d ];
                q = [q; size(d,2)];
                
                if plot
                    figure(4);hold on
                    plot(n,m,'g')
                end
            end
        end
    end
    toc
                    
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
if plot
    % Pairs 1
    figure(3);subplot(121);
    imshow(patch1);hold on
    plot(X(:,1),X(:,2),'.r')
    plot(Y(:,1),Y(:,2),'.g')
    for k=1:pairs
        plot([X(k,1); Y(k,1)],[X(k,2); Y(k,2)],'b');
    end


    % Match 2
    figure(4);hold on
    plot(match2(:,1), match2(:,2),'+r')
    plot(points2(:,1), points2(:,2),'+g')
    for i=1:length(match2)
        plot([points2(i,1) match2(i,1)], [points2(i,2) match2(i,2)],'b')
    end
end