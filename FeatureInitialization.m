
% -------------------------Short Project-----------------------------------
% Non-constant Stereo Matching Approach for Feature Initialization 
% in Inverse-Depth Monocular SLAM
%
% Advanced Topics in Computer Vision
% Eric Guerrero Font
% ericguerrerofont@gmail.com
% 10/06/2015
%--------------------------------------------------------------------------
% 1- Run the Robotics Toolbox - Peter Corke.  (rcvtools-->startup_rvc.m)
% 2- Run this script to start.
% 3- After each frame the algorithm stops, press a key to continue.
% 4- Press Ctrl+C to exit.
%--------------------------------------------------------------------------
close all;clc;clear;
format compact; warning off;
addpath(genpath('Secuencias'))
addpath(genpath('Functions'))
%% Sequence
Sec = 'S2mov02'; % Small sequence of only 60 frames

%% Plots
plot3D = true;
plotpyr = true;
plotray = true;
plotblobs  = false;
plotimages = true;

%% Data
% Camera Parameters
[cam1, cameraParams1, cam2, cameraParams2] = camPar(Sec);
pmin = pi*10/180;  % Pllx min of 10º

% Camera Pose
[time1, time2, tr1, tr2, trM] = DataLoader(Sec);
ht1 = tr1; % Camera1 Pose (0 0 0)
t1 = transl(ht1);


%% Main Loop
it = 50;
laps = 1;
squarederror = zeros(1, it/laps);
Zerror = zeros(size(200,1),it/laps);
cputime = zeros(1, it/laps);
matchings = zeros(1, it/laps);
iteration=1;

for frame=1:laps:it
    tic
    ht2 = tr2(:,:,frame);   % Camera2 Pose
    htM = trM(:,:,frame);   % Marker Pose
    t2 = transl(ht2);
    dmax = norm(t1/2-t2/2)/(tan(pmin/2)); % Maximum depth for min parallax
    
    % Draw Pyramids and compute Intersection
    [Inter, Points, FlagInter] = Pyr (ht1, ht2, htM, cam1, cam2, dmax, plot3D, plotpyr);
    if FlagInter %If there is intersection
        
        % Load list of MONO features (Simulated)
        [listedfeatures, positions, sigma] = featureslist(time1,time2,frame,cam1,cam2,ht1,ht2,dmax,Sec);
        
        % Which of the listed features are inside the pyr intersection
        in = inhull(listedfeatures,Points,Inter); 
        index = find(in);
        listedfeaturesIN = listedfeatures(index,:); 
        positionsIN = positions(index,:);
        
        if ~isempty(listedfeaturesIN) % If there are listed features in the intersection
            
            % Read Stereo Pair
            [I1, I2] = readimages(time1(frame,:), time2(frame,:), Sec);  
            
            % Search Region
            [SRextremes] = searchRegion(listedfeaturesIN, Inter, Points, cam2, ht2, sigma,false);
            
            % Match stereo features
            [match1, match2, costmin, indexlist] = BRIEFcommonfeatures(I1,I2,cam2,ht2,positionsIN,listedfeaturesIN,SRextremes,false); 

            % Get depth update
            featureLoc = depthupdate(match1, match2,cam1,cam2,ht1,ht2,dmax,plotray);
            listedfeaturesUpdate = zeros(size(listedfeaturesIN,1),3);
            for k=1:size(featureLoc,1) % Return the features in the same order
               listedfeaturesUpdate(index(indexlist(k)),:) = featureLoc(k,:); % List of features updated
            end
        end
    end
    
    %% Results SE
    for i=1:size(featureLoc,1)
        % Inverse transformation
        invhtM = [htM(1:3,1:3)' -htM(1:3,1:3)'*htM(1:3,4); 0 0 0 1];
        featureLocMarker = invhtM*transl(featureLoc(i,:)');
        Zerror(i,iteration) = featureLocMarker(3,4);
    end
    squarederror(iteration) = mean(Zerror(:,iteration).^2);
    cputime(iteration) = toc;
    matchings(iteration) = size(match2,1);
    
    %% Plot
    % Match Features
    if plotimages
        figure(3);
        plotfigure(I2,I1,match2,match1,cam2.w,'Matched Features','y')
    end
    drawnow
    pause;
%     close all;
    iteration=iteration+1;
end
plot(squarederror)