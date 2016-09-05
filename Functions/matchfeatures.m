function [match1, match2, blobs1] = matchfeatures(I1,I2)

% Collect Interest Points from Each Image
blobs1 = detectSURFFeatures(I1);
blobs2 = detectSURFFeatures(I2);

% Extract features
[features1, validBlobs1] = extractFeatures(I1, blobs1);
[features2, validBlobs2] = extractFeatures(I2, blobs2);

% Match features
indexPairs = matchFeatures(features1, features2);

matchedPoints1 = validBlobs1(indexPairs(:,1),:);
matchedPoints2 = validBlobs2(indexPairs(:,2),:);

% Estimate inlayers
[tform, inlier1, inlier2] = estimateGeometricTransform(...
    matchedPoints1, matchedPoints2, 'projective');

match1 = inlier1.Location;
match2 = inlier2.Location;

end


