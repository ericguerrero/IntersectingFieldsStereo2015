function [ res ] = intersectLines( A1,A2, B1, B2 )
%INTERSECTLINES Summary of this function goes here
%   Detailed explanation goes here
%   each line (A, B) is described by two separate points (1 and 2) 
%   lying on it. Each point is given a Xi = [x y z];

%   A0 and B0 are the projections over each line of the
%   nearest point between them. Ideally would be the same,
%   and a good aprox. being both lines of same uncertainty
%   is the median between A0 and B0


 nA = dot(cross(B2-B1,A1-B1),cross(A2-A1,B2-B1));
 nB = dot(cross(A2-A1,A1-B1),cross(A2-A1,B2-B1));
 d = dot(cross(A2-A1,B2-B1),cross(A2-A1,B2-B1));
 A0 = A1 + (nA/d)*(A2-A1);
 B0 = B1 + (nB/d)*(B2-B1);
 res = [A0; B0];


end

