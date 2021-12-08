fixed = ptCloudA;
moving = ptCloudB;
maxDistance = 0.4;
referenceVector = [0 0 1];
groundMoving = pcfitplane(moving,maxDistance,referenceVector);
groundFixed = pcfitplane(fixed,maxDistance,referenceVector);
tformMoving = normalRotation(groundMoving,referenceVector);
tformFixed = normalRotation(groundFixed,referenceVector);
movingCorrected = pctransform(moving,tformMoving);
fixedCorrected = pctransform(fixed,tformFixed);
gridSize = 100;
gridStep = 0.5;
tform = pcregistercorr(movingCorrected,fixedCorrected,gridSize,gridStep);
combinedTform = rigid3d(tform.T * tformMoving.T * tformFixed.T);
movingReg = pctransform(moving,combinedTform);
mergeSize = 0.015;
ptCloudScene = pcmerge(fixed, movingReg, mergeSize);

% Visualize the input images.
figure
pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down')
title('Initial world scene')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
