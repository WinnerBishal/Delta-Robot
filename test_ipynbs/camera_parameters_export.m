% Extract the necessary calibration parameters
intrinsicMatrix = cameraParams.IntrinsicMatrix;
radialDistortion = cameraParams.RadialDistortion;
tangentialDistortion = cameraParams.TangentialDistortion;

% Create a structure to store the parameters
paramsStruct = struct();
paramsStruct.IntrinsicMatrix = intrinsicMatrix';
paramsStruct.RadialDistortion = radialDistortion;
paramsStruct.TangentialDistortion = tangentialDistortion;

% Convert the structure to a JSON string
jsonParams = jsonencode(paramsStruct);

% Write the JSON string to a file
fid = fopen('cameraParams.json', 'w');
if fid == -1, error('Cannot create JSON file'); end
fwrite(fid, jsonParams, 'char');
fclose(fid);

