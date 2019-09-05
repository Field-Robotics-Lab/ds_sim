img = imread('synthetic_terrain.tif');
final_dimensions_m = [25600 25600 400];
origin_lat = 10;
origin_lon = 0;
origin_dep = 400;

outfilename = 'synthetic_terrain.grd';

% Prepare the data
min_lon = origin_lon - (final_dimensions_m(1)/2)/mdeglon(origin_lat);
max_lon = origin_lon + (final_dimensions_m(1)/2)/mdeglon(origin_lat);

min_lat = origin_lat - (final_dimensions_m(2)/2)/mdeglat(origin_lat);
max_lat = origin_lat + (final_dimensions_m(2)/2)/mdeglat(origin_lat);

min_dep = origin_dep - final_dimensions_m(3)/2;
max_dep = origin_dep + final_dimensions_m(3)/2;

min_z = double(min(reshape(img, 1, [])));
max_z = double(max(reshape(img, 1, [])));
z_scaled = (double(img) - min_z)/(max_z - min_z) * final_dimensions_m(3) + origin_dep;
z_prep = flipud(z_scaled)';
z_raw = reshape(z_prep, [], 1);


% Create the Net CDF file
ncid = netcdf.create(outfilename, 'NC_WRITE');

% Define dimensions
dim_side = netcdf.defDim(ncid, 'side', 2);
dim_xysize = netcdf.defDim(ncid, 'xysize', numel(img));

% Define variables
var_xrange = netcdf.defVar(ncid, 'x_range', 'double', [dim_side]);
var_yrange = netcdf.defVar(ncid, 'y_range', 'double', [dim_side]);
var_zrange = netcdf.defVar(ncid, 'z_range', 'double', [dim_side]);
var_spacing = netcdf.defVar(ncid, 'spacing', 'double', [dim_side]);
var_dim = netcdf.defVar(ncid, 'dimension', 'int', [dim_side]);
var_z = netcdf.defVar(ncid, 'z', 'float', [dim_xysize]);

% Finish the header
netcdf.endDef(ncid);

% Actually store data
netcdf.putVar(ncid, var_xrange, [min_lon, max_lon]);
netcdf.putVar(ncid, var_yrange, [min_lat, max_lat]);
netcdf.putVar(ncid, var_zrange, [min_dep, max_dep]);
netcdf.putVar(ncid, var_spacing, [min_lon, max_lon]);% TODO
netcdf.putVar(ncid, var_dim, [size(img,2), size(img,1)]);% TODO
netcdf.putVar(ncid, var_z, z_raw);

netcdf.close(ncid);
