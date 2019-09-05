%infilename = '/home/ivaughn/dev/navG3/navg3/dslmap/tests/resources/cape_cod.grd';
infilename = 'synthetic_terrain.grd';

x_range = ncread(infilename, 'x_range');
y_range = ncread(infilename, 'y_range');
z_range = ncread(infilename, 'z_range');
dims = ncread(infilename, 'dimension');
raw_data = ncread(infilename, 'z');
z = reshape(raw_data, dims(1), dims(2));
z_corr = flipud(z');

figure(2);
imagesc(linspace(x_range(1), x_range(2), dims(1)),...
    linspace(y_range(1), y_range(2), dims(2)),...
    z_corr);
axis xy;
grid on;
colorbar;
