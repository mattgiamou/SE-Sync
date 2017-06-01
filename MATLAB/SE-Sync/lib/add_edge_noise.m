function [measurements_out, id] = add_edge_noise(measurements, theta)
%add_edge_noise Add edge noise to the pose graph measurements. Assumes that
%odometry edges are listed first in measurements.edges.

% Non-odometry index start (loop closures)
id_lc_min = find(measurements.edges(:,1) ~= measurements.edges(:,2)-1, 1);
id_lc_max = size(measurements.edges, 1);
N = id_lc_max - id_lc_min;
id = randi(N, 1) + id_lc_min;
R = random_orientation_matrix(theta);
r = measurements.R(id);
r{:}
measurements.R(id) = {R*r{:}};
r2 = measurements.R(id);
r2{:}

measurements_out = measurements;

end

