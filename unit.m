%% Author: George Hinds

function b_unit = unit(bmeasure)
    rows = size(bmeasure, 1);
    columns = size(bmeasure, 2);
    b_unit = zeros(rows,columns);

    beacons = cell(rows, 1);

    for i = 1:rows
        beacons{i} = bmeasure(i,:);
        b_unit(i, :) = bmeasure(i,:) / norm(beacons{i});
    end
end