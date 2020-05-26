% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function myMap = occGridMapping(ranges, scanAngles, pose, param)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters 
% 
% % the number of grids for 1 meter.
myResol = param.resol;

% % the initial map size in pixels
myMap = zeros(param.size);
% % the origin of the map in pixels
myorigin = param.origin; 
% 
% % 4. Log-odd parameters 
lo_occ = param.lo_occ;
lo_free = param.lo_free; 
lo_max = param.lo_max;
lo_min = param.lo_min;

N = size(pose,2);

for j = 1:N % for each time,
    occ_loc = [ranges(:,j).*cos(pose(3,j) + scanAngles) + pose(1,j), -ranges(:,j).*sin(pose(3,j) + scanAngles) + pose(2,j)];
    P = ceil( pose(1:2,j)*param.resol)+param.origin;
    glob_occ = ceil(occ_loc.*myResol) + myorigin';
    
    orig = [P(1) P(2)];
    for p = 1:size(glob_occ,1)
        occ = [glob_occ(p,1) glob_occ(p,2)];
        [freex, freey] = bresenham(orig(1),orig(2),occ(1),occ(2));
        
        
        free = sub2ind(size(myMap),freey,freex);
        occ1 = sub2ind(size(myMap),occ(2),occ(1));
        
 
    for d = 1:length(free)

        myMap(free(d)) = myMap(free(d)) - lo_free;

        if myMap(free(d))<lo_min

            myMap(free(d))=lo_min;

        end

    end

    myMap(occ1) = myMap(occ1) + lo_occ;

    if myMap(occ1)>lo_max

        myMap(occ1)=lo_max;

    end
    end

end

end

