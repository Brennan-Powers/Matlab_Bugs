%BUG Tangent Bug navigation class
%
% A concrete subclass of the abstract Navigation class that implements the bug2 
% navigation algorithm.  This is a simple automaton that performs local 
% planning, that is, it can only sense the immediate presence of an obstacle.
%
% Methods::
%   Bug2        Constructor
%   query       Find a path from start to goal
%   plot        Display the obstacle map
%   display     Display state/parameters in human readable form
%   char        Convert to string
%
% Example::
%         load map1             % load the map
%         bug = Bug2(map);      % create navigation object
%         start = [20,10]; 
%         goal = [50,35];
%         bug.query(start, goal);   % animate path
%
% Reference::
% -  Dynamic path planning for a mobile automaton with limited information on the environment,,
%    V. Lumelsky and A. Stepanov, 
%    IEEE Transactions on Automatic Control, vol. 31, pp. 1058-1063, Nov. 1986.
% -  Robotics, Vision & Control, Sec 5.1.2,
%    Peter Corke, Springer, 2011.
%  
% See also Navigation, DXform, Dstar, PRM.



% Copyright (C) 1993-2017, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com

classdef tangentBug < Navigation

    properties(Access=protected)
        H       % hit points
        j       % number of hit points
        mline   % line from starting position to goal
        step    % state, in step 1 or step 2 of algorithm
        edge    % edge list
        k       % edge index
        tang    % saved tangent
        obj     % 0 if no objects have been detected yet, 1 if there has been
        cd      % 0 if bug should circle object right, 1 for left
        %g       % 1 if stuck due to equal tangent points, otherwise 0
        %dedge   % edges detected
    end

    methods

        function bug = tangentBug(varargin)
            %Bug2.Bug2 Construct a Bug2 navigation object 
            %
            % B = Bug2(MAP, OPTIONS) is a bug2 navigation object, and MAP is an occupancy grid,
            % a representation of a planar world as a matrix whose elements are 0 (free
            % space) or 1 (occupied).
            %
            % Options::
            % 'goal',G      Specify the goal point (1x2)
            % 'inflate',K   Inflate all obstacles by K cells.
            %
            % See also Navigation.Navigation.

            % invoke the superclass constructor
            bug = bug@Navigation(varargin{:});

            bug.H = [];
            bug.j = 1;
            bug.step = 1;
            bug.obj = 0;
            bug.cd = 2;
        end

        function pp = query(bug, start, goal, varargin)
            %Bug2.query  Find a path
            %
            % B.query(START, GOAL, OPTIONS) is the path (Nx2) from START (1x2) to GOAL
            % (1x2).  Row are the coordinates of successive points along the path.  If
            % either START or GOAL is [] the grid map is displayed and the user is
            % prompted to select a point by clicking on the plot.
            %
            % Options::
            %  'animate'   show a simulation of the robot moving along the path
            %  'movie',M   create a movie
            %  'current'   show the current position position as a black circle
            %
            % Notes::
            % - START and GOAL are given as X,Y coordinates in the grid map, not as
            %   MATLAB row and column coordinates.
            % - START and GOAL are tested to ensure they lie in free space.
            % - The Bug2 algorithm is completely reactive so there is no planning
            %   method.
            % - If the bug does a lot of back tracking it's hard to see the current
            %   position, use the 'current' option.
            % - For the movie option if M contains an extension a movie file with that
            %   extension is created.  Otherwise a folder will be created containing
            %   individual frames.
            %
            % See also Animate.
         
            opt.animate = false;
            opt.movie = [];
            opt.current = false;
            
            opt = tb_optparse(opt, varargin);
            
            if ~isempty(opt.movie)
                anim = Animate(opt.movie);
                opt.animate = true;
            end
       
            % make sure start and goal are set and valid
            bug.start = []; bug.goal = [];
            bug.checkquery(start, goal);
            
            % compute the m-line
            %  create homogeneous representation of the line
            %  line*[x y 1]' = 0
            bug.mline = homline(bug.start(1), bug.start(2), ...
                bug.goal(1), bug.goal(2));
            bug.mline = bug.mline / norm(bug.mline(1:2));
            
            if opt.animate
                bug.plot();
                
                bug.plot_mline();
            end
            
            % iterate using the next() method until we reach the goal
            robot = bug.start(:);
            bug.step = 1;
            path = bug.start(:);
            while true
                if opt.animate
                    plot(robot(1), robot(2), 'g.', 'MarkerSize', 12);
                    if opt.current
                        h = plot(robot(1), robot(2), 'ko', 'MarkerSize', 8);
                    end
                    drawnow
                    if ~isempty(opt.movie)
                        anim.add();
                    end
                    if opt.current
                        delete(h)
                    end
                end

                % move to next point on path
                robot = bug.next(robot);

                % are we there yet?
                if isempty(robot)
                    % yes, exit the loop
                    break
                else
                    % no, append it to the path
                    path = [path robot(:)];
                end
            end
            
            if ~isempty(opt.movie)
                anim.close();
            end

            % only return the path if required
            if nargout > 0
                pp = path';
            end
        end
        
        function plot_mline(bug, ls)
            
                % parameters of the M-line, direct from initial position to goal
                % as a vector mline, such that [robot 1]*mline = 0
                
                if nargin < 2
                    ls = 'k--';
                end
                dims = axis;
                xmin = dims(1); xmax = dims(2);
                ymin = dims(3); ymax = dims(4);
                
                hold on
                if bug.mline(2) == 0
                    % handle the case that the line is vertical
                    %plot([robot(1) robot(1)], [ymin ymax], 'k--');
                else
                    x = [xmin xmax]';
                    y = -[x [1;1]] * [bug.mline(1); bug.mline(3)] / bug.mline(2);
                    plot(x, y, ls);
                end
        end
        
        function n = next(bug, robot)
            
            % implement the main state machine for bug2
            n = [];
            robot = robot(:);
            dedge = [];    % list of detected edges
            rad = 20;      % radius of detection
            inc = 1;       % increments of detection rays, in degrees
            
            % these are coordinates (x,y)
          
            if bug.step == 1
                % Step 1.  Move along the M-line toward the goal

                if colnorm(bug.goal - robot) == 0 % are we there yet?
                    return
                end
                
                bug.mline = homline(robot(1), robot(2), ...
                    bug.goal(1), bug.goal(2));
                bug.mline = bug.mline / norm(bug.mline(1:2));

                % motion on line toward goal
                d = bug.goal-robot;
                if abs(d(1)) > abs(d(2))
                    % line slope less than 45 deg
                    dx = sign(d(1));
                    L = bug.mline;
                    y = -( (robot(1)+dx)*L(1) + L(3) ) / L(2);
                    dy = round(y - robot(2));
                else
                    % line slope greater than 45 deg
                    dy = sign(d(2));
                    L = bug.mline;
                    x = -( (robot(2)+dy)*L(2) + L(3) ) / L(1);
                    dx = round(x - robot(1));
                end
                
                if bug.isoccupied(robot + [2;0]) || ...
                    bug.isoccupied(robot + [2;2]) || ...
                    bug.isoccupied(robot + [0;2]) || ...
                    bug.isoccupied(robot + [0;-2]) || ...
                    bug.isoccupied(robot + [-2;0]) || ...
                    bug.isoccupied(robot + [-2;-2]) || ...
                    bug.isoccupied(robot + [-2;2]) || ...
                    bug.isoccupied(robot + [2;-2])
                
                    inc = 0.25;
                
                elseif bug.isoccupied(robot + [3;0]) || ...
                    bug.isoccupied(robot + [3;3]) || ...
                    bug.isoccupied(robot + [0;3]) || ...
                    bug.isoccupied(robot + [0;-3]) || ...
                    bug.isoccupied(robot + [-3;0]) || ...
                    bug.isoccupied(robot + [-3;-3]) || ...
                    bug.isoccupied(robot + [-3;3]) || ...
                    bug.isoccupied(robot + [3;-3])
                
                    inc = 0.33334;
                elseif bug.isoccupied(robot + [4;0]) || ...
                    bug.isoccupied(robot + [4;4]) || ...
                    bug.isoccupied(robot + [0;4]) || ...
                    bug.isoccupied(robot + [0;-4]) || ...
                    bug.isoccupied(robot + [-4;0]) || ...
                    bug.isoccupied(robot + [-4;-4]) || ...
                    bug.isoccupied(robot + [-4;4]) || ...
                    bug.isoccupied(robot + [4;-4])
                
                    inc = 0.5;
                else
                    %nada
                end
                
                    
                
                % Get dedge list of all detected edges in 360
                for i=0 : inc : 360
                    for q=1 : rad
                        if bug.isoccupied([robot(1) + round(q*cosd(i)), robot(2) + round(q*sind(i))])
                            dedge = [dedge [robot(1) + round((q-1)*cosd(i)); robot(2) + round((q-1)*sind(i))]];
                            break;
                        end
                    end
                end
                
                inc = 1;
                
                % Delete all duplicate points from dedge
                dedge = unique(dedge.','rows','stable').';
                
                %dedge(1:2,1);
                dedge_size = size(dedge, 2);
                z = 1;
                while z <= dedge_size
                    v = dedge(1:2,z);
                    if v(1) == 1 || v(1) == 128
                        dedge(:,z) = [];
                        z = z - 1;
                    end
                    dedge_size = size(dedge, 2);
                    z = z + 1;
                end
                
                z = 1;
                while z <= dedge_size
                    v = dedge(1:2,z);
                    if v(2) == 1 || v(2) == 128
                        dedge(:,z) = [];
                        z = z - 1;
                    end
                    dedge_size = size(dedge, 2);
                    z = z + 1;
                end
                
%                 z = 1;
%                 while z <= dedge_size
%                     v = dedge(1:2,z);
%                     if v(2) == 129
%                         dedge(:,z) = [];
%                         z = z - 1;
%                     end
%                     dedge_size = size(dedge, 2);
%                     z = z + 1;
%                 end
%                 
%                 z = 1;
%                 while z <= dedge_size
%                     v = dedge(1:2,z);
%                     if v(1) == 129
%                         dedge(:,z) = [];
%                         z = z - 1;
%                     end
%                     dedge_size = size(dedge, 2);
%                     z = z + 1;
%                 end
                
                if isempty(dedge) && bug.obj == 0
                    bug.mline = homline(bug.start(1), bug.start(2), ...
                        bug.goal(1), bug.goal(2));
                    bug.mline = bug.mline / norm(bug.mline(1:2));

                    % motion on line toward goal
                    d = bug.goal-robot;
                    if abs(d(1)) > abs(d(2))
                        % line slope less than 45 deg
                        dx = sign(d(1));
                        L = bug.mline;
                        y = -( (robot(1)+dx)*L(1) + L(3) ) / L(2);
                        dy = round(y - robot(2));
                    else
                        % line slope greater than 45 deg
                        dy = sign(d(2));
                        L = bug.mline;
                        x = -( (robot(2)+dy)*L(2) + L(3) ) / L(1);
                        dx = round(x - robot(1));
                    end
                    
                    
                    n = robot + [dx; dy];
                    return;
                end
                    
                %dedge_size = size(dedge, 2);
                
%                 for i=1 : dedge_size
%                     v = dedge(1:2,i);
%                     if v(1) == 0
%                         dedge(:,i) = [];
%                         i = i - 1;
%                     end
%                     dedge_size = size(dedge, 2);
%                 end
%                 
%                 for i=1 : dedge_size
%                     v = dedge(1:2,i);
%                     if v(2) == 0
%                         dedge(:,i) = [];
%                     end
%                     dedge_size = size(dedge, 2);
%                 end
                
                
                
                % Detect if there is object between robot and goal
                da = round(atand(d(2)/d(1)));  % da= angle in deg between robot and goal
                %if da < 0
                %    da = da * -1;
                %end
                if d(1) < 0 && d(2) < 0   % goal is bottom left
                    da = da + 180;
                elseif d(1) > 0 && d(2) > 0   % goal is top right
                    %da = da;
                elseif d(1) >= 0 && d(2) < 0   % goal is bottom right
                    %da = da;
                elseif d(1) < 0 && d(2) >= 0   % goal is top left
                    da = da + 180;
                else
                    %nada
                end
                    
                blocker = 0;   % 0 for no object between robot and goal, 1 if there is
                for q=1 : rad
                    c = [robot(1) + round(q*cosd(da)); robot(2) + round(q*sind(da))];
                    if c(1) == bug.goal(1) && c(2) == bug.goal(2)
                        break;
                    end
                    if norm(bug.goal - robot) < norm(c - robot)
                        break;
                    end
                    if bug.isoccupied([robot(1) + round(q*cosd(da)), robot(2) + round(q*sind(da))])
                         blocker = 1;
                         break;
                    end
                end
                
                
                % so at this point I have the complete detected edge list.
                % and blocker at 1 or 0.
                % dx and dy have been determined once but have not checked
                % if next step is into an obstacle.
                % dx/dy is currently straight from robot to goal.
                % mline has been created once before the dx/dy calc.
                
                % need to decide what to do now.
                % 
                % is next step into an obstacle?
                % if it is what do?
                % 
                % if blocker=1, then need to get new dx/dy
                % if blocker=0, there cant be an object in next step cause
                % dx/dy is currently straight from robot to goal. 
                % So just need to take next step.

%                % detect if next step is into object
%                if bug.isoccupied(robot + [dx; dy])
%                    bug.message('(%d,%d) obstacle!', n);
%                    bug.H(bug.j,:) = robot; % define hit point
%                    bug.step = 2;
%                    % get a list of all the points around the obstacle
%                    bug.edge = edgelist(bug.occgridnav == 0, robot);
%                    bug.k = 2;  % skip the first edge point, we are already there
%                end
                
                if blocker == 0
%                    if dx > 1
%                        dx = 1;
%                    end
%                    if dy > 1
%                        dy = 1;
%                    end
                    
                    n = robot + [dx; dy];
                    return; % just need to not run rest of bug.step1 section
                end
                
                if blocker == 1
                    
                    % If robot is directly next to object, then instead of
                    % using points of the object detected with rays, detect
                    % them using the edgelist because you are too close for
                    % the rays to get every edgepoint.
                    
                    
                    if bug.isoccupied(robot + [1;0]) || ...
                        bug.isoccupied(robot + [1;1]) || ...
                        bug.isoccupied(robot + [0;1]) || ...
                        bug.isoccupied(robot + [0;-1]) || ...
                        bug.isoccupied(robot + [-1;0]) || ...
                        bug.isoccupied(robot + [-1;-1]) || ...
                        bug.isoccupied(robot + [-1;1]) || ...
                        bug.isoccupied(robot + [1;-1])

                       bug.H(bug.j,:) = robot; % define hit point
                       % bug.step = 2;
                       % get a list of all the points around the obstacle
                       bug.edge = edgelist(bug.occgridnav == 0, robot);
                       bug.k = 2;  % skip the first edge point, we are already there
                       
                       % need to create list of edge points the robot could
                       % theoretically see from bug.edge.
                       % Can't just see 20 steps along bug.edge because
                       % they could be around a corner so not actually
                       % visible to the robot at the moment.
                       % maybe plot a line between robot and point and see
                       % if any points along the line are occupied. 
                       % If occupied, then that point is not visible, if no
                       % occupied along line then the point is visible.
                       
                       
                       %left_tang = bug.edge(1:2,(size(bug.edge, 2))-1 );
                       %right_tang = bug.edge(1:2,2);
                       
                       %left_20 = [;];
                       %right_20 = [;];
                       
                       %for k = 2 : 20
                       %    right_20 = [right_20 bug.edge(1:2,k)];
                       %end
                       
                       %for k = size(bug.edge,2)-1 : -1 : 1
                       %    left_20 = [left_20 bug.edge(1:2,k)];
                       %end
                       
                       visible_edge_points = bug.edge;
                       
                       d = bug.goal-robot;
                       da = round(atand(d(2)/d(1)));  % da= angle in deg between robot and goal

                       if d(1) < 0 && d(2) < 0   % goal is bottom left
                           da = da + 180;
                           quad = 3;
                       elseif d(1) > 0 && d(2) >= 0   % goal is top right
                           %da = da;
                           quad = 1;
                       elseif d(1) >= 0 && d(2) < 0   % goal is bottom right
                           %da = da;
                           quad = 4;
                       elseif d(1) < 0 && d(2) >= 0   % goal is top left
                           da = da + 180;
                           quad = 2;
                       else
                           %nada
                       end
                       
                       %vis = 0;
                       % vis=0 is no 
                       
                       if quad == 1
                           % floor x
                           % floor y
                           i = 2;
                           while i < size(visible_edge_points,2) -1
                               
                               d = visible_edge_points(1:2,i) - robot;
                               da = atand(d(2)/d(1));
                               
                               if d(1) < 0 && d(2) < 0
                                   da = da + 180;
                               elseif d(1) > 0 && d(2) > 0
                                   %da = da;
                               elseif d(1) >= 0 && d(2) < 0
                                   %da = da;
                               elseif d(1) < 0 && d(2) >= 0
                                   da = da + 180;
                               else
                                   %nada
                               end
                               
                               for q=1 : rad
                                   if robot(1) + floor(q*cosd(da)) == visible_edge_points(1:1,i) && ...
                                        robot(2) + floor(q*sind(da)) == visible_edge_points(2:2,i)
                                       break;
                                   end
                                   if bug.isoccupied([robot(1) + floor(q*cosd(da)), robot(2) + floor(q*sind(da))])
                                       %dedge(:,z) = [];
                                       visible_edge_points(:,i) = [];
                                       i = i - 1;
                                       break;
                                   end
                                   if q == 20
                                       visible_edge_points(:,i) = [];
                                       i = i - 1;
                                       break;
                                   end
                                   %i = i + 1;
                               end
                               i = i + 1;
                           end
                           
                       elseif quad == 2
                           % roof  x
                           % floor y
                           
                           i = 2;
                           while i < size(visible_edge_points,2)-1
                               
                               d = visible_edge_points(1:2,i) - robot;
                               da = atand(d(2)/d(1));
                               
                               if d(1) < 0 && d(2) < 0
                                   da = da + 180;
                               elseif d(1) > 0 && d(2) > 0
                                   %da = da;
                               elseif d(1) >= 0 && d(2) < 0
                                   %da = da;
                               elseif d(1) < 0 && d(2) >= 0
                                   da = da + 180;
                               else
                                   %nada
                               end
                               
                               for q=1 : rad
                                   if robot(1) + ceil(q*cosd(da)) == visible_edge_points(1:1,i) && ...
                                        robot(2) + floor(q*sind(da)) == visible_edge_points(2:2,i)
                                       break;
                                   end
                                   if bug.isoccupied([robot(1) + ceil(q*cosd(da)), robot(2) + floor(q*sind(da))])
                                       %dedge(:,z) = [];
                                       visible_edge_points(:,i) = [];
                                       i = i - 1;
                                       break;
                                   end
                                   if q == 20
                                       visible_edge_points(:,i) = [];
                                       i = i - 1;
                                       break;
                                   end
                                   %i = i + 1;
                               end
                               i = i + 1;
                           end
                           
                       elseif quad == 3
                           % roof  x
                           % roof  y
                           
                           i = 2;
                           while i < size(visible_edge_points,2)-1
                               
                               d = visible_edge_points(1:2,i) - robot;
                               da = atand(d(2)/d(1));
                               
                               if d(1) < 0 && d(2) < 0
                                   da = da + 180;
                               elseif d(1) > 0 && d(2) > 0
                                   %da = da;
                               elseif d(1) >= 0 && d(2) < 0
                                   %da = da;
                               elseif d(1) < 0 && d(2) >= 0
                                   da = da + 180;
                               else
                                   %nada
                               end
                               
                               for q=1 : rad
                                   if robot(1) + ceil(q*cosd(da)) == visible_edge_points(1:1,i) && ...
                                           robot(2) + ceil(q*sind(da)) == visible_edge_points(2:2,i)
                                       break;
                                   end
                                   if bug.isoccupied([robot(1) + ceil(q*cosd(da)), robot(2) + ceil(q*sind(da))])
                                       %dedge(:,z) = [];
                                       visible_edge_points(:,i) = [];
                                       i = i - 1;
                                       break;
                                   end
                                   if q == 20
                                       visible_edge_points(:,i) = [];
                                       i = i - 1;
                                       break;
                                   end
                                   %i = i + 1;
                               end
                               i = i + 1;
                           end
                           
                       elseif quad == 4
                           % floor x
                           % roof  y
                           
                           i = 2;
                           while i < size(visible_edge_points,2)-1
                               
                               d = visible_edge_points(1:2,i) - robot;
                               da = atand(d(2)/d(1));
                               
                               if d(1) < 0 && d(2) < 0
                                   da = da + 180;
                               elseif d(1) > 0 && d(2) > 0
                                   %da = da;
                               elseif d(1) >= 0 && d(2) < 0
                                   %da = da;
                               elseif d(1) < 0 && d(2) >= 0
                                   da = da + 180;
                               else
                                   %nada
                               end
                               
                               for q=1 : rad
                                   if robot(1) + floor(q*cosd(da)) == visible_edge_points(1:1,i) && ...
                                           robot(2) + ceil(q*sind(da)) == visible_edge_points(2:2,i)
                                       break;
                                   end
                                   if bug.isoccupied([robot(1) + floor(q*cosd(da)), robot(2) + ceil(q*sind(da))])
                                       %dedge(:,z) = [];
                                       visible_edge_points(:,i) = [];
                                       i = i - 1;
                                       break;
                                   end
                                   if q == 20
                                       visible_edge_points(:,i) = [];
                                       i = i - 1;
                                       break;
                                   end
                                   %i = i + 1;
                               end
                               i = i + 1;
                           end
                           
                       else
                           % some kinda error
                       end
                       
                       % so now right_20 and left_20 have the visible
                       % points on the object robot is next to.
                       % need to add those points to dedge and then remove
                       % any duplicates.
                       
                       
                       %dedge = [dedge right_20];
                       %dedge = [dedge left_20];
                       
                       
             %          dedge = [dedge visible_edge_points];
                       
             %          dedge = unique(dedge.','rows','stable').';
                       
                       %dedge = union(dedge.',visible_edge_points,'rows', 'stable');
                       
                       % need to combine dedge and visible_edge_points. 
                       % remove all points in dedge that are also in
                       % visible_edge_points.
                       % then append visible_edge_points to dedge.
                       
                       i=1;
                       while i < size(dedge,2)
                           for q=1 : size(visible_edge_points, 2)
                               w = dedge(1:2,i);
                               u = visible_edge_points(1:2,q);
                               if w(1) == u(1) && w(2) == u(2)
                                   dedge(:,i) = [];
                                   i = i - 1;
                                   break;
                               end
                               %i = i + 1;
                           end
                           i = i + 1;
                       end
                       
                       visible_edge_points = unique(visible_edge_points.','rows').';
                       
                       dedge = [dedge visible_edge_points];
                       
                       dedge = unique(dedge.','rows','stable').';
                                   
                       
                    end
                    
                    
                    
                    
                    
                    
                    
                    bug.obj = 1;
                    
                    % make it just a single tangent point, find the closest
                    % point on dedge.
                    
                    dedge_size = size(dedge, 2);
                    
                    tangentPoints = [;];
                    c=0;
                    for r=1 : dedge_size
                        c=0;
                        for e=1 : dedge_size
                            if norm( dedge(1:2,r) - dedge(1:2,e) ) <= 1
                                c = c+1;
                            end
                        end
                        if c <= 2
                            tangentPoints = [tangentPoints dedge(1:2,r)];
                        end
                    end
                    
                    
                    
                    
                    
                    %tangentFinal = dedge(1:2,1);
                    
                    %tangentPoints = tangentFinal;
                    
                    %tangentPoints = [tangentPoints dedge(1:2,dedge_size)];
                    
%                     for q=1 : dedge_size
%                         tl = dedge(1:2,q); % tangent line point
%                         td = norm(bug.goal - tl); % distance from tl to goal
%                         if td < norm(bug.goal - tangentFinal)
%                             tangentFinal = tl;
%                         end
%                     end
                    
                    %r = 1; % dedge index
%                     for r=1 : dedge_size -1
%                         if norm( dedge(1:2,r) - dedge(1:2,(r+1)) ) > 1.5
%                             tangentPoints = [tangentPoints dedge(1:2,r)];
%                             tangentPoints = [tangentPoints dedge(1:2,(r+1))];
%                         end
%                     end
                    
                    tangentPoints = unique(tangentPoints.','rows','stable').';
                    tangentFinal = tangentPoints(1:2,1);
                    
                    %get robots current pos out of tangentPoints
                    for i=1 : size(tangentPoints,2)
                        p = tangentPoints(1:2,i);
                        if p(1) == robot(1) && p(2) == robot(2)
                            tangentPoints(:,i) = [];
                            break;
                        end
                    end
                    
%                     tangentPoints = unique(tangentPoints.','rows').'; % sort by x value
%                     for r=1 : tangentPoints -1
%                         if norm( tangentPoints(1:2,r) - tangentPoints(1:2,(r+1)) ) > 1.5
%                             tangentPoints = [tangentPoints dedge(1:2,r)];
%                             tangentPoints = [tangentPoints dedge(1:2,(r+1))];
%                         end
%                     end
%                     
%                     %tangentPoints = unique(tangentPoints.','rows','stable').';
%                     
%                     tangentPoints = sortrows(tangentPoints',2)'; % sort by y value
%                     for r=1 : tangentPoints -1
%                         if norm( tangentPoints(1:2,r) - tangentPoints(1:2,(r+1)) ) > 1.5
%                             tangentPoints = [tangentPoints dedge(1:2,r)];
%                             tangentPoints = [tangentPoints dedge(1:2,(r+1))];
%                         end
%                     end
%                     
%                     tangentPoints = unique(tangentPoints.','rows','stable').';
                    
                    for e=2 : size(tangentPoints, 2)
                        if norm( bug.goal - tangentPoints(1:2,e) ) < norm( bug.goal - tangentFinal )
                            tangentFinal = tangentPoints(1:2,e);
                        elseif norm( bug.goal - tangentPoints(1:2,e) ) == norm( bug.goal - tangentFinal )
                            %tangentFinal = [tangentFinal tangentPoints(1:2,e)];
                            %n = robot + [dx;dy];
                            %return;
                            %bug.g = 1;
                            %n = robot;
                            %return;

                            bug.step = 4;
                            n = robot;
                            return;


%                             m = norm( robot - tangentPoints(1:2,e) );
%                             n = norm( robot - tangentFinal);
%                             
%                             if m == n
%                                 bug.step = 4;
%                                 n = robot;
%                                 return;
%                             elseif m < n
%                                 bug.tang = tangentPoints(1:2,e);
%                                 bug.step = 3;
%                                 n = robot;
%                                 return;
%                             elseif m > n
%                                 bug.tang = tangentFinal;
%                                 bug.step = 3;
%                                 n = robot;
%                                 return;
%                             else
%                                 %shouldn't happen
%                                 bug.step = 4;
%                                 bug.tang = tangentFinal;
%                                 n = robot;
%                                 return;
%                             end
                        else
                            % nada
                        end
                    end
                    
%                     if size(tangentFinal, 2) > 1 && bug.g == 1
%                         a = tangentFinal(1:2,1);
%                         b = tangentFinal(1:2,2);
%                         if a(1) + a(2) > b(1) + b(2)
%                             tangentFinal = a;
%                             bug.g = 0;
%                         elseif a(1) + a(2) < b(1) + b(2)
%                             tangentFinal = b;
%                             bug.g = 0;
%                         else
%                             tangentFinal = bug.goal;
%                             bug.g = 0;
%                         end
%                     end
%                     
%                     if bug.g == 1
%                         n = robot;
%                         return;
%                     end

                    
                    bug.mline = homline(robot(1), robot(2), ...
                        tangentFinal(1), tangentFinal(2));
                    bug.mline = bug.mline / norm(bug.mline(1:2));
                    
                    %plot_mline(bug);
                    
                    dx = 0;
                    dy = 0;
                    d = [];
                    
                    d = tangentFinal - robot;
                    if abs(d(1)) > abs(d(2))
                        % line slope less than 45 deg
                        dx = sign(d(1));
                        L = bug.mline;
                        y = -( (robot(1)+dx)*L(1) + L(3) ) / L(2);
                        dy = round(y - robot(2));
                    else
                        % line slope greater than 45 deg
                        dy = sign(d(2));
                        L = bug.mline;
                        x = -( (robot(2)+dy)*L(2) + L(3) ) / L(1);
                        dx = round(x - robot(1));
                    end
                    
                    if bug.isoccupied(robot + [dx; dy])
                        bug.step = 2;
                        bug.H(bug.j,:) = robot; % define hit point
                        bug.edge = edgelist(bug.occgridnav == 0, robot);
                        bug.k = 2;  % skip the first edge point, we are already there
                        n = robot + [0; 0];
                        return;
                    end
                        
                    
                    n = robot + [dx; dy];
                    return;
                    
                    %dedge_size = size(dedge, 2);
                    %t1 = dedge(1:2,1); % left tangent point
                    %t2 = dedge(1:2,dedge_size); % right tangent point
                    
                    %d1 = norm(bug.goal - t1); % distance from t1 to goal
                    %d2 = norm(bug.goal - t2); % distance from t2 to goal
                    
%                     if d1 > d2
%                         
%                         bug.mline = homline(robot(1), robot(2), ...
%                             t2(1), t2(2));
%                         bug.mline = bug.mline / norm(bug.mline(1:2));
%                         
%                         d = t2-robot;
%                         if abs(d(1)) > abs(d(2))
%                             % line slope less than 45 deg
%                             dx = sign(d(1));
%                             L = bug.mline;
%                             y = -( (robot(1)+dx)*L(1) + L(3) ) / L(2);
%                             dy = round(y - robot(2));
%                         else
%                             % line slope greater than 45 deg
%                             dy = sign(d(2));
%                             L = bug.mline;
%                             x = -( (robot(2)+dy)*L(2) + L(3) ) / L(1);
%                             dx = round(x - robot(1));
%                         end
%                    if dx > 1
%                        dx = 1;
%                    end
%                    if dy > 1
%                        dy = 1;
%                    end

%                        n = robot + [dx; dy];
                        
                    %end
                    
%                     elseif d2 > d1
%                         
%                         bug.mline = homline(robot(1), robot(2), ...
%                             t1(1), t1(2));
%                         bug.mline = bug.mline / norm(bug.mline(1:2));
%                         
%                         d = t1-robot;
%                         if abs(d(1)) > abs(d(2))
%                             % line slope less than 45 deg
%                             dx = sign(d(1));
%                             L = bug.mline;
%                             y = -( (robot(1)+dx)*L(1) + L(3) ) / L(2);
%                             dy = round(y - robot(2));
%                         else
%                             % line slope greater than 45 deg
%                             dy = sign(d(2));
%                             L = bug.mline;
%                             x = -( (robot(2)+dy)*L(2) + L(3) ) / L(1);
%                             dx = round(x - robot(1));
%                         end

%                    if dx > 1
%                        dx = 1;
%                    end
%                    if dy > 1
%                        dy = 1;
%                    end                        

%                        n = robot + [dx; dy];
                    
%                    else
%                        error('Something went wrong');
                        
%                    end
                end
                
%                % detect if next step is an obstacle
%                if bug.isoccupied(robot + [dx; dy])
%                    bug.message('(%d,%d) obstacle!', n);
%                    bug.H(bug.j,:) = robot; % define hit point
%                    bug.step = 2;
%                    % get a list of all the points around the obstacle
%                    bug.edge = edgelist(bug.occgridnav == 0, robot);
%                    bug.k = 2;  % skip the first edge point, we are already there
%                else
%                    n = robot + [dx; dy];
%                end

            end % step 1

            if bug.step == 2
                % Step 2.  Move around the obstacle until we reach a point
                % on the M-line closer than when we started.
                if colnorm(bug.goal-robot) == 0 % are we there yet?
                    return
                end
                
                
                bug.mline = homline(robot(1), robot(2), ...
                    bug.goal(1), bug.goal(2));
                bug.mline = bug.mline / norm(bug.mline(1:2));
                
                % motion on line toward goal
                d = bug.goal-robot;
                if abs(d(1)) > abs(d(2))
                    % line slope less than 45 deg
                    dx = sign(d(1));
                    L = bug.mline;
                    y = -( (robot(1)+dx)*L(1) + L(3) ) / L(2);
                    dy = round(y - robot(2));
                else
                    % line slope greater than 45 deg
                    dy = sign(d(2));
                    L = bug.mline;
                    x = -( (robot(2)+dy)*L(2) + L(3) ) / L(1);
                    dx = round(x - robot(1));
                end
                
                % Detect if there is object between robot and goal
                da = round(atand(d(2)/d(1)));  % da= angle in deg between robot and goal
%                 if da < 0
%                     da = da * -1;
%                 end

                if d(1) < 0 && d(2) < 0   % goal is bottom left
                    da = da + 180;
                elseif d(1) > 0 && d(2) >= 0   % goal is top right
                    %da = da;
                elseif d(1) >= 0 && d(2) < 0   % goal is bottom right
                    %da = da;
                elseif d(1) < 0 && d(2) >= 0   % goal is top left
                    da = da + 180;
                else
                    %nada
                end

                rad = 2;
                blocker = 0;   % 0 for no object between robot and one step toward goal
                for q=1 : rad
                    c = [robot(1) + round(q*cosd(da)); robot(2) + round(q*sind(da))];
                    if c(1) == bug.goal(1) && c(2) == bug.goal(2)
                        break;
                    end
                    if norm(bug.goal - robot) < norm(c - robot)
                        break;
                    end
                    if bug.isoccupied([robot(1) + round(q*cosd(da)), robot(2) + round(q*sind(da))])
                        blocker = 1;
                        break;
                    end
                    if bug.isoccupied(robot + [dx; dy])
                        blocker = 1;
                        break;
                    end
                end               
                
                if blocker == 0
                    bug.step = 1;
                    bug.cd = 2;
                    n = robot + [dx;dy];
                    return;
                end
                
                if blocker == 1
                    
                    % need to circle object left or right
                    % currently always goes right
                    % should check if right step or left step brings closer
                    % to goal, then circle that way.
                    % bug.cd 0=circle right
                    % 1=circle left
                    % 2= decide on left or right
                    
                    if bug.cd == 2
                        left_step = bug.edge(1:2,(size(bug.edge, 2))-1 );
                        right_step = bug.edge(1:2,2);

                        if norm(bug.goal - left_step) > norm(bug.goal - right_step)
                            % circle right
                            bug.cd = 0;
                            bug.k = 2;
                        elseif norm(bug.goal - left_step) < norm(bug.goal - right_step)
                            % circle left
                            bug.cd = 1;
                            bug.k = size(bug.edge,2) - 1;
                        else
                            % error or rare case where both steps are ==
                            % distance from goal.
                            bug.cd = 0;
                            bug.k = 2;
                        end
                    end
                    
                    if bug.cd == 0
                        % circle right                  
                        n = bug.edge(:,bug.k);  % next edge point
                        bug.k = bug.k+1;
                        return;
                    end
                    
                    if bug.cd == 1
                        % circle left
                        n = bug.edge(:,bug.k);
                        bug.k = bug.k-1;
                        return;
                    end
                    
                    % Should be impossible to reach here, but just in case
                    %n = bug.edge(:,bug.k);  % next edge point
                    %bug.k = bug.k+1;
                    n = robot;
                    return;
                    
                end % end of 'if blocker == 1' in bug.step=2
                
                
                
                
                
                
                
                %bug.step = 1;
                
%                bug.mline = homline(robot(1), robot(2), ...
%                    bug.goal(1), bug.goal(2));
%                bug.mline = bug.mline / norm(bug.mline(1:2));
                
%                plot_mline(bug);

%                if bug.k <= numcols(bug.edge)
%                    n = bug.edge(:,bug.k);  % next edge point
%                else
%                    % we are at the end of the list of edge points, we
%                    % are back where we started.  Step 2.c test.
%                    error('robot is trapped')
%                    return;
%                end

%                % are we on the M-line now ?
%                if abs( [robot' 1]*bug.mline') <= 0.5
%                    bug.message('(%d,%d) moving along the M-line', n);
%                    % are closer than when we encountered the obstacle?
%                    if colnorm(robot-bug.goal) < colnorm(bug.H(bug.j,:)'-bug.goal)
%                        % back to moving along the M-line
%                        bug.j = bug.j + 1;
%                        bug.step = 1;
%                        return;
%                    end
%                end
%                % no, keep going around
%                bug.message('(%d,%d) keep moving around obstacle', n)
%                bug.k = bug.k+1;
            end % step 2
            
            if bug.step == 3
                % moving around an object while currently not touching it
                % find closest tangent and save it as bug.tang
                % go to that point and then enter step 2
                
                bug.mline = homline(robot(1), robot(2), ...
                    bug.tang(1), bug.tang(2));
                bug.mline = bug.mline / norm(bug.mline(1:2));
                
                d = bug.tang - robot;
                if abs(d(1)) > abs(d(2))
                    % line slope less than 45 deg
                    dx = sign(d(1));
                    L = bug.mline;
                    y = -( (robot(1)+dx)*L(1) + L(3) ) / L(2);
                    dy = round(y - robot(2));
                    %bug.cd = 0;
                else
                    % line slope greater than 45 deg
                    dy = sign(d(2));
                    L = bug.mline;
                    x = -( (robot(2)+dy)*L(2) + L(3) ) / L(1);
                    dx = round(x - robot(1));
                    %bug.cd = 1;
                    %bug.k = size(bug.edge,2) - 1;
                end
                
                if dx == 1 && dy == 1
                    bug.cd = 1; % circle left
                    bug.k = size(bug.edge,2) - 1;
                elseif dx == 1 && dy == -1
                    bug.cd = 0; % circle right
                elseif dx == -1 && dy == 1
                    bug.cd = 0; % circle right
                    bug.k = size(bug.edge,2) - 1;
                elseif dx == -1 && dy == -1
                    bug.cd = 1; % circle left
                else
                    % nothing?
                end
                
                % detect if next step is an obstacle
                if bug.isoccupied(robot + [dx; dy])
                    bug.message('(%d,%d) obstacle!', n);
                    bug.H(bug.j,:) = robot; % define hit point
                    bug.step = 2;
                    % get a list of all the points around the obstacle
                    bug.edge = edgelist(bug.occgridnav == 0, robot);
                    %bug.k = 2;  % skip the first edge point, we are already there
                    %if bug.cd == 1
                    %    bug.k = size(bug.edge,2) - 1;
                    %else
                    %    bug.k = 2;
                    
                    if bug.cd == 0
                        bug.k = 2;
                    elseif bug.cd == 1
                        bug.k = size(bug.edge,2) - 1;
                    else
                        bug.k = 2;
                    end
                    
                    %end
                    
                    %bug.step = 1;
                    %bug.cd = 2;
                    
                    n = robot;
                    return;
                else
                    n = robot + [dx; dy];
                end
                
                
                
            end % end step 3
            
            if bug.step == 4
                % have 2 tangent points equal distance.
                % so take single step straight at goal to maybe determine
                % if one best tangent point can be found.
                
                bug.mline = homline(bug.start(1), bug.start(2), ...
                    bug.goal(1), bug.goal(2));
                bug.mline = bug.mline / norm(bug.mline(1:2));

                % motion on line toward goal
                d = bug.goal-robot;
                if abs(d(1)) > abs(d(2))
                    % line slope less than 45 deg
                    dx = sign(d(1));
                    L = bug.mline;
                    y = -( (robot(1)+dx)*L(1) + L(3) ) / L(2);
                    dy = round(y - robot(2));
                else
                    % line slope greater than 45 deg
                    dy = sign(d(2));
                    L = bug.mline;
                    x = -( (robot(2)+dy)*L(2) + L(3) ) / L(1);
                    dx = round(x - robot(1));
                end
                
                da = round(atand(d(2)/d(1)));  % da= angle in deg between robot and goal
                if d(1) < 0 && d(2) < 0   % goal is bottom left
                    da = da + 180;
                elseif d(1) > 0 && d(2) > 0   % goal is top right
                    %da = da;
                elseif d(1) >= 0 && d(2) < 0   % goal is bottom right
                    %da = da;
                elseif d(1) < 0 && d(2) >= 0   % goal is top left
                    da = da + 180;
                else
                    %nada
                end
                
                if da == 0 || da == 360
                    dx = 1;
                    dy = 0;
                elseif da == 90 || da == -270
                    dx = 0;
                    dy = 1;
                elseif da == 180 || da == -180
                    dx = -1;
                    dy = 0;
                elseif da == -90 || da == 270
                    dx = 0;
                    dy = 1;
                elseif 0 < da && da < 90
                    dx = 1;
                    dy = 1;
                elseif 90 < da && da < 180
                    dx = -1;
                    dy = 1;
                elseif (180 < da && da < 270) || (-180 < da && da < -90)
                    dx = -1;
                    dy = -1;
                elseif (270 < da && da < 360) || (-90 < da && da < 0)
                    dx = 1;
                    dy = -1;
                else
                    % error?
                end
                
               % detect if next step is an obstacle
               if bug.isoccupied(robot + [dx; dy])
                   bug.message('(%d,%d) obstacle!', n);
                   bug.H(bug.j,:) = robot; % define hit point
                   bug.step = 2;
                   % get a list of all the points around the obstacle
                   bug.edge = edgelist(bug.occgridnav == 0, robot);
                   bug.k = 2;  % skip the first edge point, we are already there
                   n = robot;
                   return;
               else
                   bug.step = 1;
                   n = robot + [dx; dy];
                   return;
               end
                
                
            end % end bug.step == 4
            
            
        end % next
        
        function plan(bug)
            error('RTB:Bug2:badcall', 'This class has no plan method');
        end

    end % methods
end % classdef