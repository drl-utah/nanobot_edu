% Initialize sensors
initallSensors(nb);

% Config variables
scaleFactor = 1.1;

% turnTillLine(nb, 7, 1.2, 'r');
% lineFollowing(nb, scaleFactor);
% moveToBar(nb, scaleFactor);
% attemptCenter(nb, scaleFactor);
% turnByAngle(nb, 180);
% attemptCenter(nb, scaleFactor);
% lineFollowing(nb, scaleFactor);

% INITALIZE NODES
start = Node('start');
home = Node('home');
line = Node('line');
wall = Node('wall');
odo = Node('odo');

% Child-parent relationships
start.addChild(home);
home.addParent(start);

home.addChild(line);
home.addChild(wall);
home.addChild(odo);

line.addParent(home);
odo.addParent(home);
wall.addParent(home);

% Sibling Relationships
line.addSibling(odo, 'r');
odo.addSibling(line, 'l');

odo.addSibling(wall, 'r');
wall.addSibling(odo, 'l');

line.addSibling(start,'l');
start.addSibling(line,'r');

wall.addSibling(start,'r');
start.addSibling(wall,'l');

% DEFINE START NODE AND DIRECTIONALITY
currentNode = start;
prevNode = currentNode.parent;

while(1)
    fprintf('Current position: %s\n', currentNode.name);
    fprintf('Go to:');
    parent = currentNode.parent; 
    children = currentNode.children;
    nodeCtr = 0;
    gotoOptions = {};
    
    if(parent ~= 0)
        nodeCtr = nodeCtr + 1;
        gotoOptions{end+1} = currentNode.parent;
        fprintf(' (%d. %s)', nodeCtr, currentNode.parent.name);
    end
    if(size(children, 1) ~= 0)
        for i=1:size(children,2)
            nodeCtr = nodeCtr + 1;
            gotoOptions{end+1} = children{i};
            fprintf(' (%d. %s)', nodeCtr, children{i}.name);
        end
    end
    fprintf('\n');
    text = input('Type the number corresponding to where you want to go\n');
    if((text <= size(gotoOptions,2)) & (text > 0))
        % Target acquired
        targetNode = gotoOptions{text};

        % Turn around before moving only if not at start
        if(prevNode ~= 0)
            turnByAngle(nb, 180); % Previous node wasn't 0, align to previous node path
            attemptCenter(nb, scaleFactor);
        end

        % Based on prevNode's direction, get number of turns l/r to turn to
        % target
        [numTurns,dir] = getNumTurnsTill(prevNode, targetNode);
        timesTurned = 0;
        while(timesTurned < numTurns)
            switch dir
                case 'l'
                    turnTillLine(nb, 9, scaleFactor, 'l');
                case 'r'
                    turnTillLine(nb, 9, scaleFactor, 'r');
                otherwise
                    fprintf("ERROR - getNumTurnsTill, how did we get here?\n");
            end
            timesTurned = timesTurned + 1;
        end

        % Aligned to path to next node, engage line following
        attemptCenter(nb, scaleFactor);
        lineFollowing(nb, scaleFactor);

        % Reached next node, drive forward to center of node bar;
        moveToBar(nb, scaleFactor);

        % Update prev and current nodes
        prevNode = currentNode;
        currentNode = targetNode;

        % Special Node Action handling
        switch currentNode.name
            case 'wall'
                % Do something
                % Approach forward with US reads until below threshold
                % Turn right till curve diverges (no left due to asymmetry)
                % Read wall offset
                % Wall follow with wall offset
                % Read all black
                % Either advance and turn-in-place by 90 deg, or use one
                %   wheel
            case 'odo'
                % Perform odometry by color
                % Implement return functionality
        end
    else
        fprintf("Invalid entry, please try again.\n");
    end
end

%% X. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
clear all


