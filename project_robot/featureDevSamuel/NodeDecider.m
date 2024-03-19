start = Node('start');
home = Node('home');
line = Node('line');
wall = Node('wall');
odo = Node('odo');
lstop = Node('lstop');
redStop = Node('redStop');
blueStop = Node('blueStop');

% Child-parent relationships
start.addChild(home);
home.addParent(start);
home.addChild(line);
home.addChild(wall);
home.addChild(odo);
line.addParent(home);
line.addChild(lstop);
lstop.addParent(line);
odo.addParent(home);
odo.addChild(redStop);
redStop.addParent(odo)
odo.addChild(blueStop);
blueStop.addParent(odo);
wall.addParent(home);

% Sibling Relationships
line.addSibling(odo, 'r');
odo.addSibling(line, 'l');
odo.addSibling(wall, 'r');
wall.addSibling(odo, 'l');
redStop.addSibling(blueStop, 'l');
redStop.addSibling(home, 'r');
blueStop.addSibling(redStop, 'r');
line.addSibling(start,'l');
wall.addSibling(start,'r');


currentNode = start;
tic
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
        currentNode = gotoOptions{text};
    else
        fprintf("Invalid entry, please try again.\n");
    end
end

%% X. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
clear all


