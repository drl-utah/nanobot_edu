classdef Node < handle
    %NODE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        parent
        children
        name
        Lsib
        Rsib
    end
    
    methods
        function obj = Node(nameStr)
            %NODE Construct an instance of this class
            %   Detailed explanation goes here
            obj.name = nameStr;
            obj.parent = 0;
            obj.children = {};
            obj.Lsib = 0;
            obj.Rsib = 0;
        end

        function addParent(obj, parentNode)
            obj.parent = parentNode;
        end
        
        function addChild(obj, childNode)
            obj.children{end + 1} = childNode;
        end

        function addSibling(obj, sibNode, dir)
            switch dir
                case 'l'
                    obj.Lsib = sibNode;
                case 'r'
                    obj.Rsib = sibNode;
            end
        end
    end
end

