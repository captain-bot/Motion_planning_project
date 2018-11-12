classdef mytree
    properties %(GetAccess='public', SetAccess='public')
        nodes;           % position and velocity information
        edges;           % node connectivity and costs
        exp_node_ind;    % index of already expanded nodes
        unexp_node_ind;  % index of unexpanded nodes
    end
    methods        
        % Constructor
        function obj=mytree()            
            obj.nodes=[];           
            obj.edges=[];           
            obj.exp_node_ind=[];
            obj.unexp_node_ind=[];
        end
        
        % Add a node to the tree
        function obj = add_node(obj,new_node)
            obj.nodes = [obj.nodes; new_node];
        end
        
        % Add a node to the tree
        function obj = add_edge(obj,new_edge)
            obj.edges = [obj.edges; new_edge];
        end
        
        % Add nodes that are not exoanded yet (indices)
        function obj = update_unexp_node_ind(new_ind)
            obj.unexp_node_ind = [obj.unexp_node_ind, new_ind];
        end
        
        % Updates the list of expanded nodes (indices)
        function obj = update_exp_node_ind(new_ind)
            obj.exp_node_ind = [obj.exp_node_ind, new_ind];
            % Remove this index from the list of unexpanded nodes
            temp_ind = find(obj.unexp_node_ind==new_ind);
            if temp_ind == 1
                obj.unexp_node_ind = obj.unexp_node_ind(2:end);
            elseif temp_ind == length(obj.unexp_node_ind)
                obj.unexp_node_ind = obj.unexp_node_ind(1:end-1);
            else
                obj.unexp_node_ind = [obj.unexp_node_ind(1:temp_ind-1,:)...
                    ,obj.unexp_node_ind(temp_ind+1:end)];
            end
        end
        
%         % Print tree nodes
%         function print_nodes(obj)
%             obj.nodes;
%         end        
    end
end