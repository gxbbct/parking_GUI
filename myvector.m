classdef myvector < handle
    properties
        data;
        indice;
        dim;
        volum;
    end
    
    methods
        
        function obj = myvector(dim)
            obj.dim = dim;
            obj.indice = 0;
            obj.volum = 16;
            obj.data = zeros(16, dim);
        end
        
        function val = empty(obj)
            val = obj.indice == 0;
        end
        
        function push_back(obj, val)
            if(~ismatrix(val) || size(val, 1) > 1 || size(val, 2) ~= obj.dim)
                error('input error');
            end
            obj.indice = obj.indice + 1;
            if(size(obj.data,1) < obj.indice)
                obj.data = [obj.data; zeros(obj.volum, obj.dim)];
                obj.volum = obj.volum .*2;
            end
            obj.data(obj.indice,:) = val;
        end
        
        function val = mySize(obj)
            val = obj.indice;
        end
        
        function val = back(obj)
            if(obj.indice <=0)
                error('vector is empty');
            end
            val = obj.data(obj.indice,:);
        end
        
        
        function pop_back(obj)
            obj.indice = max(obj.indice - 1, 0);
        end
        
        function clear(obj)
            obj.indice = 0;
            obj.data = zeros(16, obj.dim);
        end
        
        function val = at(obj, row)
            if(row <=0 || obj.indice < row)
                error('input error');
            end
            val = obj.data(row,:);
        end
        
        function val = get_data(obj)
           val = obj.data(1 : obj.indice, :);
        end
        
        function val = get_above(obj, row)
            if(row <=0 || obj.indice < row)
                error('input error');
            end
            val = obj.data(1 : row, :);
        end
    end
end



