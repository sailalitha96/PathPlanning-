function array = remove(array, e)
   if sum(ismember(array, e)) == 0
       fprintf('element does not exist!')
       return
   else
       idx = find(array == e);
       array = vertcat(array(1:idx-1),array(idx+1:end));
   end
end
        
        