function Nodes = updateChildrenCost(q,root, Nodes)
   if length(q.child) == 1  
        q.cost = newCost(q, root, Nodes);
        return
   else 
       q.cost = newCost(q, root, Nodes);
       for i=2:length(q.child)
          Nodes = updateChildrenCost(Nodes(q.child(i)), root, Nodes);
       end
   end
end
