function cost = newCost(q, root, Nodes)
    if q.idx == root.idx
        cost = root.cost;
    else
        cost = norm(q.point - Nodes(q.parent).point) + newCost(Nodes(q.parent), root, Nodes);
    end
end
    
