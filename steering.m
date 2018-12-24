%% steering
function p = steering(p_rand , p_nearest , distance, dthres)

if distance > dthres
    p = p_nearest + (p_rand- p_nearest).* (dthres / distance);
else 
    p = p_rand;
end

end

