function y = checkVisibility(x1, y1, x2, y2,lm,nb)
    if( squaredDist(x1,y1,x2,y2)>5*5/4)
        y = false;
        return;
    end
    
    for i=1:length(lm)
        obs_x1 = lm(i,4);
        obs_y1 = lm(i,5);
        dist=point_line_distance(x1,y1,x2,y2,obs_x1,obs_y1);
        if(dist>0 && dist<0.4)
            y = false;
            return
        end
    end

    for i=1:length(nb)
        obs_x1 = nb(i,1);
        obs_x2 = nb(i,2);
        obs_y1 = nb(i,3);
        obs_y2 = nb(i,4);
        
        s1_x = x2 - x1;     s1_y = y2 - y1;
        s2_x = obs_x2 - obs_x1;     s2_y = obs_y2 - obs_y1;

        s = (-s1_y * (x1 - obs_x1) + s1_x * (y1 - obs_y1)) / (-s2_x * s1_y + s1_x * s2_y);
        t = ( s2_x * (y1 - obs_y1) - s2_y * (x1 - obs_x1)) / (-s2_x * s1_y + s1_x * s2_y);
        if (s > 0 && s < 1 && t > 0 && t < 1)
             y = false;
             return;
        end
    y = true;
    end
end

function y=squaredDist(x1,y1,x2,y2)
    y = (x1-x2)*(x1-x2)+(y1-y2)*(y1-y2);
end

function y= point_line_distance(vx, vy, wx, wy, px, py)
  l2 = squaredDist(vx,vy,wx,wy);
  t = ( (px-vx)*(wx-vx)+ (py-vy)*(wy-vy) ) / l2;

  if (t < 0.0) 
      y=sqrt( squaredDist(px,py,vx,vy) );
  else
      if (t > 1.0) 
          y=sqrt( squaredDist(px,py,wx,wy) );
      else
          y=sqrt( squaredDist(px,py, vx+t*(wx-vx), vy+t*(wy-vy)));
      end
  end
end