function T = transl(tx,ty,tz)

T = [eye(3),[tx,ty,tz]';0 0 0 1];