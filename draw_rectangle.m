function[]= draw_rectangle(Cx, Cy,L,H,theta,rgb, id, egoToSensorMatrix, zLidar)
    R= ([cos(theta), -sin(theta); sin(theta), cos(theta)]);
    X=([-L/2, L/2, L/2, -L/2]);
    Y=([-H/2, -H/2, H/2, H/2]);
    for i=1:4
        T(:,i)=R*[X(i); Y(i)];
    end
    x_lower_left=Cx+T(1,1);
    x_lower_right=Cx+T(1,2);
    x_upper_right=Cx+T(1,3);
    x_upper_left=Cx+T(1,4);
    y_lower_left=Cy+T(2,1);
    y_lower_right=Cy+T(2,2);
    y_upper_right=Cy+T(2,3);
    y_upper_left=Cy+T(2,4);

    egoX=[x_lower_left x_lower_right x_upper_right x_upper_left];
    egoY=[y_lower_left y_lower_right y_upper_right y_upper_left];
    transformedX = [];
    transformedY = [];
    for i=1:4
        trans = egoToSensorMatrix*[egoX(i); egoY(i); -zLidar; 1];
        transformedX = [transformedX trans(1)];
        transformedY = [transformedY trans(2)];
    end
    
    patch('Vertices',[transformedX; transformedY]','Faces',[1 2 3 4],'Edgecolor',rgb,'Facecolor','none','Linewidth',1.2);
    textTrans = egoToSensorMatrix*[Cx+5; Cy+5; -zLidar; 1];
    text(textTrans(1), textTrans(2), "Track " + id, 'Color', rgb, 'FontSize', 10);
    axis equal;
end