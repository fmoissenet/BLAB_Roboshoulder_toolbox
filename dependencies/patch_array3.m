function mesh = patch_array3(faces,vertices,facecolor,edgecolor,facelighting,facealpha)

mesh = patch('Faces',permute(faces,[2,1,3]),...
             'Vertices',permute(vertices,[2,1,3]),...
             'FaceColor',facecolor,...
             'EdgeColor',edgecolor,...
             'FaceLighting',facelighting,...
             'FaceAlpha',facealpha);
  
  
