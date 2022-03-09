struct PathPoint {
    float x; // x location in global coordinates
    float y; // y location in global coordinates
    float w; // heading in global coordinates (Head Direction, may known as w omega)
  	float curvature; // curvature of the path at this point
    float distance; // distance (along a straight line) of this point from the beginning of the path
};