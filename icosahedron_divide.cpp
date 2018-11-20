

void subdivide(int $n){
  //reset all icosahedron variables
  subfaces = new Face[100000]; points = new Point[100000];
  nsubfaces = 0; npoints = 0;
  //subdivide faces
  for(int i=0;i<20;i++){
    Face[] fs = faces[i].subdivide($n,zero_point);
    //add to our array of subfaces
    if(nsubfaces>0){ subfaces = (Face[]) concat(subfaces,fs); }else{ subfaces = fs; }
    nsubfaces += fs.length;
  }
  //add new points to the icosahedron's list of points
  for(int i=0;i<nsubfaces;i++){
    boolean found = false;
    for(int j=0;j<npoints;j++){
      if(points[j]==subfaces[i].p1){ found = true; break; }
    }
    if(!found){
      points[npoints] = subfaces[i].p1;
      npoints++;
    }
    found = false;
    for(int j=0;j<npoints;j++){
      if(points[j]==subfaces[i].p2){ found = true; break; }
    }
    if(!found){
      points[npoints] = subfaces[i].p2;
      npoints++;
    }
    found = false;
    for(int j=0;j<npoints;j++){
      if(points[j]==subfaces[i].p3){ found = true; break; }
    }
    if(!found){
      points[npoints] = subfaces[i].p3;
      npoints++;
    }
  }
  //project each point to the sphere
  for(int i=0;i<npoints;i++){
    //first get the point's magnitude
    float m = mag(points[i].x,points[i].y,points[i].z);
    //now make its magnitude a unit vector then scale it to the radius in one step
    points[i].x *= radius/m; points[i].y *= radius/m; points[i].z *= radius/m;
  }
}

int main (int argc, char** argv) {

  float tao = 1.61803399;
  float[][] _handles = {{1,tao,0},{-1,tao,0},{1,-tao,0},{-1,-tao,0},
                        {0,1,tao},{0,-1,tao},{0,1,-tao},{0,-1,-tao},
                        {tao,0,1},{-tao,0,1},{tao,0,-1},{-tao,0,-1}};







  return 0;
}
