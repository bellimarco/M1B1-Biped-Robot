
//returns solution to the tridiagonal matrix ai*xi-1 + bixi +cixi+1 =di for i=1...n-1
float[] thomas_algorithm(float[] a, float[] b, float[] c, float[] d, int n) {
  float[] x=new float[n];
  float[] c_=new float[n];
  float[] d_=new float[n];
  c_[0] = c[0] / b[0];
  d_[0] = d[0] / b[0];

  for (int i=1; i<n; i++) {
    float m = 1.0 / (b[i] - a[i] * c_[i-1]);
    c_[i] = c[i] * m;
    d_[i] = (d[i] - a[i] * d_[i-1]) * m;
  }
  x[n-1] = d_[n-1];
  for (int i=n-2; i>0; i--) {
    x[i] = d_[i] - c_[i] * x[i+1];
  }
  return x;
}

//Bezier functions of x
float Bezier(float x, float p0,float p1,float p2,float p3){
  return (1-x)*(1-x)*(1-x)*p0+3*(1-x)*(1-x)*x*p1+3*(1-x)*x*x*p2+x*x*x*p3;
}
float BezierD(float x, float p0,float p1,float p2,float p3){
  return (-3*(1-x)*(1-x)*p0 +(3-12*x+9*x*x)*p1 +(6*x-9*x*x)*p2 +3*x*x*p3);
}



//given a path and a time deadline, this class generates a bezier curve along the path,
//  with start and end velocities = 0 and evenly distributed time between the points(each movement
//  takes T/pathlength seconds to complete), this means that if a path of constant average velocity is
//  desired, its up to the path generation to space out evenly the points
//with all the bezier curves stored(as arrays of control points, the Update function can calculate the
//  target position and velocity at any proportion time/T ([0:1]) of the path, and store theeses values
//  in the class object for any other process to read
//note that every coordinate is calculated separetely and has its own bezier curve, i.e. the amount of
//  floats stored in the object is dimensions*pathlength*3+sidevariables
//note also that the curve generation doesn't check for any bounderies in the position, velocity or
//  acceleration, which means its again up to the pathgenerator to create conservative path waypoints
//  and T (since the bezier curve for ex. overshoots/undershoots on rough edges)

int PathControlMaxPoints=20;

class PathControl {

  boolean finished=true;
  //0-1 completeness proportion
  float completed=0.0;
  //block execution time deadline
  float T;

  //from initial to final point, theta coordinates
  float[][] P=new float[4][PathControlMaxPoints];
  int Plength=0;

  //bezier control points
  float[][] P1=new float[4][PathControlMaxPoints-1];
  float[][] P2=new float[4][PathControlMaxPoints-1];
  
  //updated in time, read from the main PID controller
  float[] TargetPos={0,0,0,0};
  float[] TargetVel={0,0,0,0};

  PathControl(float[][] p, int len, float time) {
    completed=0.0; 
    finished=false;
    T=time;
    P = p; 
    Plength = min(len, PathControlMaxPoints);

    //P0 and P3 of the bezier curves are already stored in the P array
    //now calculating the P1 points with "the algorithm"
    float[] a=new float[len-1];
    float[] b=new float[len-1];
    float[] c=new float[len-1];
    float[] d=new float[len-1];
    for (int t=0; t<4; t++) {
      b[0]=2;
      c[0]=1; 
      d[0]=P[t][0]+2*P[t][1];
      for (int i=1; i<len-2; i++) {
        a[i]=1; 
        b[i]=4; 
        c[i]=1; 
        d[i]=4*P[t][i]+2*P[t][i+1];
      }
      a[len-2]=2;
      b[len-2]=7; 
      d[len-2]=8*P[t][len-2]+P[t][len-1];
      P1[t]=thomas_algorithm(a, b, c, d, len-1);
      P1[t][0]=(P[t][0]+2*P[t][1]-P1[t][1])/2;

      for (int i=0; i<len-2; i++) {
        P2[t][i]=2*P[t][i+1]-P1[t][i+1];
      }
      P2[t][len-2]=P[t][len-1];
    }
  }
  
  //update the target position and velocity at given time
  void Update(float time){
    if(time<T){
      int i=floor(time*(Plength-1)/T);
      float x=time*(Plength-1)/T-i;
      for(int t=0; t<4;t++){
        TargetPos[t]=Bezier(x,P[t][i],P1[t][i],P2[t][i],P[t][i+1]);
        TargetVel[t]=BezierD(x,P[t][i],P1[t][i],P2[t][i],P[t][i+1])/(T/Plength);
      }
      completed=time/T;
    }else{
      completed=1.0; finished=true;
    }
  }
}
