
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

//returns point on the curve
float Bezier(float x, float p0,float p1,float p2,float p3){
  return (1-x)*(1-x)*(1-x)*p0+3*(1-x)*(1-x)*x*p1+3*(1-x)*x*x*p2+x*x*x*p3;
}
float BezierD(float x, float p0,float p1,float p2,float p3){
  return (-3*(1-x)*(1-x)*p0 +(3-12*x+9*x*x)*p1 +(6*x-9*x*x)*p2 +3*x*x*p3);
}

//constraints of leg individual coordinates
float JerkMax=32*PI;
float Vmax=3*PI;
float Amax=JerkMax/2;
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
  
  float[][] BezierLength=new float[4][PathControlMaxPoints-1];
  float[] PathLength={0,0,0,0};
  float[][] X=new float[4][PathControlMaxPoints];
  
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
      P1[t][0]=(path[t][0]+2*path[t][1]-P1[t][1])/2;

      for (int i=0; i<len-2; i++) {
        P2[t][i]=2*P[t][i+1]-P1[t][i+1];
      }
      P2[t][len-2]=P[t][len-1];
    }
    
    //arc length of each curve
    for(int t=0; t<4; t++){
      for(int i=0; i<Plength-1; i++){
        BezierLength[t][i]=0;
        for(float x=0; x<=1; x+=0.01){
          BezierLength[t][i]+=(abs(BezierD(x-0.01,P[t][i],P1[t][i],P2[t][i],P[t][i+1])));//-Bezier(x,P[t][i],P1[t][i],P2[t][i],P[t][i+1]));
        }
        BezierLength[t][i]=1;
        PathLength[t]+=abs(BezierLength[t][i]);
      }
      X[t][0]=0;
      for(int i=1; i<Plength-1; i++){
        X[t][i]=X[t][i-1]+BezierLength[t][i-1]/PathLength[t];
      }
      X[t][Plength-1]=1;
    }
    
    
    //find roots of B'(x)

  }
  
  void display(){
    stroke(0);
    //xy plane
    for (int i=0; i<Plength-1; i++) {
      for (float t=0; t<1; t+=0.05) {
        fill(255*(1-t), 255*t, 0);
        circle(Bezier(t,P[0][i],P1[0][i],P2[0][i],P[0][i+1]),Bezier(t,P[1][i],P1[1][i],P2[1][i],P[1][i+1]),4);
        line(Bezier(t,P[0][i],P1[0][i],P2[0][i],P[0][i+1]),Bezier(t,P[1][i],P1[1][i],P2[1][i],P[1][i+1]),
             Bezier(t,P[0][i],P1[0][i],P2[0][i],P[0][i+1])+BezierD(t,P[0][i],P1[0][i],P2[0][i],P[0][i+1])
             ,Bezier(t,P[1][i],P1[1][i],P2[1][i],P[1][i+1]));
      }
    }
    /*for(float x=0; x<1; x+=0.01){
      int i=0;
      while(x>X[0][i+1]){ i++;}
      float t=(x-X[0][i])/(X[0][i+1]-X[0][i]);
      fill(255*(1-t), 255*t, 0);
      circle(Bezier(t,P[0][i],P1[0][i],P2[0][i],P[0][i+1]),Bezier(t,P[1][i],P1[1][i],P2[1][i],P[1][i+1]),4);
      line(Bezier(t,P[0][i],P1[0][i],P2[0][i],P[0][i+1]),Bezier(t,P[1][i],P1[1][i],P2[1][i],P[1][i+1]),
           Bezier(t,P[0][i],P1[0][i],P2[0][i],P[0][i+1])+BezierD(t,P[0][i],P1[0][i],P2[0][i],P[0][i+1])
           ,Bezier(t,P[1][i],P1[1][i],P2[1][i],P[1][i+1]));
    }*/
  }
  
  //update the target position and velocity at given time
  void Update(float time){
    if(time<T){
      /*int i=0;
      while(time/T>X[0][i+1]){ i++;}
      float x=(time/T-X[0][i])/(X[0][i+1]-X[0][i]);
      for(int t=0; t<4;t++){
        TargetPos[t]=Bezier(x,P[t][i],P1[t][i],P2[t][i],P[t][i+1]);
        TargetVel[t]=BezierD(x,P[t][i],P1[t][i],P2[t][i],P[t][i+1])/(T/Plength);
      }*/
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


float[][] path={{0,-200,-300,-100,100,300,280,250,250,200},{0,50,100,150,200,250,300,350,400,450,500},{0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0}};
PathControl C = new PathControl(path, 10, 15);
float FPS=60;
float time=0;

float px=0; float py=0;

void setup() {
  size(1200, 800);
  background(255);
  frameRate(FPS);

}

void draw() {
  translate(600, 50);
  background(255);
  time+=1/FPS;
  
  fill(255, 0, 0);
  //xy plane
  circle(path[0][0], path[1][0], 15);
  fill(0, 0, 255);
  for (int i=1; i<8; i++) {
    circle(path[0][i], path[1][i], 10);
    line(path[0][i], path[1][i],path[0][i-1], path[1][i-1]);
  }
  C.display();
  
  
  C.Update(time);
  px=(px+C.TargetVel[0]/FPS)*0.8+C.TargetPos[0]*0.2;
  py=(py+C.TargetVel[1]/FPS)*0.8+C.TargetPos[1]*0.2;
  circle(px,py,10);
  
}
