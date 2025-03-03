
//random useful vars
float[][] I3={{1,0,0},{0,1,0},{0,0,1}};
float[][] I4={{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
float[] v0_4 = {0,0,0,1};
float[] v0_3 = {0,0,0};
float[] vx = {1,0,0}; float[] vy = {0,1,0}; float[] vz = {0,0,1};


int sgn(float x){return (x<0?-1:1);}


//vectors stuff
void VCopy(float[] a, float[] b){
  a[0] = b[0]; a[1] = b[1]; a[2] = b[2];  
}
void VCopy4(float[] a, float[] b){
  a[0] = b[0]; a[1] = b[1]; a[2] = b[2]; a[3] = b[3];
} 
float[] Vxyrotate(float[] v){
  float[] c={-v[1], v[0], v[2]};
  return c;
}
void VNull(float[] v){ v[0]=0; v[1]=0; v[2]=0; }
float[] VAdd(float[] a,float[] b){
  float[] v={a[0]+b[0],a[1]+b[1],a[2]+b[2]};
  return v;
}
float[] VAdd(float[] a,float[] b,float[] c){
  float[] v={a[0]+b[0]+c[0],a[1]+b[1]+c[1],a[2]+b[2]+c[2]};
  return v;
}
float[] VAdd(float[] a,float[] b,float[] c,float[] d){
  float[] v={a[0]+b[0]+c[0]+d[0],a[1]+b[1]+c[1]+d[1],a[2]+b[2]+c[2]+d[2]};
  return v;
}
float[] VOpposite(float[] v){
  float[] c = {-v[0],-v[1],-v[2]};
  return c;
}
void VAddInplace(float[] a, float[] b){
  a[0]+=b[0]; a[1]+=b[1]; a[2]+=b[2];
}
float[] VSub(float[] a,float[] b){
  float[] v={a[0]-b[0],a[1]-b[1],a[2]-b[2]};
  return v;
}
void VSubInplace(float[] a,float[] b){
  a[0]-=b[0]; a[1]-=b[1]; a[2]-=b[2];
}
float VMag(float[] v){
  return sqrt(sq(v[0])+sq(v[1])+sq(v[2]));
}
float VQuadr(float[] v){
  return sq(v[0])+sq(v[1])+sq(v[2]);
}
float[] VNorma(float[] v){
  float m=sqrt(sq(v[0])+sq(v[1])+sq(v[2]));
  if(m>0){
    float[] u={v[0]/m, v[1]/m, v[2]/m};
    return u;
  }else{
    float[] u={1,0,0};
    return u;
  }
}
float[] VNorma4(float[] v){
  float m=sqrt(sq(v[0])+sq(v[1])+sq(v[2])+sq(v[3]));
  float[] u={v[0]/m, v[1]/m, v[2]/m, v[3]/m};
  return u;
}
float[] VCross(float[] a,float[] b){
  float[] v={a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};
  return v;
}
float[] VScale(float k, float[] v){
  float[] c={k*v[0],k*v[1],k*v[2]};
  return c;
}
void VScaleInplace(float k, float[] v){
  v[0]*=k; v[1]*=k; v[2]*=k;
}
float VDot(float[] a, float[] b){
  return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
}
float[] VTransf(float[] v,float[][] m){
  float[] c={0,0,0};
  for(int j=0; j<3; j++){
    for(int i=0; i<3; i++){
      c[j]+=v[i]*m[i][j];
    }
  }
  return c;
}
float[] VTransf4(float[] v,float[][] m){
  float[] c={0,0,0,0};
  for(int j=0; j<4; j++){
    for(int i=0; i<4; i++){
      c[j]+=v[i]*m[i][j];
    }
  }
  return c;
}
float[] VTransf4D(float[] v, float[][] m){
  float[] c={0,0,0};
  for(int j=0; j<3; j++){
    for(int i=0; i<3; i++){
      c[j]+=v[i]*m[i][j];
    }
    c[j]+=m[3][j];
  }
  return c;
}
float[] VProjNorm(float[] a, float[] b){
  return VScale(VDot(a,b)/VQuadr(b),b);
}
float[] VProjTan(float[] a,float[] b){
  return VSub(a,VScale(VDot(a,b)/VQuadr(b),b));
}
void VProjTanInplace(float[] a,float[] b){
  VSubInplace(a,VScale(VDot(a,b)/VQuadr(b),b));
}



//matrix stuff
void MCopy(float[][] a, float[][] b){
  for(int i=0; i<3; i++){
    for(int j=0; j<3; j++){
      a[i][j]=b[i][j];
    }
  }
}
void MCopy4(float[][] a, float[][] b){
  for(int i=0; i<4; i++){
    for(int j=0; j<4; j++){
      a[i][j]=b[i][j];
    }
  }
}
float[][] MSum(float[][] a,float[][] b){
  float[][] c={{0,0,0},{0,0,0},{0,0,0}};
  for(int i=0; i<3; i++){
    for(int j=0; j<3; j++){
      c[i][j]=a[i][j]+b[i][j];
    }
  }
  return c;
}
float[][] MMult(float[][] a,float[][] b){
  float[][] c={{0,0,0},{0,0,0},{0,0,0}};
  for(int i=0; i<3; i++){
    for(int j=0; j<3; j++){
      for(int k=0; k<3; k++){
        c[i][j]+=a[k][j]*b[i][k];
      }
    }
  }
  return c;
}
float[][] MMult4(float[][] a,float[][] b){
  float[][] c={{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};
  for(int i=0; i<4; i++){
    for(int j=0; j<4; j++){
      for(int k=0; k<4; k++){
        c[i][j]+=a[k][j]*b[i][k];
      }
    }
  }
  return c;
}
float[][] MScale(float[][] m,float k){
  float[][] c={{0,0,0},{0,0,0},{0,0,0}};
  for(int i=0; i<3; i++){
    for(int j=0; j<3; j++){
      c[i][j]=m[i][j]*k;
    }
  }
  return c;
}
float[][] MScale4(float[][] m,float k){
  float[][] c={{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};
  for(int i=0; i<4; i++){
    for(int j=0; j<4; j++){
      c[i][j]=m[i][j]*k;
    }
  }
  return c;
}
float[][] WMat(float[] w){
  float[][] c={{0,w[2],-w[1]},{-w[2],0,w[0]},{w[1],-w[0],0}};
  return c;
}
float[][] Rot(float[] w,float t){
  float[][] wmat = WMat(w);
  return MSum(I3,MSum(MScale(wmat,sin(t)),MScale(MMult(wmat,wmat),(1-cos(t)))));
}
float[][] Rot(float[] W){
  float t = VMag(W);
  if(t>1e-6){
    float[][] wmat = WMat(VScale(1/t,W));
    return MSum(I3,MSum(MScale(wmat,sin(t)),MScale(MMult(wmat,wmat),(1-cos(t)))));
  }else{ return I3; }
}
float[][] T(float[][] R,float[] D){
  float[][] c={{1,0,0,0},{0,1,0,0},{0,0,1,0},{D[0],D[1],D[2],1}};
  MCopy(c,R);
  return c;
}
float[][] MTransp(float[][] m){
  float[][] c={{0,0,0},{0,0,0},{0,0,0}};
  for(int i=0; i<3; i++){
    for(int j=0; j<3; j++){
      c[j][i] = m[i][j];      
    }
  }
  return c;
}
float[][] MTransp4(float[][] m){
  float[][] c={{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};
  for(int i=0; i<4; i++){
    for(int j=0; j<4; j++){
      c[j][i] = m[i][j];      
    }
  }
  return c;
}
//determinant of 3x3 matrix
float Det(float[][] m){
  return (m[0][0]*m[1][1]*m[2][2]+m[0][1]*m[1][2]*m[2][0]+m[0][2]*m[1][0]*m[2][1]-
    m[2][0]*m[1][1]*m[0][2]-m[2][1]*m[1][2]*m[0][0]-m[2][2]*m[1][0]*m[0][1]);
}
//inverse of 3x3 matrix
float[][] Invert(float[][] m,float det){
  float[][] I=new float[3][3];
  I[0][0]=(m[1][1]*m[2][2]-m[2][1]*m[1][2])/det;
  I[0][1]=(m[0][2]*m[2][1]-m[2][2]*m[0][1])/det;
  I[0][2]=(m[0][1]*m[1][2]-m[1][1]*m[0][2])/det;
  
  I[1][0]=(m[1][2]*m[2][0]-m[2][2]*m[1][0])/det;
  I[1][1]=(m[0][0]*m[2][2]-m[2][0]*m[0][2])/det;
  I[1][2]=(m[0][2]*m[1][0]-m[1][2]*m[0][0])/det;
  
  I[2][0]=(m[1][0]*m[2][1]-m[2][0]*m[1][1])/det;
  I[2][1]=(m[0][1]*m[2][0]-m[2][1]*m[0][0])/det;
  I[2][2]=(m[0][0]*m[1][1]-m[1][0]*m[0][1])/det;
  
  return I;
}
//invert a transformation matrix
float[][] TInverse(float[][] t){
  float[][] c= {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
  MCopy(c,MTransp(t)); VCopy(c[3],VTransf(VScale(-1,t[3]),c));
  return c;
}


//quaternion corresponding to rotation(unit axis, angle)
float[] Quat(float t, float[] w){
  float s=sin(t/2);
  float[] q={cos(t/2),s*w[0],s*w[1],s*w[2]};
  return q;
}
float[] QMult(float[] q1, float[] q2){
  float[] c={q1[0]*q2[0]-q1[1]*q2[1]-q1[2]*q2[2]-q1[3]*q2[3],
             q1[0]*q2[1]+q1[1]*q2[0]+q1[2]*q2[3]-q1[3]*q2[2],
             q1[0]*q2[2]-q1[1]*q2[3]+q1[2]*q2[0]+q1[3]*q2[1],
             q1[0]*q2[3]+q1[1]*q2[2]-q1[2]*q2[1]+q1[3]*q2[0]};
  return c;
}
void QMultInplace(float[] q1, float[] q2){
   q1[0] = q1[0]*q2[0]-q1[1]*q2[1]-q1[2]*q2[2]-q1[3]*q2[3];
   q1[1] = q1[0]*q2[1]+q1[1]*q2[0]+q1[2]*q2[3]-q1[3]*q2[2];
   q1[2] = q1[0]*q2[2]-q1[1]*q2[3]+q1[2]*q2[0]+q1[3]*q2[1];
   q1[3] = q1[0]*q2[3]+q1[1]*q2[2]-q1[2]*q2[1]+q1[3]*q2[0];
}
float[] QgetUnitW(float[] q){
  float m=sqrt(sq(q[1])+sq(q[2])+sq(q[3]));
  float[] c={q[1]/m,q[2]/m,q[3]/m};
  return c;
}
//quaternion to rotation matrix
float[][] QRot(float[] q){
  float s = 1;//VQuadr(q);
  float[][] c={
    {1-2*s*(sq(q[2])+sq(q[3])),2*s*(q[1]*q[2]+q[0]*q[3]),2*s*(q[1]*q[3]-q[0]*q[2])},
    {2*s*(q[1]*q[2]-q[0]*q[3]),1-2*s*(sq(q[1])+sq(q[3])),2*s*(q[2]*q[3]+q[0]*q[1])},
    {2*s*(q[1]*q[3]+q[0]*q[2]),2*s*(q[2]*q[3]-q[0]*q[1]),1-2*s*(sq(q[1])+sq(q[2]))}
  };
  return c;
}




//see theory paper "force effects" for more details
//without external force
float[] FpressOfInvertedPendulum(float ML2, float Icom, float[] Fg, float[] L){
  float[] FgNorm = VScale(VDot(Fg,L)/VQuadr(L),L);
  return VAdd(FgNorm,VScale(Icom/(Icom+ML2),VSub(Fg,FgNorm)));
}
//with external force
float[] FpressOfInvertedPendulum(float ML2, float Icom, float[] Fg, float[] L, float[] Fq, float Kql){
  float[] FgNorm = VScale(VDot(Fg,L)/VQuadr(L),L);
  float[] FqNorm = VScale(VDot(Fq,L)/VQuadr(L),L);
  return VAdd(VAdd(FqNorm,FgNorm)
             ,VScale(1/(Icom+ML2),VAdd(VScale(Kql*ML2+Icom,VSub(Fq,FqNorm)),VScale(Icom,VSub(Fg,FgNorm)))));
}




class PIDobj{
  //target value
  float target=0;

  //PID constants
  float pk=0;
  float ik=0; 
  float dk=0;
  //PID error values
  float e=0;
  float ie=0; //integral error
  float de=0; //average derivative error
  //previous instantaneous error derivative values
  float de1 = 0; float de2=0; float de3=0; float de4=0;

  PIDobj(float p,float i, float d){
    pk = p; ik = i; dk = d;
  }

  void SetTarget(float t){
    e += t-target;  //solve derivative inconsistency between target change
    target = t;
  }
  //just update
  void Tick(float dt, float val){
    de4 = de3; de3 = de2; de2 = de1; de1 = (val-target-e)/dt;
    e = val-target; ie += e*dt;
    //weighted average of previous insta derivatives
    de = (de1*3+de2*2+de3*2+de4)/8;
  }
  //update and get PID output
  float Get(float dt, float val){
    de4 = de3; de3 = de2; de2 = de1; de1 = (val-target-e)/dt;
    e = val-target; ie += e*dt;
    //weighted average of previous insta derivatives
    de = de1*0.375+de2*0.25+de3*0.25+de4*0.125;

    return (pk*e + ik*ie + dk*de);
  }

  void Reset(float val){
    e=val-target; ie=0; de=0;
    de1=0; de2=0; de3=0; de4=0;
  }
}

class PIDobj3D{
  //target value
  float[] target={0,0,0};

  //PID constants
  float pk=0;
  float ik=0; 
  float dk=0;
  //PID error values
  float[] e={0,0,0};
  float[] ie={0,0,0}; //integral error
  float[] de={0,0,0}; //average derivative
  //previous instantaneous derivative values
  float[] de1 = {0,0,0}; float[] de2={0,0,0}; float[] de3={0,0,0}; float[] de4={0,0,0};

  PIDobj3D(float p,float i, float d){
    pk = p; ik = i; dk = d;
  }

  void SetTarget(float[] t){
    VSubInplace(e,VSub(t,target));
    VCopy(target,t);
  }
  //just update
  void Tick(float dt, float[] val){
    VCopy(de4,de3); VCopy(de3,de2); VCopy(de2,de1); de1 = VScale(1/dt,VSub(VSub(val,target),e));
    e = VSub(val,target); VAddInplace(ie,VScale(dt,e));
    //weighted average of previous insta derivatives
    de = VAdd(VScale(0.375,de1),VScale(0.25,de2),VScale(0.25,de3),VScale(0.125,de4));
  }
  //update and get PID output
  float[] Get(float dt, float[] val){
    VCopy(de4,de3); VCopy(de3,de2); VCopy(de2,de1); de1 = VScale(1/dt,VSub(VSub(val,target),e));
    e = VSub(val,target); VAddInplace(ie,VScale(dt,e));
    //weighted average of previous insta derivatives
    de = VAdd(VScale(0.375,de1),VScale(0.25,de2),VScale(0.25,de3),VScale(0.125,de4));

    return VAdd(VScale(pk,e),VScale(ik,ie),VScale(dk,de));
  }

  //reset the accumulated error values
  void Reset(float[] val){
    e = VSub(val,target); VNull(ie); VNull(de);
    VNull(de1); VNull(de2); VNull(de3); VNull(de4);
  }
}
