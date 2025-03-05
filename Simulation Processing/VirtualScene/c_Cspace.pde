//everything that defines and works with the Cspace object,
//  and the contruction of the default Joint and Mass structure


//config objects contain general/common parameters to create a template on which every actual
//  Mass and Joint object is built on, cause the only parameters that differentiate actual objects
//  are the ones Cspace dependent (i.e. position in S frame influenced by P,Q and Th of the Cspace)
class MassObjConfig{
  float mass = 0;
  int type = 0;         //not defined, point mass, rod mass, cylinder mass...
  float[] p = {0,0,0};  //position in joint frame
  float[] l = {0,0,0};  //length span vector in joint frame
  float[] com = {0,0,0}; //centre of mass in joint frame
  //point mass constructor
  MassObjConfig(float m, float[] P){
    mass=m; type=1; VCopy(p,P); VCopy(com,p);
  }
  //rod mass constructor
  MassObjConfig(float m, float[] P, float[] L){
    mass=m; type=2; VCopy(p,P); VCopy(l,L); VCopy(com,VAdd(p,VScale(0.5,l)));
  }
}
class JointConfig{
  int parent=0;
  int[] children;             //indexes of all the children joints
  boolean[] childrenBool;     //if the joint with this index in the joint array is a child
  float[] p = {0,0,0}; //displacement vector from parent
  float[] w = {1,0,0}; //rotation axis in parent frame
  
  int masses;
  MassObjConfig[] massCnf;
  float mass=0;
  float[] com = {0,0,0};     //centre of mass in parent frame
  
  JointConfig(int Pa, String C, float[] P, float[] W){
    parent = Pa;
    for(int i=0; i<C.length() && i<14; i++){ children[i] = C.charAt(i)-48; }
    VCopy(p,P); VCopy(w,W);
  }
  JointConfig(int Pa, String C, float[] P, float[] W, MassObjConfig[] M){
    parent = Pa;
    children = new int[C.length()]; childrenBool = new boolean[C.length()];
    int childrenIndex = 0;
    for(int i=0; i<C.length(); i++){
      if(C.charAt(i)-48==1){
        childrenBool[i] = true;
        children[childrenIndex] = i;
        childrenIndex++;
      }else{
        childrenBool[i] = false;
      }
    }children[childrenIndex] = 0; //signaling end of children when reading through the array

    VCopy(p,P); VCopy(w,W);
    masses=M.length;
    massCnf = new MassObjConfig[masses];
    for(int i=0; i<masses; i++){
      massCnf[i] = M[i];
      mass+=massCnf[i].mass;
      VAddInplace(com,VScale(massCnf[i].mass,massCnf[i].com));
    }
    VScaleInplace(1/mass,com);
  }
}



class MassObj{
  MassObjConfig cnf;
  //parameters updated manually by parent joint
  float[] p = {0,0,0};  //position start and end in S frame
  float[] l = {0,0,0};  //length span vector in S frame
                        //updated by the parent joint whenever a theta changes
  MassObj(MassObjConfig c){ cnf=c; }

  //moment of inertia of this element, calculations done in S frame
  float CalcI(float[] Jorigin, float[] W){
    if(cnf.type==1){
      return cnf.mass*(VQuadr(VSub(p,Jorigin))-sq(VDot(W,VSub(p,Jorigin))));
    }else if(cnf.type==2){
      float[] jp=VSub(p,Jorigin);
      return cnf.mass*(VQuadr(jp)-sq(VDot(W,jp))+VDot(jp,l)-VDot(W,jp)*VDot(W,l)+VQuadr(l)/3-sq(VDot(W,l))/3);
    }else{
      //if not defined
      return 0;
    }
  }

}

class Joint{
  JointConfig cnf;
  Joint J;               //the parent joint object
  float[][] Tp = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};//frame transform from parent to itself
  float[][] Ts = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};//frame transform from S frame to itself
  float[] W_s = {1,0,0}; //rotation axis in S frame
  
  MassObj[] Mass;        //mass objects array
  float[] COM={0,0,0};   //centre of mass in S frame

  float childrenMass = 0;     //mass sum of itself and children, computed by SetupJ in Cspace
  
  //update this joint's masses to this joint's Ts
  void UpdateMass(){
    for(int i=0; i<cnf.masses; i++){
      if(Mass[i].cnf.type==1){
        Mass[i].p = VTransf4D(Mass[i].cnf.p,Ts);
      }else if(Mass[i].cnf.type==2){
        Mass[i].p = VTransf4D(Mass[i].cnf.p,Ts);
        Mass[i].l = VTransf(Mass[i].cnf.l,Ts);
      }
    }
    COM = VTransf4D(cnf.com,Ts);
  }
  //update this joint's parameters to a new theta value
  void Update(float t){
    Tp = T(Rot(cnf.w,t),cnf.p);
    Ts = MMult4(J.Ts,Tp);
    W_s = VTransf(cnf.w,Tp);
    UpdateMass();
  }
  //just update this joint's S frame parameters
  void Update(){
    Ts = MMult4(J.Ts,Tp);
    W_s = VTransf(cnf.w,Tp);
    UpdateMass();
  }

  //blank joint object, can act as the parent of the first joint in joint array
  Joint(){}
  //joint created as child of another joint object
  Joint(JointConfig c, Joint j){
    cnf = c; J=j;
    //from the mass config array in the joint config, create the mass objects of this joint
    if(cnf.masses>0){
      Mass = new MassObj[cnf.masses];
      for(int i=0; i<cnf.masses; i++){ Mass[i] = new MassObj(cnf.massCnf[i]); }
    }
    Update(0);
  }
}


//a Cspace configuration of the robot
class Cspace{
  int Joints=0;
  Joint[] J;            //joint array
  float[] Th;           //Theta vector of joint array
  
  float Mass=0;             //mass sum of all the mass objects in the joint array
  //rotation and displacement due to angular conservation through joint movements
  float[] Wb_com={0,0,0}; //rotation between b and com frame
  float[] Db_com={0,0,0};   //displacement between b and S frame
  float[] Db_com0={0,0,0};  //body position in com frame at start configuration (theta is null)
  //the final T matrix used to transform from com to b frame
  float[][] Tb_com = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
  float[][] Tcom_b = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};

  //transform from S to com frame, updated manually externally to this object,
  //  either by value, or by linking by reference an external matrix to this Tcom_s matrix
  float[][] Tcom_s = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
  float[][] Ts_com = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};

  //HARDCODED... will fix later?
  float[] Dfoot = {0,0,-Tibia};
  int FootRparent = 4;
  int FootLparent = 8;
  //updated with UpdateJ()
  float[] FootR = {0,0,0};
  float[] FootL = {0,0,0};
  float[] COMFootR = {0,0,0};
  float[] COMFootL = {0,0,0};
  float[] FootRL = {0,0,0}; //right to left foot vector
  
  void UpdateTcoms(float[][] ts){
    Tcom_s = ts; Ts_com = TInverse(Tcom_s);
    UpdateJ();
  }
  void UpdateTcoms(float[][] ts, float[][] tcom){
    Tcom_s = ts; Ts_com = tcom;
    UpdateJ();
  }

  //update every joint to the current Tcom and Tb
  void UpdateJ(){
    //MCopy4(Tb_com,I4);    //DEBUG TO EXCLUDE BODYCONSERVATION EFFECTS

    if(Joints>0){
      J[0].Ts = MMult4(Tcom_s,Tb_com);
      J[0].UpdateMass();
      for(int j=1; j<Joints; j++){
        J[j].Update();
      }
    }

    FootR = VTransf4D(Dfoot,J[FootRparent].Ts); FootL = VTransf4D(Dfoot,J[FootLparent].Ts);
    COMFootR = VSub(FootR,Tcom_s[3]); COMFootL = VSub(FootL,Tcom_s[3]);
    FootRL = VSub(FootL,FootR);
  }
  //updates the joint to new angle value, and adds to Wb_com the resulting angle conservation term
  void ComputeWbcom(int j, float t){
    //compute COMchild
    float[] COMchild = {0,0,0}; float[] COMparent = {0,0,0};
    for(int i=0; i<Joints; i++){
      if(J[j].cnf.childrenBool[i] || i==j){ VAddInplace(COMchild,VScale(J[i].cnf.mass,J[i].COM)); }
      else{ VAddInplace(COMparent,VScale(J[i].cnf.mass,J[i].COM)); }
    }
    VScaleInplace(1/J[j].childrenMass,COMchild);
    VScaleInplace(1/(Mass-J[j].childrenMass),COMparent);
    //update joint and all its children to new theta value
    J[j].Update(t);
    for(int i=0; J[j].cnf.children[i]!=0; i++){ J[J[j].cnf.children[i]].Update(); }
    float[] wj = VTransf(J[j].cnf.w,J[j].J.Ts);  //compute rotation axis of child in S frame
    float tj = t-Th[j];
    Th[j] = t;
    //compute COMchild2
    float[] COMchild2 = VScale(J[j].cnf.mass,J[j].COM);
    for(int i=0; J[j].cnf.children[i]!=0; i++){
      VAddInplace(COMchild2,VScale(J[J[j].cnf.children[i]].cnf.mass,J[J[j].cnf.children[i]].COM));
    }
    VScaleInplace(1/J[j].childrenMass,COMchild2);

    //compute COMparent->COMchild vectors, and its rotation vector wb
    float[] rjst = VSub(COMchild,COMparent); float[] rjen = VSub(COMchild2,COMparent);
    float[] wb; float tb = 0;
    float cosine = VDot(rjst,rjen)/sqrt(VQuadr(rjst)*VQuadr(rjen));
    if(abs(cosine)+1e-6<1){
      wb = VCross(rjst,rjen); VScaleInplace(1/VMag(wb),wb); tb = acos(cosine);
    }else if(cosine>0){  //if cos = 1, sin = 0, -> no rotation
      wb = vx; tb = 0;
    }else{  //if cos = -1, sin = 0
      //any unit vector perpendicular to rj fits
      wb = VScale(1/VQuadr(rjst),VCross(rjst,Vxyrotate(rjst))); tb = PI; 
    }

    //compute inertia moment of parent and child
    float Iparent = 0; float Ichildj = 0; float Ichildb = 0;
    for(int i=0; i<Joints; i++){
      if(J[j].cnf.childrenBool[i] || i==j){
        for(int m=0; m<J[i].cnf.masses; m++){
          Ichildj+=J[i].Mass[m].CalcI(COMchild2,wj);
          Ichildb+=J[i].Mass[m].CalcI(COMchild2,wb);
        }
      }else{
        for(int m=0; m<J[i].cnf.masses; m++){
          Iparent+=J[i].Mass[m].CalcI(COMparent,wb);
        }
      }
    }

    VAddInplace(Wb_com,VAdd(VScale(tb*Iparent,wb),VScale(-tj*Ichildj,wj)));
    VAddInplace(Wb_com,VScale(-tb*(Iparent+Ichildb+VQuadr(rjen)*(Mass-J[j].childrenMass)*J[j].childrenMass/Mass),wb));
  }
  //for this to work the rotational part of Tb_com needs to have been already computed
  void ComputeDbs(){
    VNull(Db_com);
    for(int j=0; j<Joints; j++){ VAddInplace(Db_com,VScale(J[j].cnf.mass,J[j].COM)); }
    VScaleInplace(-1/Mass,Db_com);
    Db_com = VTransf(VAdd(Db_com,J[0].Ts[3]),Ts_com);
  }

  //update joint angle value for a joint
  void UpdateJ(int j, float t,boolean coerce){
    //ResetJ();
    if(abs(Th[j]-t)>1e-6 || coerce){
      float th = Th[j];
      /*float dth = (th-t>0?-0.09:0.09);
      while(abs(th-t)>0.1){
        th+=dth;
        ComputeWbcom(j,th);
      }*/
      th = t; ComputeWbcom(j,th);
      ComputeDbs();
      Tb_com = T(Rot(Wb_com),Db_com);
      Tcom_b = TInverse(Tb_com);
      UpdateJ();    
    }
  }

  //with a theta vector, only the joints that change and their children will be updated
  void UpdateJ(float[] t, boolean coerce){
    //ResetJ();
    for(int j=1; j<Joints; j++){
      //update only if the new theta changed, and its children
      if(abs(Th[j]-t[j])>1e-6 || coerce){
        float th = Th[j];
        /*float dth = (th-t[j]>0?-0.04:0.04);
        while(abs(th-t[j])>0.05){
          th+=dth;
          ComputeWbcom(j,th);
        }*/
        th = t[j]; ComputeWbcom(j,th);
      }
    }
    ComputeDbs();
    Tb_com = T(Rot(Wb_com),Db_com);
    Tcom_b = TInverse(Tb_com);
    UpdateJ();
  }
  //recalculates joint changes from home position (theta vector is null), to reset accrued drift error
  void ResetJ(){
    for(int j=1; j<Joints; j++){ Th[j]=0; }
    VNull(Wb_com); Db_com = Db_com0;
    Tb_com = T(I3,Db_com);
    Tcom_b = T(I3,VOpposite(Db_com));
    UpdateJ();
  }

  //create the joint array, initialise Th vector and compute total Mass
  void SetupJ(JointConfig[] jcnf){
    Joints = jcnf.length;
    J = new Joint[Joints];
    Th = new float[Joints];
    //first joint is child of a blank joint (with just a Ts = I4)
    J[0] = new Joint(jcnf[0],(new Joint()));
    J[0].Ts = I4;
    J[0].UpdateMass();
    //other joints are children of the first joint
    for(int j=1; j<Joints; j++){
      J[j] = new Joint(jcnf[j],J[jcnf[j].parent]);
    }
    //compute total mass and childrenMass of each joint
    for(int j=0; j<Joints; j++){
      Th[j]=0;
      VSubInplace(Db_com0,VScale(J[j].cnf.mass,J[j].COM));
      for(int i=0; i<jcnf[j].masses; i++){
        Mass+=jcnf[j].massCnf[i].mass;
        J[j].childrenMass+=jcnf[j].massCnf[i].mass;
      }
      for(int i=0; jcnf[j].children[i]!=0; i++){
        for(int k=0; k<jcnf[jcnf[j].children[i]].masses; k++){
          J[j].childrenMass+=jcnf[jcnf[j].children[i]].massCnf[k].mass;
      }}
      J[j].childrenMass = round(J[j].childrenMass*10000.0)/10000.0;
    }
    Mass = round(Mass*10000.0)/10000.0;
    VScaleInplace(1/Mass,Db_com0);
    ResetJ();
  }

  //different constructors
  Cspace(JointConfig[] jcnf){
    SetupJ(jcnf);
  }
  Cspace(JointConfig[] jcnf, float[] t){
    SetupJ(jcnf);
    UpdateJ(t,true);
  }
  Cspace(JointConfig[] jcnf, float[][] T){
    Tcom_s = T;
    SetupJ(jcnf);
  }
  
  Cspace(JointConfig[] jcnf, float[][] T, float[] t){
    Tcom_s = T;
    SetupJ(jcnf);
    UpdateJ(t,true);
  }
  
  //copies a given cspace onto itself, both must have same joint and masses number
  //!!every variable is copied by value, i.e. any reference links are deleted (like Tcom_s)
  void Copy(Cspace c){
    Joints = c.Joints;
    Mass = c.Mass;
    VCopy(Wb_com,c.Wb_com); VCopy(Db_com,c.Db_com);
    MCopy4(Tb_com,c.Tb_com); MCopy4(Tcom_b,c.Tcom_b);
    MCopy4(Tcom_s,c.Tcom_s); MCopy4(Ts_com,c.Ts_com);
  
    for(int i=0; i<Joints; i++){
      Th[i] = c.Th[i];
      //copy joints
      J[i].cnf = c.J[i].cnf;
      for(int k=0; k<4; k++){for(int j=0; j<4; j++){
        J[i].Tp[k][j] = c.J[i].Tp[k][j]; J[i].Ts[k][j] = c.J[i].Ts[k][j];
      }}
      //copy masses
      for(int m=0; m<J[i].cnf.masses; m++){
        J[i].Mass[m].cnf = c.J[i].Mass[m].cnf;
        VCopy(J[i].Mass[m].p,c.J[i].Mass[m].p); VCopy(J[i].Mass[m].l,c.J[i].Mass[m].l);
      }
      J[i].childrenMass = c.J[i].childrenMass;
    }
    //the first joint's parent doesn't change, update the other parents
    for(int i=1; i<Joints; i++){ J[i].J = J[c.J[i].cnf.parent]; }
  }

  //moment of inertia of the whole joint array through some unit vector in S frame
  float Itot(float[] p, float[] w){
    float itot=0;
    for(int j=0; j<Joints; j++){
      for(int m=0; m<J[j].cnf.masses; m++){
        itot+=J[j].Mass[m].CalcI(p,w);
      }
    }
    return itot;
  }

  //given a new foot coordinate in COM frame, find the corresponding theta configuration (approx.)
  //!!!for now JCstandard specific, cause it has to use the custom function JCstandard
  void SetFootR(float[] fr){
    fr = VTransf4D(fr,Tcom_b);
    boolean kneeFront = true;//(Th[4]<0); //if the knee is bent forward
    float[] newTheta = new float[Joints];
    for(int i=0; i<Joints; i++){ newTheta[i] = Th[i]; }

    float[] thetaR = JCstandardGetTheta(true,kneeFront,newTheta[1],fr);
    newTheta[2] = thetaR[0]; newTheta[3] = thetaR[1]; newTheta[4] = thetaR[2];
    UpdateJ(newTheta,false);
  }
  void SetFootL(float[] fl){
    fl = VTransf4D(fl,Tcom_b);
    boolean kneeFront = true;
    float[] newTheta = new float[Joints];
    for(int i=0; i<Joints; i++){ newTheta[i] = Th[i]; }

    float[] thetaL = JCstandardGetTheta(false,kneeFront,newTheta[5],fl);
    newTheta[6] = thetaL[0]; newTheta[7] = thetaL[1]; newTheta[8] = thetaL[2];
    UpdateJ(newTheta,false);
  }
  void SetFootRL(float[] fr, float[] fl){
    fr = VTransf4D(fr,Tcom_b); fl = VTransf4D(fl,Tcom_b);
    boolean kneeFront = true;
    float[] newTheta = new float[Joints];
    for(int i=0; i<Joints; i++){ newTheta[i] = Th[i]; }

    float[] thetaR = JCstandardGetTheta(true,kneeFront,newTheta[1],fr);
    float[] thetaL = JCstandardGetTheta(false,kneeFront,newTheta[5],fl);
    newTheta[2] = thetaR[0]; newTheta[3] = thetaR[1]; newTheta[4] = thetaR[2];
    newTheta[6] = thetaL[0]; newTheta[7] = thetaL[1]; newTheta[8] = thetaL[2];
    
    UpdateJ(newTheta,false);
  }
}


//children must appear any index after their parent in a joint array
//the standard joint structure looks like this:
//  body,h1r,h2r,lr,kr,h1l,h2l,ll,kl

//MOC->MassObjConfig,   b->body h1->hip1...
MassObjConfig[] MOCb,MOCh1,MOCh2,MOCl,MOCk;
//JC-> JointConfig
JointConfig[] JCstandard;

void SetupCspaceConfigs(){
  //argument vectors
  float[] u = {0,0,0}; float[] v = {0,0,0};

  //mass config arrays, u,v correspond to p,l in the config object
  MOCb = new MassObjConfig[4];
  
  MOCb[0] = new MassObjConfig(1,u);
  v[0]=HipRad;
  MOCb[1] = new MassObjConfig(0.4,u,v);
  v[0]=-HipRad;
  MOCb[2] = new MassObjConfig(0.4,u,v);
  u[2]=0.2;
  MOCb[3] = new MassObjConfig(1,u);
  
  MOCh1 = new MassObjConfig[2];
  u[2]=0;
  MOCh1[0] = new MassObjConfig(0.2,u);
  v[0]=0; v[2]=-Hip1Height;
  MOCh1[1] = new MassObjConfig(0.2,u,v);
  
  MOCh2 = new MassObjConfig[2];
  MOCh2[0] = new MassObjConfig(0.2,u);
  v[2]=-Hip2Height;
  MOCh2[1] = new MassObjConfig(0.05,u,v);
  
  MOCl = new MassObjConfig[2];
  MOCl[0] = new MassObjConfig(0.1,u);
  v[2]=-Femur;
  MOCl[1] = new MassObjConfig(0.15,u,v);
  
  MOCk = new MassObjConfig[3];
  MOCk[0] = new MassObjConfig(0.15,u);
  v[2]=-Tibia;
  MOCk[1] = new MassObjConfig(0.1,u,v);
  u[2]=-Tibia;
  MOCk[2] = new MassObjConfig(0.1,u);

  u[0]=0; u[1]=0; u[2]=0; v[0]=0; v[1]=0; v[2]=0;
  //joint config objects, u,v correspond to P,W in the config object
  JCstandard = new JointConfig[9];
  
  JCstandard[0] = new JointConfig(-1,"011111111",u,v,MOCb);
  
  
  
  u[0]=HipRad; v[0] = 0; v[2]=1;
  JCstandard[1] = new JointConfig(0,"001110000",u,v,MOCh1);
  u[0]=0; u[2]=-Hip1Height; v[2]=0; v[1]=1;
  JCstandard[2] = new JointConfig(1,"000110000",u,v,MOCh2);
  u[2]=-Hip2Height; v[1]=0; v[0]=1;
  JCstandard[3] = new JointConfig(2,"000010000",u,v,MOCl);
  u[2]=-Femur;
  JCstandard[4] = new JointConfig(3,"000000000",u,v,MOCk);

  u[2]=0; u[0]=-HipRad; v[0]=0; v[2]=1;
  JCstandard[5] = new JointConfig(0,"000000111",u,v,MOCh1);
  u[0]=0; u[2]=-Hip1Height; v[2]=0; v[1]=1;
  JCstandard[6] = new JointConfig(5,"000000011",u,v,MOCh2);
  u[2]=-Hip2Height; v[1]=0; v[0]=1;
  JCstandard[7] = new JointConfig(6,"000000001",u,v,MOCl);
  u[2]=-Femur;
  JCstandard[8] = new JointConfig(7,"000000000",u,v,MOCk);
}


//for the standard JointConfig object, get theta configuration of the leg given foot coordinate 
//not including inverse bodyconservation calculation, constraints are yaw theta and if knee is bent forward
float[] JCstandardGetTheta(boolean right, boolean kneeFront, float t1, float[] x){
  //hip2,leg,knee found theta
  float[] t={0,0,0};

  float k5=right?HipRad:-HipRad;

  float s4 = sin(t1); float c4 = cos(t1);
  float s3 = -(x[0]*c4+x[1]*s4-k5*c4)/sqrt(sq(x[0]*c4+x[1]*s4-k5*c4)+sq(x[2]+Hip1Height));
  if(s3>1-1e-6||s3<-1+1e-6){ s3 = s3>0?1:-1; }
  float c3 = sqrt(1-sq(s3));

  if(x[2]<-Hip1Height){
    t[0] = asin(s3);
  }else{
    c3*=-1;
    t[0] = PI-asin(s3);
    if(t[0]>2*PI){ t[0]-= 2*PI; }
  }

  float[] vL = {x[0]+Hip2Height*s3*c4-k5,x[1]+Hip2Height*s3*s4,x[2]+Hip2Height*c3+Hip1Height};
  float L = VMag(vL);

  if(L>Tibia+Femur-1e-6){
    VSubInplace(x,VScale((L-Femur-Tibia+1e-6)/L,vL));
    L=Femur+Tibia-1e-6;
  }

  t[2] = (kneeFront?-1:1)*(PI-acos((sq(Tibia)+sq(Femur)-sq(L))/(2*Tibia*Femur)));
  t[1] = ((x[2]<-Hip2Height*c3-Hip1Height)?asin((-x[0]*s4+x[1]*c4+k5*s4)/L):
            PI-asin((-x[0]*s4+x[1]*c4+k5*s4)/L))+acos((sq(L)+sq(Femur)-sq(Tibia))/(2*L*Femur));
  return t;
}
