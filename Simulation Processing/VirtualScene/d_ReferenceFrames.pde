
//Live body C space
Cspace RealC;

//aircraft principal axes
float[] yaw={0,0,-1}; float[] pitch={1,0,0}; float[] roll={0,1,0};



void SetupFrames(){

  float[] t = {0,0,0,0,0,0,0,0,0};
  RealC = new Cspace(JCstandard,t);

}





//returns foot coordinates in the body frame, from the t1,t2,t3,t4 of the specific leg
float[] getFoot(boolean right, float[] X){
 float[] Y = {
    Tibia*(sin(X[0])*sin(X[3])*cos(X[1])+sin(X[0])*sin(X[1])*sin(X[2])*cos(X[3])+sin(X[1])*sin(X[3])*cos(X[0])-sin(X[2])*cos(X[0])*cos(X[1])*cos(X[3]))
      +Femur*(sin(X[1])*sin(X[3])-sin(X[2])*cos(X[1])*cos(X[3]))+(right?HipRad:-HipRad),
    Tibia*(sin(X[0])*cos(X[1])*cos(X[3])-sin(X[0])*sin(X[1])*sin(X[2])*sin(X[3])+sin(X[1])*cos(X[0])*cos(X[3])+sin(X[2])*sin(X[3])*cos(X[0])*cos(X[1]))
      +Femur*(sin(X[1])*cos(X[3])+sin(X[2])*sin(X[3])*cos(X[1])),
    Tibia*(sin(X[0])*sin(X[1])*cos(X[2])-cos(X[0])*cos(X[1])*cos(X[2]))-Femur*cos(X[1])*cos(X[2]),
    1
  };
  return Y;
}



//LEGACY

//3x4 Jacob
float[][] fatJacobian = new float[3][4];
//square 3x3, dropped theta4 column
float[][] sqrJacobian = new float[3][3];

//3x4 jacobian of the foot in function of theta, from the t1,t2,t3,t4 of the specific leg
float[][] LegJacobian(float[] t){
  float[][] J=new float[3][4];
  
  float s1,s2,s3,s4,c1,c2,c3,c4;
  s1=sin(t[0]); s2=sin(t[1]); s3=sin(t[2]); s4=sin(t[3]);
  c1=cos(t[0]); c2=cos(t[1]); c3=cos(t[2]); c4=cos(t[3]);
  
  J[0][0]=Tibia*c1*(s4*c2+s2*s3*c4)+Tibia*s1*(s3*c2*c4-s2*s4);
  J[1][0]=Tibia*c1*(c4*c2-s2*s3*s4)+Tibia*s1*(-s3*c2*s4-s2*c4);
  J[2][0]=Tibia*c1*s2*c3+Tibia*s1*c3*c2;
  
  J[0][1]=Tibia*c2*(s1*s3*c4+s4*c1)+Tibia*s2*(s3*s1*s4-s1*s4)+c2*Femur*s4+s2*Femur*s3*c4;
  J[1][1]=Tibia*c2*(c1*c4-s1*s3*s4)+Tibia*s2*(-s1*c4-s3*s4*c1)+c2*Femur*c4-s2*Femur*s3*s4;
  J[2][1]=Tibia*c2*s1*c3+Tibia*s2*c1*c3+s2*Femur*c3;
  
  J[0][2]=Tibia*c3*(s1*s2*c4-c1*c2*c4)-c3*Femur*c2*c4;
  J[1][2]=Tibia*c3*(s4*c1*c2-s1*s2*s4)+c3*Femur*s4*c2;
  J[2][2]=Tibia*s3*(c1*c2-s1*s2)+s3*Femur*c2;
  
  J[0][3]=Tibia*c4*(s1*c2+s2*c1)+Tibia*s4*(s3*c1*c2-s1*s2*s3)+c4*Femur*s2+s4*Femur*s3*c2;
  J[1][3]=Tibia*c4*(s3*c1*c2-s1*s2*s3)+Tibia*s4*(-s1*c2-s2*c1)+c4*Femur*s3*c2-s4*Femur*s2;
  J[2][3]=0;
  
  return J;
}


//invert the LegJacobian, by specifying a fixed theta coordinate and its derivative
float[] InverseJacobianTransf(float[] t, float[] dx, int dtfixIndex, float dtfix){
  float[][] J = LegJacobian(t);
  float[][] J3 = new float[3][3];
  for(int i=0; i<dtfixIndex; i++){
    for(int j=0; j<3; j++){
      J3[i][j]=J[i][j];
    }
  }
  for(int i=dtfixIndex+1; i<4; i++){
    for(int j=0; j<3; j++){
      J3[i-1][j]=J[i][j];
    }
  }
  
  float[] v={dx[0]-J[0][dtfixIndex]*dtfix,dx[1]-J[1][dtfixIndex]*dtfix,dx[2]-J[2][dtfixIndex]*dtfix};
  
  return VTransf(v,Invert(J3,Det(J3)));
}
