
float frameMag = 0.03;
float forceK = 0.06/(LawsStd.g*3); float linvelK = 0.5; float angvelK = 0.1;


//color legend
color cmass=color(160,160,0);
color cjoint=color(50,50,50);
color clinvel=color(20,160,255,120);
color clinforce=color(20,160,255,255);
color cangvel=color(200,20,255,120);
color cangforce=color(200,20,255,255);
color ccontrol=color(250,20,250);


void displayJoints(Cspace c){
  //origin of every joint
  fill(cjoint); noStroke();
  for(int j=0; j<c.Joints; j++){
    translate(c.J[j].Ts[3][0],c.J[j].Ts[3][1],c.J[j].Ts[3][2]);
    sphere(0.006);
    translate(-c.J[j].Ts[3][0],-c.J[j].Ts[3][1],-c.J[j].Ts[3][2]);
  }
  //first child of every joint
  stroke(cjoint); strokeWeight(0.003);
  for(int j=0; j<c.Joints; j++){
    if(c.J[j].cnf.children[0]!=0){
      line(c.J[j].Ts[3][0],c.J[j].Ts[3][1],c.J[j].Ts[3][2]
          ,c.J[c.J[j].cnf.children[0]].Ts[3][0],c.J[c.J[j].cnf.children[0]].Ts[3][1],c.J[c.J[j].cnf.children[0]].Ts[3][2]);
    }
  }
}


void displayMass(Cspace c){
  fill(cmass);
  for(byte i=0; i<c.Joints; i++){
    for(byte j=0; j<c.J[i].cnf.masses; j++){
      if(c.J[i].Mass[j].cnf.type==1){
        noStroke();
        translate(c.J[i].Mass[j].p[0],c.J[i].Mass[j].p[1],c.J[i].Mass[j].p[2]);
        sphere(0.004+c.J[i].Mass[j].cnf.mass/1000000);
        translate(-c.J[i].Mass[j].p[0],-c.J[i].Mass[j].p[1],-c.J[i].Mass[j].p[2]);
      }else if(c.J[i].Mass[j].cnf.type==2){
        stroke(cmass); strokeWeight(0.002+c.J[i].Mass[j].cnf.mass/1000000);
        line(c.J[i].Mass[j].p[0],c.J[i].Mass[j].p[1],c.J[i].Mass[j].p[2],
             c.J[i].Mass[j].p[0]+c.J[i].Mass[j].l[0],
             c.J[i].Mass[j].p[1]+c.J[i].Mass[j].l[1],
             c.J[i].Mass[j].p[2]+c.J[i].Mass[j].l[2]);
      }
    }
  }
}

void displayFrames(Cspace c){
  strokeWeight(0.001);
  stroke(100);
  fill(150);
  rect(-1.5,-1.5,3.0,3.0);
  for(int i=-15; i<16; i++){
    line(1.5,0.1*i,-1.5,0.1*i);
    line(0.1*i,1.5,0.1*i,-1.5);
  }
  
  
  strokeWeight(0.003);
  for(int i=0; i<3; i++){
    if(i==0){ stroke(255,0,0);}
    else if(i==1){ stroke(0,255,0);}
    else{stroke(0,0,255);}
    
    
    line(0,0,0,(i==0?frameMag:0),(i==1?frameMag:0),(i==2?frameMag:0));
    for(byte j=0; j<c.Joints; j++){
      line(c.J[j].Ts[3][0],c.J[j].Ts[3][1],c.J[j].Ts[3][2],
           c.J[j].Ts[3][0]+c.J[j].Ts[i][0]*frameMag,
           c.J[j].Ts[3][1]+c.J[j].Ts[i][1]*frameMag,
           c.J[j].Ts[3][2]+c.J[j].Ts[i][2]*frameMag);
    }
  }

  
}

void displayPhysics(Pspace p){
  noStroke();
  //COM
  if(p.GravState==0){fill(250,20,250);}else if(p.GravState==1){fill(20,250,20);}
  else if(p.GravState==2){fill(250,20,20);}else if(p.GravState==3){fill(20,20,250);}
  translate(p.P[0],p.P[1],p.P[2]);

  sphere(0.005);

  strokeWeight(0.003);
  stroke(clinforce);
  //external force
  line(0,0,0,VScale(p.C.Mass,p.Laws.G)[0]*forceK,VScale(p.C.Mass,p.Laws.G)[1]*forceK,VScale(p.C.Mass,p.Laws.G)[2]*forceK);
  //acting force
  translate(p.C.COMFootR[0],p.C.COMFootR[1],p.C.COMFootR[2]);
  line(0,0,0,(p.FbackRnorm[0]+p.FbackRtan[0])*forceK,(p.FbackRnorm[1]+p.FbackRtan[1])*forceK,(p.FbackRnorm[2]+p.FbackRtan[2])*forceK);
  //line(0,0,0,p.FbackRnorm[0]*forceK,p.FbackRnorm[1]*forceK,p.FbackRnorm[2]*forceK);
  //line(0,0,0,p.FbackRtan[0]*forceK,p.FbackRtan[1]*forceK,p.FbackRtan[2]*forceK);
  translate(p.C.COMFootL[0]-p.C.COMFootR[0],p.C.COMFootL[1]-p.C.COMFootR[1],p.C.COMFootL[2]-p.C.COMFootR[2]);
  line(0,0,0,(p.FbackLnorm[0]+p.FbackLtan[0])*forceK,(p.FbackLnorm[1]+p.FbackLtan[1])*forceK,(p.FbackLnorm[2]+p.FbackLtan[2])*forceK);
  //line(0,0,0,p.FbackLnorm[0]*forceK,p.FbackLnorm[1]*forceK,p.FbackLnorm[2]*forceK);
  //line(0,0,0,p.FbackLtan[0]*forceK,p.FbackLtan[1]*forceK,p.FbackLtan[2]*forceK);
  translate(-p.C.COMFootL[0],-p.C.COMFootL[1],-p.C.COMFootL[2]);

  //line(0,0,0,(p.FbackRnorm[0]+p.FbackLnorm[0]+p.Fnorm[0])*forceK,(p.FbackRnorm[1]+p.FbackLnorm[1]+p.Fnorm[1])*forceK,(p.FbackRnorm[2]+p.FbackLnorm[2]+p.Fnorm[2])*forceK);
  line(0,0,0,p.Feff[0]*forceK,p.Feff[1]*forceK,p.Feff[2]*forceK);
  

  stroke(cangforce);
  line(0,0,0,p.TQeff[0]*forceK,p.TQeff[1]*forceK,p.TQeff[2]*forceK);

  stroke(clinvel);
  line(0,0,0,p.V[0]*linvelK,p.V[1]*linvelK,p.V[2]*linvelK);
  stroke(cangvel);
  line(0,0,0,p.W[0]*angvelK,p.W[1]*angvelK,p.W[2]*angvelK);
  
  translate(-p.P[0],-p.P[1],-p.P[2]);
/*
  translate(p.C.J[2].Ts[3][0],p.C.J[2].Ts[3][1],p.C.J[2].Ts[3][2]);
  line(0,0,0,p.FsuppR[0]*forceK,p.FsuppR[1]*forceK,p.FsuppR[2]*forceK);
  translate(p.C.J[6].Ts[3][0]-p.C.J[2].Ts[3][0],p.C.J[6].Ts[3][1]-p.C.J[2].Ts[3][1],p.C.J[6].Ts[3][2]-p.C.J[2].Ts[3][2]);
  line(0,0,0,p.FsuppL[0]*forceK,p.FsuppL[1]*forceK,p.FsuppL[2]*forceK);
  translate(-p.C.J[6].Ts[3][0],-p.C.J[6].Ts[3][1],-p.C.J[6].Ts[3][2]);
*/
  fill(170,40,40); noStroke();
  translate(p.GroundFootR[0],p.GroundFootR[1],p.GroundFootR[2]);
  sphere(0.007);
  translate(p.GroundFootL[0]-p.GroundFootR[0],p.GroundFootL[1]-p.GroundFootR[1],p.GroundFootL[2]-p.GroundFootR[2]);
  sphere(0.007);
  translate(-p.GroundFootL[0],-p.GroundFootL[1],-p.GroundFootL[2]);
/*
  fill(255,0,0); noStroke();
  translate(footTest[0],footTest[1],footTest[2]);
  sphere(0.01);
  translate(-footTest[0],-footTest[1],-footTest[2]);
*/
}

void displayController(Controller c){
  fill(ccontrol); stroke(ccontrol);

  /*line(c.Cspc.Tcom_s[3][0],c.Cspc.Tcom_s[3][1],c.Cspc.Tcom_s[3][2],
       c.Cspc.Tcom_s[3][0]+c.dv_targ[0],c.Cspc.Tcom_s[3][1]+c.dv_targ[1],c.Cspc.Tcom_s[3][2]+c.dv_targ[2]);
  */
  line(c.Cspc.Tcom_s[3][0],c.Cspc.Tcom_s[3][1],c.Cspc.Tcom_s[3][2],
       c.Cspc.Tcom_s[3][0]+c.v_targ[0],c.Cspc.Tcom_s[3][1]+c.v_targ[1],c.Cspc.Tcom_s[3][2]+c.v_targ[2]);

  noStroke();
  
  translate(c.Ptarg[0],c.Ptarg[1],c.Ptarg[2]);
  sphere(0.004);
  translate(-c.Ptarg[0],-c.Ptarg[1],-c.Ptarg[2]);
  translate(c.p_targ[0],c.p_targ[1],c.p_targ[2]);
  sphere(0.004);
  translate(-c.p_targ[0],-c.p_targ[1],-c.p_targ[2]);
}
