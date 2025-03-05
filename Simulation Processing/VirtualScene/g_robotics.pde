
class RawSense{

    float[] BodyQuaternion = {1,0,0,0};

    float TOF1 = 0;
    float TOF2 = 0;
    float TOF3 = 0;
    float TOF4 = 0;

    //int encoders


    void Update(){

    }
}

class Sense{

    //linked C and P spaces
    Cspace Cspc;
    Pspace Pspc;

    //COM frame
    float[] P={0,0,0};    //linear position
    float[] A={0,0,0};    //angular position
    float[] V={0,0,0};    //linear velocity
    float[] W={0,0,0};    //angular velocity
    float[][] Tcom_s = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
    float[][] Ts_com = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};

    Sense(Cspace c, Pspace p){
        Cspc = RealC; Pspc = RealP;
    }

    //measure the outside world and update the linked C and P space with it
    void Update(){
        

        Cspc.UpdateTcoms(Tcom_s,Ts_com);
        Pspc.UpdateParams(Tcom_s,Ts_com,P,A,V,W);
    }

}
Sense RealSense;



class Controller{
    Cspace Cspc;
    Pspace Pspc;

    //COM target parameters
    boolean TargDefined = false;
    float[] Ptarg = {0,0,0}; float[] Atarg = {0,0,0};
    float[] Vtarg = {0,0,0}; float[] Wtarg = {0,0,0};

    //target position, Ptarg + PIDofA
    float[] p_targ = {0,0,0};
    PIDobj3D PtargPIDofA = new PIDobj3D(0.006,0.00,0.01); //shifts position given error orientation
    //target velocity, Vtarg + PIDofP + PIDofW
    float[] v_targ = {0,0,0};
    PIDobj3D VtargPIDofP = new PIDobj3D(-10,0,-3); //velocity component of position error
    PIDobj3D VtargPIDofW = new PIDobj3D(0,0,0);     //velocity component of angular speed error
    //target acceleration, PIDofV
    float[] dv_targ = {0,0,0};
    PIDobj3D DVtargPIDofV = new PIDobj3D(-16,0,0); //acceleration given error velocity
    
    //note: PID controllers of angular errors are actually based on (angularVect cross Pivot)/Pivot^2 error vectors
    //  this way they yield directly their contribution to the linear position, velocity

    //Target acting force on feet
    float[] FactR = {0,0,0};
    float[] FactL = {0,0,0};
    float[] FactRnorm = {0,0,0};
    float[] FactLnorm = {0,0,0};
    float[] FactRtan = {0,0,0};
    float[] FactLtan = {0,0,0};

    Controller(Cspace c, Pspace p){
        Cspc = c; Pspc = p;
    }

    void SetTarget(float[] p, float[] a, float[] v, float[] w){
        TargDefined = true;
        VCopy(Ptarg,p); VCopy(Atarg,a); VCopy(Vtarg,v); VCopy(Wtarg,w);
        PtargPIDofA.SetTarget(Atarg); VtargPIDofP.SetTarget(Ptarg);
        VtargPIDofW.SetTarget(Wtarg); DVtargPIDofV.SetTarget(Vtarg);
    }

    void Reset(){
        TargDefined = false;
        VNull(Ptarg); VNull(Atarg); VNull(Vtarg); VNull(Wtarg);
        PtargPIDofA.Reset(Pspc.A); VtargPIDofP.Reset(Pspc.P);
        VtargPIDofW.Reset(Pspc.W); DVtargPIDofV.Reset(Pspc.V);
    }
    
    //using the current linked C and P space,
    //  and given a target state of the COM, compute the desired force on the feet,
    //  and send it for execution to the force controller
    void Update(float dt){

        if(TargDefined){


            //TODO: include Fsupport calculation and subtract it from the desired Fact
            //  to get the real force needed on the motors,
            //  for now entire force is simulated by the motors
            if(Pspc.GravState==0){

                VNull(FactR); VNull(FactL);

            }else if(Pspc.GravState==1){

                //two feet pivot
                float lambda = (1- (VQuadr(Cspc.COMFootL)-VQuadr(Cspc.COMFootR)) / VQuadr(Cspc.FootRL) )/2;
                float[] Pivot = VSub(VAdd(Cspc.COMFootR,VScale(lambda,Cspc.FootRL)),Cspc.Tcom_s[3]);

                //run PID loops
                p_targ = VAdd( Ptarg, PtargPIDofA.Get(dt, VCross( VScale(VDot(Cspc.FootRL,Pspc.A)/VQuadr(Cspc.FootRL),Cspc.FootRL) , VScale(1/VQuadr(Pivot),Pivot )) ));
                
                VtargPIDofP.SetTarget(p_targ);
                v_targ = VAdd( Vtarg, VtargPIDofP.Get(dt, Pspc.P) , VtargPIDofW.Get(dt, VCross( VScale(VDot(Cspc.FootRL,Pspc.W)/VQuadr(Cspc.FootRL),Cspc.FootRL) , VScale(1/VQuadr(Pivot),Pivot )) ));

                DVtargPIDofV.SetTarget(v_targ);
                dv_targ = DVtargPIDofV.Get(dt, Pspc.V);

                //derive feet forces from desired COM acceleration
                FactR = VScale(-(1-lambda)*Cspc.Mass, VSub(dv_targ,Pspc.Laws.G));
                FactL = VScale(-lambda*Cspc.Mass, VSub(dv_targ,Pspc.Laws.G));

            }else if(Pspc.GravState==2){
    
                VNull(FactL);
                FactRnorm = VScale(Cspc.Mass*VDot(Pspc.Laws.G,Cspc.COMFootR)/VQuadr(Cspc.COMFootR),Cspc.COMFootR);
                FactRtan = VScale(Cspc.Itot(Cspc.Tcom_s[3],VNorma(VCross(Pspc.Laws.G,Cspc.COMFootR)))/(Cspc.Itot(Cspc.Tcom_s[3],VNorma(VCross(Pspc.Laws.G,Cspc.COMFootR)))+Cspc.Mass*VQuadr(Cspc.COMFootR)),VSub(VScale(Cspc.Mass,Pspc.Laws.G),FactRnorm));
                FactR = VAdd(FactRnorm,FactRtan);
    
            }else if(Pspc.GravState==3){

                VNull(FactR);
                FactLnorm = VScale(Cspc.Mass*VDot(Pspc.Laws.G,Cspc.COMFootL)/VQuadr(Cspc.COMFootL),Cspc.COMFootL);
                FactLtan = VScale(Cspc.Itot(Cspc.Tcom_s[3],VNorma(VCross(Pspc.Laws.G,Cspc.COMFootL)))/(Cspc.Itot(Cspc.Tcom_s[3],VNorma(VCross(Pspc.Laws.G,Cspc.COMFootL)))+Cspc.Mass*VQuadr(Cspc.COMFootL)),VSub(VScale(Cspc.Mass,Pspc.Laws.G),FactLnorm));
                FactL = VAdd(FactLnorm,FactLtan);

            }

            //send desired force to joint controllers
            //for now controller directly simulates the acting forces
            Pspc.UpdateFactRL(FactR,FactL);
        }
    }
}
Controller RealController;




void SetupRobotics(){

    RealSense = new Sense(RealC,RealP);

    RealController = new Controller(RealC,RealP);

    float[] comtarget = {0,0,RealC.Tcom_s[3][2]-0.1};
    VAddInplace(comtarget, VAdd(RealC.FootR,VScale(0.5,RealC.FootRL)) );

    RealController.Reset();
    RealController.SetTarget(comtarget,v0_3,v0_3,v0_3);
}
