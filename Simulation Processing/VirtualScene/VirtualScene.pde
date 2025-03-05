import peasy.*; //<>//

//seconds duration of the world ticker
int FPS=400;
float loopTick = 1.0/FPS;
int DisplayTimer=8;
int DisplayTimerCount=0;
float K=1000;            //pixel/meter


//timekeeping
float TimeScale = 0.1;    //scales directly WorldTime
double WorldTime = 0;   //simulated seconds since start of program
//RunTime timer
double LastTick = 0;
double CurrentTick = 0;
float ElapsedT = 0;
boolean Running = false;
void RunTime() {
  if (Running) {
    
    WorldTime += loopTick*TimeScale;
    
    CurrentTick = WorldTime;
    ElapsedT = (float)(CurrentTick-LastTick);
    LastTick=CurrentTick; 

    //measure the outside world and update the linked C and P space with it
    //RealSense.Update();
    //compute actuators
    RealController.Update(ElapsedT);

    //run physics simulation
    //update Real P and C accordingly
    RealP.Tick(ElapsedT);
  }
}

void keyPressed() {
  if (key == 't') {
    TimeScale=1;
    Running = true;
    LastTick = WorldTime;
  } else if (key == 'z') {
    TimeScale=0.1;
    Running = true;
    LastTick = WorldTime;
  } else if (key == 'd') {
    print("Debugger");
  } else {
    Running = false;
  }
}
void keyReleased() {
  if (key == 'z') {
    Running = false;
  }
}



PeasyCam cam;
void setup() {
  size(1400, 1000, P3D);
  frameRate(FPS);
  cam = new PeasyCam(this, 0, 0, 0, 1000);

  cam.setMinimumDistance(20);
  cam.setMaximumDistance(2000);
  cam.setRotations(0.8, 1, 3.9); // rotations are applied in that order


  SetupCspaceConfigs();
  SetupFrames();
  SetupPhysics();
  SetupRobotics();
  
}

void draw() {

  RunTime();

  //display things
  DisplayTimerCount++;
  if (DisplayTimer<DisplayTimerCount) {
    DisplayTimerCount=1;

    background(220);
    scale(K, -K, K);
    directionalLight(255, 255, 255, 0, 0, 1);

    //displayJoints(RealC);
    displayMass(RealC);
    displayFrames(RealC);
    displayPhysics(RealP);
    displayController(RealController);
  }
}
