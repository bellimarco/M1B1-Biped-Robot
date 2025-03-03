import processing.serial.*;
Serial myPort;      // The serial port
String portName="/dev/cu.usbmodem93356901";
String serialIn = "";    // Incoming serial data

boolean running = false;

int xsize = 2000;
float[] x = new float[xsize];
int xi = 0;

void setup() {
  size(600,600);
  myPort = new Serial(this, portName, 115200);

}

void draw() {
  background(220);
  
}


void keyPressed(){
  running=!running;
}

void serialIncoming(String in){
  if(running){
    if(xi%50==0){ print("."); }
    x[xi] = float(in);
    xi++;
    if(xi>=xsize){
      xi=0; running=false;
      
      float avrg = 0;
      for(int i=0; i<xsize; i++){
        avrg+=x[i];
      }
      avrg /= xsize;
      
      println(avrg);
    }
  }
}

void serialEvent(Serial myPort) {
  char c = char(myPort.read());
  if(c=='\n'){
    serialIncoming(serialIn);
    serialIn="";
    
  }else{
    serialIn+=c;
  }
}
