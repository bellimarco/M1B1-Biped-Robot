

//gcode buffers management
#define GcodeBuffersSize 10
String GcodeExecute[GcodeBuffersSize];  //confirmed by the teensy, in line for execution
byte GcodeExecuteHead = 0;
byte GcodeExecuteTail = 0;
String GcodeExecuted[GcodeBuffersSize];  //codes that have been executed
//byte GcodeExecutedHead = 0;   //not needed
byte GcodeExecutedTail = 0;

//gbuffer on the teensy
bool TeensyBufferFull = false;



void CheckGcode(String g){
    GcodeExecute[GcodeExecuteTail]=g;  //push the code to the execute buffer
    GcodeExecuteTail = (GcodeExecuteTail+1)%GcodeBuffersSize;
}
void ExecuteGcode(String g){
    //go through the Gexecute buffer, if find the code (which should be there) erase it and shift the other codes along with the array head
    for(byte i=0; i<GcodeBuffersSize; i++){
        if(GcodeExecute[(GcodeExecuteHead+i)%GcodeBuffersSize].equals(g)){
            for(byte j=i; j>0; j--){
                GcodeExecute[(GcodeExecuteHead+j)%GcodeBuffersSize] = GcodeExecute[(GcodeExecuteHead+j-1)%GcodeBuffersSize];
            }
            GcodeExecuteHead = (GcodeExecuteHead+1)%GcodeBuffersSize;
            break;
        }
    }


    GcodeExecuted[GcodeExecutedTail]=g;  //push the code to the executed buffer
    GcodeExecutedTail = (GcodeExecutedTail+1)%GcodeBuffersSize;
}

