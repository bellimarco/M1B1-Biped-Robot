String indexHTML = 
R"(
<!DOCTYPE html>
<html>
    <head>
        <title>M1B1 Web Page</title>
        <style>
            * {font-size: 100%;;}
            button {margin: 0.3em;}
            #joystickContainer button {width:5em; height:5em; background-image:contain;}
            .myBox {padding:0.5em;border:0.2em solid #444444; margin:0.2em;}

        </style>
    </head>
    
    <body style="margin:0.5%;">
        <div style="display:inline-block;vertical-align:top;margin:0">
            <div class="myBox">
                <h3>Controls:</h3>
                <div style="display:inline-block">
                    <input type="text" id="CustomGcode" placeholder="custom Gcode" style="display:block;width:7em;height:2em;margin:0.4em">
                    <button id="StandButton" style="display:block;width:5em;height:2em;margin:0.4em">Stand</button>
                    <button id= "SitButton" style="display:block;width:5em;height:2em;margin:0.4em">Sit</button>
                    <button id="StopButton" style="display:block;width:5em;height:2em;margin:0.4em">Stop</button>
                </div>
                <div style="display:inline-block;vertical-align:top;margin-left:2em">
                    <select id="ControlMode" style="height:2em;display: inline-block;">
                        <option value="1">Indefinite</option>
                        <option value="0">Incremental</option>
                    </select>
                    <p id="ControlModeUnit" style="margin-left:1em;display: inline-block;">Speed[mm/s]:</p>
                    <input type="text" id="ControlModeInput" value="10" style="height:2em;width:4em;display: inline-block;">
                    <div style="display:flex;flex-direction:column;" id="joystickContainer">
                        <div style="display:flex">
                            <button id="CW">CW</button>
                            <button id="FORW">forw</button>
                            <button id="RUNFORW">FORW</button>
                        </div>
                        <div style="display:flex">
                            <button id="LEFT">left</button>
                            <button id="nothing" style="visibility:hidden;"></button>
                            <button id="RIGHT">right</button>
                        </div>
                        <div style="display:flex">
                            <button id="CCW">CCW</button>
                            <button id="BACK">back</button>
                            <button id="RUNBACK">BACK</button>
                        </div>
                    </div>
                </div>
            </div>
            <div class="myBox">
                <h3>Display:</h3>
                <div><h5>X position:</h5><p id="Xpos">50.4</p></div>
                <button id="DisplayUpdate">update</button>
            </div>
        </div>
        <div class="myBox" style="display:inline-block;vertical-align:top;">
            <h3>Gcode Buffer:</h3>
            <div id="GcodeExecute">
            </div>

            <h3>Last Gcodes executed:</h3>
            <div id="GcodeExecuted">
            </div>
            <br>
            <button id="GcodeUpdate">update</button>
        </div>
    </body>


    <script>
        var MCU_IP_ADDRESS = ""

        function httpGet(addr,res){
            var xmlHttp = new XMLHttpRequest()
            xmlHttp.open( "GET", addr)
            xmlHttp.send( null )
            xmlHttp.onload = ()=>{
                if (xmlHttp.status == 200) {  res(xmlHttp.response)  }
                else{  alert(`Error ${xmlHttp.status}: ${xmlHttp.statusText}`)  }
            }
        }

        var GcodeExecute = document.getElementById("GcodeExecute")
        var GcodeExecuted = document.getElementById("GcodeExecuted")


        var RUNFORW = document.getElementById("RUNFORW")
        var RUNBACK = document.getElementById("RUNBACK")

        var gcodeinput = document.getElementById("CustomGcode")
        gcodeinput.addEventListener("keyup",(e)=>{if(e.keyCode===13){  httpGet("Gcode/"+gcodeinput.value,(res)=>{ console.log(res)})  }})
        document.getElementById("StandButton").addEventListener("click",()=>{  httpGet("Gcode/ST_1",console.log)  })
        document.getElementById("SitButton").addEventListener("click",()=>{  httpGet("Gcode/ST_0",console.log) })
        document.getElementById("StopButton").addEventListener("click",()=>{  httpGet("Gcode/CC_0",console.log) })

        var controlmode = document.getElementById("ControlMode")
        var controlmodeinput = document.getElementById("ControlModeInput")
        controlmode.addEventListener("input",(e)=>{
            document.getElementById("ControlModeUnit").innerHTML = (controlmode.value == "0")?"Step[mm]:":"Speed[mm/s]:"
            RUNFORW.style.cssText = "opacity:"+((controlmode.value == "0")?"0.5":"1")+";pointer-events:"+((controlmode.value == "0")?"none":"auto")
            RUNBACK.style.cssText = "opacity:"+((controlmode.value == "0")?"0.5":"1")+";pointer-events:"+((controlmode.value == "0")?"none":"auto")
        })

        document.getElementById("FORW").addEventListener("click",()=>{  httpGet("Gcode/"+((controlmode.value == "0")?"Ws_0_":"Wi_0_")+controlmodeinput.value,console.log)  })
        document.getElementById("BACK").addEventListener("click",()=>{  httpGet("Gcode/"+((controlmode.value == "0")?"Ws_0_-":"Wi_0_-")+controlmodeinput.value,console.log)  })
        document.getElementById("RIGHT").addEventListener("click",()=>{  httpGet("Gcode/"+((controlmode.value == "0")?"Ws_1_":"Wi_1_")+controlmodeinput.value,console.log)  })
        document.getElementById("LEFT").addEventListener("click",()=>{  httpGet("Gcode/"+((controlmode.value == "0")?"Ws_1_-":"Wi_1_-")+controlmodeinput.value,console.log)  })

        RUNFORW.addEventListener("click",()=>{  httpGet("Gcode/Ri_"+controlmodeinput.value,console.log)  })
        RUNBACK.addEventListener("click",()=>{  httpGet("Gcode/Ri_-"+controlmodeinput.value,console.log)  })

        document.getElementById("CW").addEventListener("click",()=>{  httpGet("Gcode/"+((controlmode.value == "0")?"Ts_":"Ti_")+controlmodeinput.value,console.log)  })
        document.getElementById("CCW").addEventListener("click",()=>{  httpGet("Gcode/"+((controlmode.value == "0")?"Ts_-":"Ti_-")+controlmodeinput.value,console.log)  })

        document.getElementById("DisplayUpdate").addEventListener("click",()=>{  httpGet("DisplayUpdate",console.log)  })
        
        document.getElementById("GcodeUpdate").addEventListener("click",GcodeUpdate)
        function GcodeUpdate(){  httpGet("GcodeUpdate",(res)=>{
            GcodeExecute.innerHTML=""
            GcodeExecuted.innerHTML=""

            var foundsemi=false
            var code=""
            for(var i=0; i<res.length; i++){
                if(!foundsemi){
                    if(res[i]!=";"){
                        if(res[i]!=","){ code+=res[i] }
                        else{ GcodeExecute.innerHTML+="<p>"+code+"</p>"; code="" }
                    }else{
                        foundsemi=true
                    }
                }else{
                    if(res[i]!=","){ code+=res[i] }
                    else{ GcodeExecuted.innerHTML+="<p>"+code+"</p>"; code="" }
                }
            }

            console.log("Updated Gcode buffers")
        })}

        setInterval(GcodeUpdate,500);


    </script>
</html>
  )";