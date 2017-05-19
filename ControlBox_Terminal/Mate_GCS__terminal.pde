/*

!IMPORTANT = If you want to activate the client connection you need to change DEBUG variable to false


What is UI:
  UI is a monitor that shows coming datas from client except errors.

What is Terminal:
  Terminal shows error datas from client or this program. 


Errors:
  WARNING! UI datas are too long. You may not see all datas                 if you see this error, input string that come from client has too part more than UI_String
  WARNING! Debugging mode enabled.                                          
  WARNING! Cannot send data to mate.                                        When debugging mode enable, you see that.
  
  

   
   **************************************************************
   *                                *                           *
   *      MATE POSITION             *          UI DATAS         *
   *                                *                           *
   **************************************************************
   *                        TERMINAL                            *
   *                                                            *
   **************************************************************
*/


import processing.net.*; 

Client myClient; 
/////////////////////////////////////////////////////////////////VARIABLES
String terminalString[] = new String[8];
int terminalColor[] = new int[8];
PFont font;
String[] str;
float x = 0,y = 0,z = 0;
float pos = 0;
int UI_counter = 0;
String UI_String[] = new String[16];
boolean DEBUG = true, dataDefined = false, validCommand = false;
String ipAddress = "192.168.2.98";
String logFile = "logs/sysLog.txt";
String command = "";
///////////////////////////////////////////////////////////////





void setup(){
  

  
  font = createFont( "Ubuntu", 16, true );
  size(600,400);
  
  //////////////To clear Terminal String
  for( int a = 0; a < terminalString.length; a++){
    terminalString[a] = "";
  }
  terminal("system@"+ ipAddress + ": System was started, [ "+day()+"/"+month()+"/"+year()+" "+hour()+" : "+minute()+" ]",1);
  log("System was started");
  
  //////////////Client Connection
  if(!DEBUG){
    myClient = new Client(this, ipAddress, 5000); 
    if(myClient.active()){
      terminal("system@"+ ipAddress + ": Connection established. Ip address: "+ipAddress,1); 
      log("Connection established. Ip address: "+ipAddress);
    }
  } 
  else{
    terminal("WARNING! Debugging mode enabled.",4);
  }
  background(255);
  parse("");
}

void draw(){
  if(!DEBUG){
    if (myClient.available() > 0) {
      String dataIn = myClient.readString();
      dataIn = dataIn.substring(dataIn.indexOf("#")+1,dataIn.indexOf("%"));
      println(dataIn);
      parse(dataIn); 
    } 
  }
  background(220);
  stroke(0);
  strokeWeight(4);
  line(3*width/5,0,3*width/5,2*height/3);
  stroke(0,224,0);
  line(0,2*height/3,width,2*height/3);
  fill(0);
  noStroke();
  rect(0,2*height/3,width,height);
  textFont(font);
  fill(0,224,0);
  text(">>  pilot@"+ ipAddress + ": " + command,width/30,11*height/15);
  for( int a = 0; a < terminalString.length; a++ ){
    if(terminalString[a] != ""){
      switch(terminalColor[a]){
        case 0:
          fill(224,0,0);
          break;
        case 1:
          fill(0,224,0);
          break;
        case 2:
          fill(0,0,224);
          break;
        case 3:
          fill(255,255,255);
          break;
        case 4:
          fill(255,143,31);
          break;
      }
      text(">>  "+terminalString[a],width/30,9*height/12+a*15+10); 
    }
  }
////////////////////////////////////////////////////////////////PRINT UI DATAS
  fill(0,0,224);
  for( int a = 0; a<UI_String.length; a++){
    if(UI_String[a] != ""){
      text(UI_String[a],25*width/40,a*16+20);
    }
  }
  
}
void parse(String dataIn){
  //This function is parse the input data called dataIn.
  //To parse input data from client you can use this function
  
  
  str =   splitTokens(dataIn,"|");
  UI_counter = 0;
  dataDefined = false;
  for( int a = 0; a < UI_String.length; a++){
    UI_String[a] = "";
  }
  for( int a = 0; a<str.length; a++){
    if(str[a].length()>6){
      if(str[a].substring(0,6).equals("error:")){
        if(DEBUG)println("ERROR!:"+str[a].substring(6));
        terminal( "mate@" + ipAddress + ": " + str[a].substring(6),0);
        dataDefined = true;
      }
    }
    if(str[a].length()>4){
      if(str[a].substring(0,6).equals("log:")){
        if(DEBUG)println("LOG!:"+str[a].substring(4));
        log(str[a].substring(4));
        dataDefined = true;
      }
    }
    if(str[a].length()>2){
      if(str[a].substring(0,2).equals("x:")){
        x = float(trim(str[a].substring(2)));
        dataDefined = true;
      }
      else if(str[a].substring(0,2).equals("y:")){
        y = float(trim(str[a].substring(2)));
        dataDefined = true;
      }
      else if(str[a].substring(0,2).equals("z:")){
        z = float(trim(str[a].substring(2)));
        dataDefined = true;
      }
    }
    if(!dataDefined){
        if(UI_counter<UI_String.length-3){
          UI_String[UI_counter+3] = str[a];
        }
        else{
          terminal("system@"+ ipAddress + ": WARNING! UI datas are too long. You may not see all datas ",4);
        }
        UI_counter++;
     }
  }
  UI_String[0] = "Roll: "+str(x);
  UI_String[1] = "Pitch: "+str(y);
  UI_String[2] = "Yaw: "+str(z);

}

void terminal(String data,int colorData){
  /*
   Color variables for Terminal:
   0: RED
   1: GREEN
   2: BLUE
   3: WHITE
   4: ORANGE
 
  To add a terminal output You can use terminal function
  */
  
  
  for( int a = terminalString.length-1; a > 0; a--){
    terminalString[a] = terminalString[a-1];
    terminalColor[a] = terminalColor[a-1];
  }
  terminalString[0] =  data;
  terminalColor[0] =  colorData;
  
}
void keyPressed() {
    if(keyCode == ENTER ){
      if(command.length()<1)command = " ";
      terminal("pilot@"+ ipAddress + ": " + command,1);
      exec(command);
      command = "";
    }
    else{
      if(keyCode == BACKSPACE){
        if(command.length()>0)command = command.substring(0,command.length()-1);
      }  
      else{
        command = command + str(key);
      }
    }
}
void exec(String command){
  validCommand  = false;
  if(command.length()>3){
    if(command.substring(0,4).equals("date")){
      String date = day() + "/" + month() + "/" + year() + " " + hour() + ":" + minute() + ":" + second();
      terminal(date,3);
      validCommand = true;
    }
  }
  if(command.length()>5){
    if(command.substring(0,6).equals("millis")){
      terminal(str(millis()),3);
      validCommand = true;
    }
  }
  if(command.length()>4){
    if(command.substring(0,5).equals("clear")){
      for( int a = 0; a < terminalString.length; a++){
        terminalString[a] = "";
      }
      validCommand = true;
    }
  }
  if(command.length()>5){
    if(command.substring(0,4).equals("send")){
      
      sendData(command.substring(5));
      validCommand = true;
    }
  }
  if(!validCommand){
    terminal("Invalid Syntax: "+command,0);
  }
}

void log(String logData){ 
  
  /*
    This function save log data to logFile variable
  */
  String[] oldData = loadStrings(logFile);
  String[] data = new String[oldData.length+1];
  String date = "[" + day() + "/" + month() + "/" + year() + " " + hour() + ":" + minute() + ":" + second() +"] ";
  for(int a = 0; a<oldData.length;a++){
    data[a] = oldData[a];
  }
  data[data.length-1] = date + logData;
  saveStrings(logFile,data); 
}

void sendData(String data){
  if(!DEBUG){
    myClient.write(data);
    terminal("Data: " + data,3);
    terminal("system@" + ipAddress +" : Data was sended",3);
  }
  else{
    terminal("WARNING! Cannot send data to mate.",4);
  }
}