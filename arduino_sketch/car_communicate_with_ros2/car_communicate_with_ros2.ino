// Receive twist vel_lin_x and vel_ang_z

const byte numChars = 32;
char receivedChars[numChars];
char temp_chars_[numChars];

// variables to hold the parsed data
char label_lin_[numChars] = {0};
char label_ang_[numChars] = {0};
float twist_lin_ = 0.0;
float twist_ang_ = 0.0;
boolean newData = false;

// Motors
const int A_1B = 5;
const int A_1A = 6;
const int B_1B = 9;
const int B_1A = 10;

String debug_string_;


void setup() {
    Serial.begin(9600);

    pinMode(A_1B, OUTPUT);
    pinMode(A_1A, OUTPUT);
    pinMode(B_1B, OUTPUT);
    pinMode(B_1A, OUTPUT);
}

void loop() {
    recvWithStartEndMarkers();
    if (newData == true) {
        strcpy(temp_chars_, receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() used in parseData() replaces the commas with \0
        parseData();
        motionCommand();
        debugShowData();
        newData = false;
    }
}

//============

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

//============

void parseData() {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(temp_chars_,",");      // get the first part - the string
    strcpy(label_lin_, strtokIndx);
 
    strtokIndx = strtok(NULL, ",");
    twist_lin_ = atof(strtokIndx);     // convert this part to a float

    strtokIndx = strtok(NULL, ",");
    strcpy(label_ang_, strtokIndx);
    
    strtokIndx = strtok(NULL, ",");
    twist_ang_ = atof(strtokIndx);     // convert this part to a float

    //strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    //integerFromPC = atoi(strtokIndx);     // convert this part to an integer
    
}

//============

void debugShowData() {
    Serial.println(debug_string_);
}

void motionCommand() {

  int vel_left, vel_right;
  calculate_speeds(twist_lin_, twist_ang_, vel_left, vel_right);
   
  motionAction(vel_left, vel_right);

  debug_string_ = String(label_lin_) + String(": ") + String(twist_lin_, 3) + String(" --- ") + String(label_ang_) + String(": ") + String(twist_ang_, 3) ; 
  debug_string_ = debug_string_ + " ** " + " vel_left: " + String(vel_left) + " vel_right: " + String(vel_right);
}

void calculate_speeds(float vlin, float vang, int &vleft, int &vright)
{
  float lwr = 0.05*1; // wheel radius
  float ws = 0.20; // wheel separation

  float vleft_float = ( twist_lin_ - twist_ang_*ws/2.0 )/ lwr;
  float vright_float = ( twist_lin_ + twist_ang_*ws/2.0 )/ lwr;

  // Calculate scaled values w.r.t. max value
  // consider: max value for vleft/vright=255 when vlin=1
  float vmax = 26.5;
  vleft = (int)(vleft_float/vmax * 255);
  vright = (int)(vright_float/vmax * 255);

  applyLimits(vleft, vright);  
}

void motionAction(int vleft, int vright)
{
  if (vleft >= 0)
  {
   analogWrite(A_1B, vleft);
   analogWrite(A_1A, 0);
  } else {
   analogWrite(A_1B, 0);
   analogWrite(A_1A, abs(vleft));    
  }

  if (vright >= 0)
  {
   analogWrite(B_1B, 0);
   analogWrite(B_1A, vright);
  } else {
    analogWrite(B_1B, abs(vright));
    analogWrite(B_1A, 0);
  }
}

void applyLimits(int &vleft, int &vright)
{
   if (vleft < -255)
     vleft = -255;
   if (vleft > 255)
     vleft = 255;
   if (vright < -255)
     vright = -255;
   if (vright > 255)
     vright = 255;
}
