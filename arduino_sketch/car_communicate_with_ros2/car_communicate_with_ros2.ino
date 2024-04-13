// Receive twist vel_lin_x and vel_ang_z

const byte numChars = 32;
char receivedChars[numChars];
char temp_chars_[numChars];        // temporary array for use when parsing

// variables to hold the parsed data
char label_lin_[numChars] = {0};
char label_ang_[numChars] = {0};
float twist_lin_ = 0.0;
float twist_ang_ = 0.0;
boolean newData = false;

String debug_string_;


void setup() {
    Serial.begin(9600);
    //Serial.println("Enter data in this style <HelloWorld, 12, 24.7>  ");
}

void loop() {
    recvWithStartEndMarkers();
    if (newData == true) {
        strcpy(temp_chars_, receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() used in parseData() replaces the commas with \0
        parseData();
        showParsedData();
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
    
    debug_string_ = String(label_lin_) + String(": ") + String(twist_lin_, 3) + String(" --- ") + String(label_ang_) + String(": ") + String(twist_ang_, 3) ; 

}

//============

void showParsedData() {
    Serial.println(debug_string_);
}
