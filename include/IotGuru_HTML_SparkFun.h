void initializeESP8266();
void connectESP8266();
void displayConnectInfo();

void IoTguruSend(char *nodeID,char *FieldName,float Value);
// errorLoop prints an error code, then loops forever.
void errorLoop(int error);
// serialTrigger prints a message, then waits for something
// to come in from the serial port.
void serialTrigger(String message);