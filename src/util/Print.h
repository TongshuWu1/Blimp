//
// Created by dav on 2/9/24.
//


void printControlInput(ControlInput cmd){

    int n = sizeof(cmd.params)/ sizeof(cmd.params[0]);
    for(int i = 0; i < 5; ++i) {
        Serial.print(cmd.params[i]); // Print each byte in hexadecimal
        Serial.print(" "); // Print a space between bytes for readability
    }
    Serial.println();
}