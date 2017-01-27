#include "main.h"

//Returns number parsed from character buffer
long parseNumber(const char *numberString) {
    char digit;

    int len = strlen(numberString);

    //Catch empty string
    if (len == 0) {
        char errorMessage[100];
        sprintf(errorMessage, "Error, unable to parse number: %s", numberString);
        writeJINXData("Error", errorMessage);
    }

    for (int i = 0; i < len; i++) {
        digit = numberString[i];
        if (((digit < '0') || (digit > '9')) && (digit != '-') && (digit != '.')) {
            char errorMessage[100];
            sprintf(errorMessage, "Error, unable to parse number: %s", numberString);
            writeJINXData("Error", errorMessage);
            return -1;
        }
    }

    return atol (numberString);
}

void writeJINXDataNumeric (const char *name, double value) {
    // if (strlen(name) + strlen(value) >= MAX_MESSAGE_SIZE + PROTOCOL_SIZE) {
    //     fprintf(comPort, "Warning: Tried to send too large a message named %s", name);
    //     return;
    // }
    char buffer[50];
    sprintf (buffer, "%f", value);
    buffer[49] = '\0';
    writeJINXData (name, buffer);
    buffer[0] = '\0';
}

void parseMessage (JINX *inStr) {
    //Echo entire recieved message
    //writeJINXMessage(inStr->command);

    //Set inStr->token to first token (space-delimated word)
    getToken (inStr, 0);

    if (strcmp (inStr->token, "kill") == 0) {
        armTarget = -1;
        clapperTarget = -1;
    }
    else if (strcmp (inStr->token, "armTarget") == 0) {
        getToken (inStr, 1);
        armTarget = parseNumber (inStr->token);
    }
    else if (strcmp (inStr->token, "armKpUp") == 0) {
        getToken (inStr, 1);
        armKpUp = parseNumber (inStr->token);
    }
    else if (strcmp (inStr->token, "armKpDown") == 0) {
        getToken (inStr, 1);
        armKpDown = parseNumber (inStr->token);
    }
    else if (strcmp (inStr->token, "clapperTarget") == 0) {
        getToken (inStr, 1);
        clapperTarget = parseNumber (inStr->token);
    }
    else if (strcmp (inStr->token, "clapperKp") == 0) {
        getToken (inStr, 1);
        clapperKp = parseNumber (inStr->token);
    }
    else {
        writeJINXMessage ("Invalid command:");
        writeJINXMessage (inStr->token);
    }
}
