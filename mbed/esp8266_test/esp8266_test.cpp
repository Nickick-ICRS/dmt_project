#include "mbed.h"
#include "ATParser.h"

#include <string>

Serial pc(USBTX, USBRX);
// ESP-01 tx and rx pins
BufferedSerial *esp_bs;
// AT command parser
ATParser *esp;

DigitalOut reset(PA_0);

#define REMOTE_IP "10.42.0.1"
#define REMOTE_PORT 25565 /*53821*/
#define DEFAULT_TIMEOUT 50

char ssid[32] = "nickick-laptop";
char pwd[32] = "auSEtdYV";

bool connect_to_network() {
    bool ret = false;
    
    // Longer timeout
    esp->setTimeout(8000);
    
    // Try to connect
    if(esp->send("AT+CWJAP=\"%s\",\"%s\"", ssid, pwd) && 
         esp->recv("OK"))
        ret = true;
    
    // Reset timeout
    esp->setTimeout(DEFAULT_TIMEOUT);

    if(ret)
        pc.printf("Successfully connected to \"%s\".\n", ssid);
    else
        pc.printf("Failed to connect to \"%s\".\n", ssid);

    return ret;
}

// Make a single TCP connection - multiple connections are not allowed
// Returns success // fail
bool connect_TCP() {
    bool ret = false;
    
    // Single TCP connection
    esp->send("AT+CIPMUX=0") && esp->recv("OK");

    // Longer timeout for this
    esp->setTimeout(1000);

    // Make the connection
    if((esp->send("AT+CIPSTART=\"TCP\",\"%s\",%d", REMOTE_IP, REMOTE_PORT)
         && esp->recv("ALREADY CONNECTED"))) {
        pc.printf("Already Connected\n");
        ret = true;
    }
    else {
        // If the connection was closed by the server we have to close it
        // Before we can open a new one, causing the above command to fail
        esp->send("AT+CIPCLOSE") && esp->recv("OK");

        // Try to connect again
        if((esp->send("AT+CIPSTART=\"TCP\",\"%s\",%d", REMOTE_IP, REMOTE_PORT)
             && esp->recv("OK"))) {
            pc.printf("Connection closed and reconnected\n");
            ret = true;
        }
    }

    // Reset timeout
    esp->setTimeout(DEFAULT_TIMEOUT);

    if(ret) 
        pc.printf("Successfully connected to \"%s\".\n", REMOTE_IP);
    else
        pc.printf("Failed to connect to \"%s\".\n", REMOTE_IP);

    return ret;
}

bool check_connection_status() {
    bool ret = false;

    esp->send("AT+CIPSTATUS");
    int status = 0;
    esp->recv("STATUS:%d", &status);

    pc.printf("Status: %d", status);

    switch(status) {
    // TCP or UDP connection
    case 3:
        pc.printf("Status ok\n");
        ret = true;
        break;
    // Connected to network and IP obtained - not connected to TCP
    case 2:
    // TCP or UDP is disconnected
    case 4:
        pc.printf("TCP disconnected...");
        // Try connecting to network, as occasionally this status is thrown when network connection failed too
        if(!connect_TCP())
            pc.printf("Lost connection with \"%s\".\n", REMOTE_IP);
        else {
            pc.printf("Reconnected.\n");
            ret = true;
        }
        break;
    // Not connected to a network
    case 5:
        pc.printf("Not connected to network...");
        if(!connect_to_network())
            pc.printf("Lost connection with \"%s\".\n", ssid);
        else if(!connect_TCP())
            pc.printf("Connected to network - Failed to connect to \"%s\".\n", REMOTE_IP);
        else {
            pc.printf("Connected.");
            ret = true;
        }
        break;
    }
    
    return ret;
}

void setup() {
    // Hardware reset
    reset = 0;
    
    // Setup PC communication
    pc.baud(115200);
    pc.printf("\n---------- Setting Up ESP8266 ----------\n");

    wait(1);

    // Finish reset
    reset = 1;
    
    // Wait for crazy stuff to disappear
    wait(3);
    
    // Setup serial and parser
    esp_bs = new BufferedSerial(PA_9, PA_10);
    esp = new ATParser(*esp_bs);

    // Set timeout
    esp->setTimeout(DEFAULT_TIMEOUT);

    // Set baud rate of esp
    esp_bs->baud(115200);

    // Check that all is ok
    while(!(esp->send("AT") && esp->recv("OK")));
    pc.printf("ESP8266 ok :)");
    
    // Turn off AT command echoing
    esp->send("ATE0") && esp->recv("OK");

    while(!connect_to_network());
    while(!connect_TCP());

    // Check status
    while(!check_connection_status());

    // No-longer send AT commands, just send data
    //esp->send("AT+CIPSEND");


    pc.printf("\n---------- Setup Complete ----------\n");
}

int main() {
    setup();
    char buf[256];
    int len;

    const char response[] = { 0xFF, 0x05, 0x07, 0x00, 0x00, 0x00, 0x00, 0xFE };
    
    // Set the timeout to be very low now as we are looking for many 
    // different messages
    // esp->setTimeout(5);

    Timer send_timer;
    send_timer.start();

    int send_delay_ms = 100;

    esp->send(response);
    while(true) {
        // Read incoming TCP messages if existant
        if(esp->recv("+IPD,%d:", &len)) {
            memset(buf, '\0', len);
            int data = esp->read(buf, len);
            pc.printf("Received: %.*s\n", data, buf);
        }


        // Send response
        if(send_timer.read_ms() > send_delay_ms) {
            esp->send("AT+CIPSEND=%d", sizeof(response)/sizeof(char));
            if(esp->recv(">")) {
                // Can't use send here as null characters end the string
                esp->write(response, sizeof(response)/sizeof(char));
                pc.printf("Sending message...");
            }
            send_timer.reset();
        }
    }
}
