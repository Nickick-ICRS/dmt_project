#include "web_client.h"

WebClient::WebClient(PinName tx, PinName rx, PinName reset, 
                     CommandProcessor *cmd)
        :reset_(reset), error_(""), cmd_(cmd),
         esp_bs_(tx, rx), esp_(esp_bs_, "\r\n", 128),
         recv_led_(LED3) {
    recv_led_ = 1;
    // Hardware reset
    reset_ = 0;

    wait(1);

    // Finish reset
    reset_ = 1;

    // Wait for initial status message to disappear
    wait(3);

    // Set timeout
    esp_.setTimeout(DEFAULT_TIMEOUT);

    // Set the baud rate - the default baud is 115200
    esp_bs_.baud(115200);

    // Wait for ESP-01
    while(!(esp_.send("AT") && esp_.recv("OK"))) {
        //pc.printf("waiting for esp...");
    }
    pc.printf("ESP OK");

    // Turn off AT command echoing
    esp_.send("ATE0") && esp_.recv("OK");

    connect_to_network();
    connect_TCP();

    // Wait for connection
    while(!check_connection_status()) {
        auto err = get_error();
        if(err != "")
            pc.printf("%s", err.c_str());
    }
    // pc.printf("connected");
    recv_led_ = 0;
}

WebClient::~WebClient() {
    // Free memory
    // delete esp_;
    // delete esp_bs_;
}

std::string WebClient::get_error() {
    // Copy the error
    std::string ret = error_;

    // Error has been processed so reset
    error_ = "";

    return ret;
}

void WebClient::send(std::string msg) {
    // Prepare ESP for  sending data
    esp_.send("AT+CIPSEND=%d", msg.size()) && esp_.recv("OK");

    // Send the message - can't use send here in case of '\0' characters
    esp_.write(msg.c_str(), msg.length());
    return;
}

void WebClient::update() {
    int len;
    // Read incoming TCP messages if existant
    if(esp_.recv("+IPD,%d:", &len)) {
        recv_led_ = !recv_led_;
        esp_.read(buf_, len);
        // Add data read to the storage string
        for(char i = 0; i < len && i < 64; i++) {
            data_ += buf_[i];
        }
    }

    // Check if a full message has been received yet
    if(data_.size() > 0) {
        bool started = false;
        char start;
        char end;
        std::string message;
        for(char i = 0; i < data_.size(); i++) {
            // If there's a start of message byte and we haven't started yet
            if(data_[i] == Msgs::MSG_STRT && !started) {
                start = i;
                started = true;
            }
            // If there's an end of bessage byte, and we've started
            else if(data_[i] == Msgs::MSG_END && started) {
                // Store the last end position
                end = i;

                // Add the data to the message string
                for(char j = start+1; j < end; j++) {
                    message += data_[j];
                }
                // Delete data 
                data_.clear();
            }
        }

        // If a message was fully received then call the callback
        if(message.size()) {
            cmd_->process_message(message);
        }
    }
}

bool WebClient::connect_to_network() {
    bool ret = false;
    
    // Longer timeout
    esp_.setTimeout(8000);
    
    // Try to connect
    if(esp_.send("AT+CWJAP=\"%s\",\"%s\"", SSID, PWD) && 
         esp_.recv("OK"))
        ret = true;
    
    // Reset timeout
    esp_.setTimeout(DEFAULT_TIMEOUT);

    if(ret)
        //pc.printf("Successfully connected to \"%s\".\n", SSID);

    return ret;
}

bool WebClient::connect_TCP() {
    bool ret = false;
    
    // Single TCP connection
    esp_.send("AT+CIPMUX=0") && esp_.recv("OK");

    // Longer timeout for this
    esp_.setTimeout(1000);

    // Check if we're already connected
    if((esp_.send("AT+CIPSTART=\"TCP\",\"%s\",%d", REMOTE_IP, REMOTE_PORT)
         && esp_.recv("ALREADY CONNECTED"))) {
        ret = true;
    }
    else {
        // If the connection was closed by the server we have to close it
        // Before we can open a new one, causing the above command to fail
        esp_.send("AT+CIPCLOSE") && esp_.recv("OK");

        // Try to connect again
        if((esp_.send("AT+CIPSTART=\"TCP\",\"%s\",%d", REMOTE_IP, REMOTE_PORT)
             && esp_.recv("OK"))) {
            ret = true;
        }
    }

    // Reset timeout
    esp_.setTimeout(DEFAULT_TIMEOUT);

    if(ret) {}
        //pc.printf("Successfully connected to \"%s\".\n", REMOTE_IP);
    else
        pc.printf("Failed to connect to \"%s\".\n", REMOTE_IP);

    return ret;
}

bool WebClient::check_connection_status() {
    bool ret = false;

    esp_.send("AT+CIPSTATUS");
    int status;
    esp_.recv("STATUS:%d", &status);

    switch(status) {
    // TCP or UDP connection
    case 3:
        ret = true;
        break;
    // Connected to network and IP obtained - not connected to TCP
    case 2:
    // TCP or UDP is disconnected
    case 4:
        connect_to_network();
        if(!connect_TCP())
            set_error("Lost connection with '" + (std::string)REMOTE_IP + "'.");
        else
            ret = true;
        break;
    // Not connected to a network
    case 5:
        if(!connect_to_network())
            set_error("Lost connection with '" + (std::string)SSID + "'.");
        else if(!connect_TCP())
            set_error("Failed to connect to '" + (std::string)REMOTE_IP + "'.");
        else
            ret = true;
        break;
    }
    
    return ret;
}

void WebClient::set_error(std::string err) {
    // Don't overwrite the previous error, as errors often 
    // end up causing other errors
    if(error_ == "") {
        error_ = err;
    }
}
