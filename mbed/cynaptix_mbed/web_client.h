#ifndef __WEB_CLIENT_H__
#define __WEB_CLIENT_H__

#include "mbed.h"
#include "ATParser.h"
#include <string>

#include "messages.h"

#define REMOTE_IP (std::string)"10.42.0.1"
#define REMOTE_PORT 13/*53821*/
#define DEFAULT_TIMEOUT 50

#define SSID (std::string)"nickick-laptop"
#define PWD (std::string)"auSEtdYV"

class WebClient {
public:
    // ctor takes the tx, rx and reset PinNames of the ESP, as well as a
    // callback for when a full message has been received
    WebClient(PinName tx, PinName rx, PinName reset, 
              Callback<void(std::string)> message_callback);

    // dtor
    ~WebClient();

    // Update function called regularly to ensure messages are not lost
    void update();

    // Get an error message if any
    std::string get_error();
private:
    // Connect to the network, and return success status
    bool connect_to_network();

    // Make a single TCP connection, return success or fail
    bool connect_TCP();

    // Check whether we are connected or not
    bool check_connection_status();

    // Set the error message
    void set_error(std::string err);

    // Buffered serial to receive lots of messages between updates
    BufferedSerial *esp_bs_;

    // AT command parser for the esp
    ATParser *esp_;

    // Reset pin of the esp
    DigitalOut reset_;

    // String error message
    std::string error_;

    // Data buffer
    char buf_[64];
    
    // All received data
    std::string data_;

    // Message received callback
    Callback<void(std::string)> message_callback_;
};

#endif // __WEB_CLIENT_H__