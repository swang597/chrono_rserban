#include "message.h"

Message::Message(int senderID, double time, std::string messageText) {
    time_stamp = time;
    this->senderID = senderID;
    this->messageText = messageText;
}

Message::Message() : senderID(-1), time_stamp(0), messageText("") {}

Message::~Message() {}
