#include "message.h"

Message::Message(int senderID, std::string messageText) {
    this->senderID = senderID;
    this->messageText = messageText;
}

Message::Message() {
    senderID = 0;
    messageText = "";
}

Message::~Message() {}
