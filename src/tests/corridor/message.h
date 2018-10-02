#include <string>

class Message {
  private:
    int senderID;
    std::string messageText;

  public:
    Message(int senderID, std::string messageText);
    Message();
    std::string getText() { return messageText; }
    int getSenderID() { return senderID; }

    ~Message();
};
