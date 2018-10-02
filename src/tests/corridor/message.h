#include <string>

class Message {
  private:
    double time_stamp;
    int senderID;
    std::string messageText;

  public:
    Message(int senderID, double time, std::string messageText);
    Message();
    std::string getText() { return messageText; }
    int getSenderID() { return senderID; }
    double getTimeStamp() { return time_stamp; }

    ~Message();
};
