

#include <iomanip>
#include <iostream>
#include "SerialCommunicationHandler.h"
#include <restbed>

using namespace ::boost::asio;  // save tons of typing
using namespace std;
using namespace restbed;
RobotControl::Al5dLowLevelDriver::SerialCommunicationHandler serialPort("/dev/ttyACM0");

void post_method_handler(const shared_ptr<Session> session) {
    const auto request = session->get_request();
    int content_length = request->get_header("Content-Length", 0);
    session->fetch(content_length, [](const shared_ptr<Session> session, const Bytes &body) {
        std::stringstream stringstream1;
        for (unsigned char it: body) {
            stringstream1 << it;
        }
        serialPort.WriteLine(RobotControl::Al5dLowLevelDriver::Line::Second, stringstream1.str());
        session->close(OK, "Hello, World!", {{"Content-Length", "13"}});
    });
}

int main() {
    auto resource = make_shared<Resource>();
    resource->set_path("/resource");
    resource->set_method_handler("POST", post_method_handler);

    auto settings = make_shared<Settings>();
    settings->set_port(1984);
    settings->set_default_header("Connection", "close");

    Service service;
    service.publish(resource);
    try {
        serialPort.ClearDisplay();
    }
    catch (boost::system::system_error &e) {
        std::cerr << "Ошибка открытия порта: " << e.what() << std::endl;
        return -1;
    }
    service.start(settings);
}