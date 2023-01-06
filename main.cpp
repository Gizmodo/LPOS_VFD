

#include <iomanip>
#include <iostream>
#include "SerialCommunicationHandler.h"
#include <restbed>

using namespace ::boost::asio;  // save tons of typing
std::ostream &render_printable_chars(std::ostream &os, const char *buffer, size_t bufsize) {
    os << " | ";
    for (size_t i = 0; i < bufsize; ++i) {
        if (std::isprint(buffer[i])) {
            os << buffer[i];
        } else {
            os << ".";
        }
    }
    return os;
}

std::ostream &hex_dump(std::ostream &os, const uint8_t *buffer, size_t bufsize, bool showPrintableChars = true) {
    auto oldFormat = os.flags();
    auto oldFillChar = os.fill();

    os << std::hex;
    os.fill('0');
    bool printBlank = false;
    size_t i = 0;
    for (; i < bufsize; ++i) {
        if (i % 8 == 0) {
            if (i != 0 && showPrintableChars) {
                render_printable_chars(os, reinterpret_cast<const char *>(&buffer[i] - 8), 8);
            }
            os << std::endl;
            printBlank = false;
            os << (void *) &buffer[i] << ": ";
        }
        if (printBlank) {
            os << ' ';
        }
        os << std::setw(2) << std::right << unsigned(buffer[i]);
        if (!printBlank) {
            printBlank = true;
        }
    }
    if (i % 8 != 0 && showPrintableChars) {
        for (size_t j = 0; j < 8 - (i % 8); ++j) {
            os << "   ";
        }
        render_printable_chars(os, reinterpret_cast<const char *>(&buffer[i] - (i % 8)), (i % 8));
    }

    os << std::endl;

    os.fill(oldFillChar);
    os.flags(oldFormat);

    return os;
}

std::ostream &hex_dump(std::ostream &os, const std::string &buffer, bool showPrintableChars = true) {
    return hex_dump(os, reinterpret_cast<const uint8_t *>(buffer.data()), buffer.length(), showPrintableChars);
}

std::ostream &hex_dump1(std::ostream &os, const void *buffer,
                        std::size_t bufsize, bool showPrintableChars = true) {
    if (buffer == nullptr) {
        return os;
    }
    auto oldFormat = os.flags();
    auto oldFillChar = os.fill();
    constexpr std::size_t maxline{8};
    // create a place to store text version of string
    char renderString[maxline + 1];
    char *rsptr{renderString};
    // convenience cast
    const unsigned char *buf{reinterpret_cast<const unsigned char *>(buffer)};

    for (std::size_t linecount = maxline; bufsize; --bufsize, ++buf) {
        os << std::setw(2) << std::setfill('0') << std::hex
           << static_cast<unsigned>(*buf) << ' ';
        *rsptr++ = std::isprint(*buf) ? *buf : '.';
        if (--linecount == 0) {
            *rsptr++ = '\0';  // terminate string
            if (showPrintableChars) {
                os << " | " << renderString;
            }
            os << '\n';
            rsptr = renderString;
            linecount = std::min(maxline, bufsize);
        }
    }
    // emit newline if we haven't already
    if (rsptr != renderString) {
        if (showPrintableChars) {
            for (*rsptr++ = '\0'; rsptr != &renderString[maxline + 1]; ++rsptr) {
                os << "   ";
            }
            os << " | " << renderString;
        }
        os << '\n';
    }

    os.fill(oldFillChar);
    os.flags(oldFormat);
    return os;
}

std::array<uint8_t, 3> Brightness(uint8_t brillo) {

    uint8_t brilloa;
    brilloa = ("0x0", brillo);
    std::array<uint8_t, 3> f_array = {0x1B, 0x2A, brilloa};
    uint8_t brillomsg[] = {0x1B, 0x2A, brilloa};

    return f_array;
}

using namespace std;
using namespace restbed;
RobotControl::Al5dLowLevelDriver::SerialCommunicationHandler ss("/dev/ttyACM0");

void post_method_handler(const shared_ptr<Session> session) {
    const auto request = session->get_request();

    int content_length = request->get_header("Content-Length", 0);

    session->fetch(content_length, [](const shared_ptr<Session> session, const Bytes &body) {
        fprintf(stdout, "%.*s\n", (int) body.size(), body.data());
        auto buf = body.data();
        std::string s(reinterpret_cast<const char *>(buf));
        ss.WriteLine(RobotControl::Al5dLowLevelDriver::Line::Second, s);
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
        ss.ClearDisplay();
        //ss.setBlinkMode(RobotControl::Al5dLowLevelDriver::Suit::Blink);
        //ss.MoveCursorEnd();
        //  ss.WriteLine(RobotControl::Al5dLowLevelDriver::Line::First,"прстуфхцшщьыъэюя");
        ss.WriteLine(RobotControl::Al5dLowLevelDriver::Line::First, "222");
    }
    catch (boost::system::system_error &e) {
        std::cerr << "Ошибка открытия порта: " << e.what() << std::endl;
        return -1;
    }
    service.start(settings);
}