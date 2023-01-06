//
// Created by user on 05.01.23.
//

#include "SerialCommunicationHandler.h"
#include <exception>
#include <string>

#include <iostream>
#include <iomanip>
#include <boost/asio/streambuf.hpp>

namespace RobotControl {
    namespace Al5dLowLevelDriver
    {
        std::ostream& hex_dump2(std::ostream& os, const void* buffer,
                                std::size_t bufsize, bool showPrintableChars = true)
        {
            if (buffer == nullptr) {
                return os;
            }
            auto oldFormat = os.flags();
            auto oldFillChar = os.fill();
            constexpr std::size_t maxline{ 8 };
            // create a place to store text version of string
            char renderString[maxline + 1];
            char* rsptr{ renderString };
            // convenience cast
            const unsigned char* buf{ reinterpret_cast<const unsigned char*>(buffer) };

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

        SerialCommunicationHandler::SerialCommunicationHandler(const std::string& serialPortPath)
                : port(ioservice, serialPortPath)
        {
            port.set_option(boost::asio::serial_port_base::baud_rate(9600));
            port.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
            port.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
            port.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
            port.set_option(boost::asio::serial_port::character_size(boost::asio::serial_port::character_size(8)));

            if (!port.is_open())
            {
                throw std::runtime_error("UNABLE TO ESTABLISH SERIAL CONNECTION WITH SERIAL DEVICE SSC32U.");
            }
        }

        SerialCommunicationHandler::~SerialCommunicationHandler()
        {
            if (port.is_open()) {
                port.close();
            }
        }
        void SerialCommunicationHandler::ClearDisplay() {
            std::vector<int> arr = { 0x0C };

            if (!port.is_open())
            {
                throw std::runtime_error("Порт не открыт.");
            }
            try
            {
                port.write_some(boost::asio::buffer(arr, sizeof(arr)));
                std::cout << "Дисплей очищен" << std::endl;
            }
            catch (const std::exception& exc)
            {
                std::cout << "Serial: error on write: " << exc.what() << std::endl;
            }
        }
        void SerialCommunicationHandler::WriteLine(Line line, std::string message) {
            uint8_t curs[] = { 0x1B, 0x51, static_cast<uint8_t>(line), 0x83, 0x0D };
            std::array<uint8_t, 3> f_array;
            std::vector<uint8_t> arr = { 0x1B, 0x51, static_cast<uint8_t>(line) };
            std::vector<uint8_t> converted = {};

            std::cout << "message: " << message << std::endl;
            hex_dump2(std::cout, message.data(), message.length() * sizeof(message.front()));
            std::cout << std::endl;
            for (uint8_t item : message) {
                // TODO: Символы ьЬ перепутаны с ъЪ - нужно перезалить их в EEPROM контроллера
                if (item == static_cast<uint8_t>(0xA8))
                {
                    //Ё записываем как Е
                    converted.push_back(0x85);
                }
                else
                if (item == static_cast<uint8_t>(0xB8))
                {
                    //ё записываем как е
                    converted.push_back(0xA5);
                }
                else
                if (
                        (item >= static_cast<uint8_t>(0xF0)) &&
                        (item <= static_cast<uint8_t>(0xFF))
                        )
                {
                    //для диапазона р-я
                    converted.push_back(item - 0x10);
                }
                else {
                    converted.push_back(item - 0x40);
                }

            }
            std::cout << "converted: " << message << std::endl;
            arr.insert(
                    arr.end(),
                    std::make_move_iterator(converted.begin()),
                    std::make_move_iterator(converted.end())
            );
            arr.push_back(0x0D);
            hex_dump2(std::cout, converted.data(), converted.size());
            std::cout << std::endl;
            //Logging
            std::string s(std::begin(curs), std::end(curs));
            //hex_dump2(std::cout, s.data(), s.length() * sizeof(s.front()));
            //hex_dump2(std::cout, arr.data(), sizeof(arr));

            if (!port.is_open())
            {
                throw std::runtime_error("Порт не открыт.");
            }

            try
            {

                //std::cout << "Size of curs " << sizeof(curs) << std::endl;
                //std::cout << "sizeof of arr " << sizeof(arr) << " size " << arr.size() << std::endl;
                std::cout << "sizeof(arr)=" << sizeof(arr) << ", arr.size()=" << arr.size() << std::endl;
                hex_dump2(std::cout, arr.data(), arr.size());
                //port.write_some(boost::asio::buffer(curs, sizeof(curs)));
                port.write_some(boost::asio::buffer(arr.data(), arr.size()));
                std::cout << "Выполнен вывод строки" << std::endl;
            }
            catch (const std::exception& exc)
            {
                std::cout << "Ошибка записи в порт: " << exc.what() << std::endl;
            }
        }
        void SerialCommunicationHandler::MoveCursorEnd() {
            uint8_t curs[] = { 0x1B, 0x5B, 0x52 };
            if (!port.is_open())
            {
                throw std::runtime_error("Порт не открыт.");
            }
            try
            {
                port.write_some(boost::asio::buffer(curs, sizeof(curs)));
            }
            catch (const std::exception& exc)
            {
                std::cout << "Ошибка записи в порт: " << exc.what() << std::endl;
            }
        }
        void SerialCommunicationHandler::MoveCursorStart() {
            uint8_t curs[] = { 0x1B, 0x5B, 0x4C };
            if (!port.is_open())
            {
                throw std::runtime_error("Порт не открыт.");
            }
            try
            {
                port.write_some(boost::asio::buffer(curs, sizeof(curs)));
            }
            catch (const std::exception& exc)
            {
                std::cout << "Ошибка записи в порт: " << exc.what() << std::endl;
            }
        }
        void SerialCommunicationHandler::setBlinkMode(Suit mode) {
            uint8_t curbkk;
            uint8_t intMode = static_cast<uint8_t>(mode);
            curbkk = ("0x0", intMode);
            uint8_t curs[] = { 0x1B, 0x5F, curbkk };
            if (!port.is_open())
            {
                throw std::runtime_error("Порт не открыт.");
            }
            try
            {
                port.write_some(boost::asio::buffer(curs, sizeof(curs)));
            }
            catch (const std::exception& exc)
            {
                std::cout << "Ошибка записи в порт: " << exc.what() << std::endl;
            }
        }

        std::string SerialCommunicationHandler::timedRead(long timeout)
        {
            if (!port.is_open())
            {
                throw std::runtime_error("Порт не открыт.");
            }

            //SerialBlockingReader reader(port, timeout);

            //char c;
            std::string rsp;

            /* while (reader.readChar(c) && c != '\n' && rsp != "+" && rsp != ".")
             {
                 rsp += c;
             }*/

            if (rsp == "." || rsp == "+")
            {
                //ROS_DEBUG("READ SSC32U SERIAL FEEDBACK: %s", rsp.c_str());
                return rsp;
            }

            //if (c != '\n')
            //{
            //    // Timeout
            //    throw std::runtime_error("UNABLE TO READ DATA FROM SSC32U WITHIN DEADLINE TIMER DURATION.");
            //}

            //ROS_DEBUG("READ SSC32U SERIAL FEEDBACK: %s", rsp.c_str());
            return rsp;
        }

    }// namespace RobotControl
}