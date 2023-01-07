//
// Created by user on 05.01.23.
//

#include "SerialCommunicationHandler.h"
#include <exception>
#include <string>

#include <iostream>
#include <iomanip>
#include <boost/asio/streambuf.hpp>

namespace RobotControl::Al5dLowLevelDriver {
    std::ostream &hex_dump2(std::ostream &os, const void *buffer,
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

    SerialCommunicationHandler::SerialCommunicationHandler(const std::string &serialPortPath)
            : port(ioservice, serialPortPath) {
        port.set_option(boost::asio::serial_port_base::baud_rate(9600));
        port.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
        port.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
        port.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
        port.set_option(boost::asio::serial_port::character_size(boost::asio::serial_port::character_size(8)));

        if (!port.is_open()) {
            throw std::runtime_error("НЕВОЗМОЖНО УСТАНОВИТЬ ПОСЛЕДОВАТЕЛЬНОЕ СОЕДИНЕНИЕ С ПОСЛЕДОВАТЕЛЬНЫМ УСТРОЙСТВОМ.");
        }
    }

    SerialCommunicationHandler::~SerialCommunicationHandler() {
        if (port.is_open()) {
            port.close();
        }
    }

    void SerialCommunicationHandler::ScrollHorizontal() {
        uint8_t payload[] = {0x1B, 0x13};
        if (!port.is_open()) {
            throw std::runtime_error("Порт не открыт.");
        }
        try {
            port.write_some(boost::asio::buffer(payload, sizeof(payload)));
            std::cout << "Режим горизонтальной прокрутки" << std::endl;
        }
        catch (const std::exception &exc) {
            std::cout << "Ошибка записи в порт: " << exc.what() << std::endl;
        }
    }

    void SerialCommunicationHandler::ScrollVertical() {
        uint8_t payload[] = {0x1B, 0x12};
        if (!port.is_open()) {
            throw std::runtime_error("Порт не открыт.");
        }
        try {
            port.write_some(boost::asio::buffer(payload, sizeof(payload)));
            std::cout << "Режим вертикальной прокрутки" << std::endl;
        }
        catch (const std::exception &exc) {
            std::cout << "Ошибка записи в порт: " << exc.what() << std::endl;
        }
    }

    void SerialCommunicationHandler::ScrollOverwrite() {
        uint8_t payload[] = {0x1B, 0x11};
        if (!port.is_open()) {
            throw std::runtime_error("Порт не открыт.");
        }
        try {
            port.write_some(boost::asio::buffer(payload, sizeof(payload)));
            std::cout << "Режим перезаписи" << std::endl;
        }
        catch (const std::exception &exc) {
            std::cout << "Ошибка записи в порт: " << exc.what() << std::endl;
        }
    }

    void SerialCommunicationHandler::MoveTo(Direction direction) {
        uint8_t payload[] = {0x1B, 0x5B, static_cast<uint8_t>(direction)};
        if (!port.is_open()) {
            throw std::runtime_error("Порт не открыт.");
        }
        try {
            port.write_some(boost::asio::buffer(payload, sizeof(payload)));
            std::cout << "Перемещение курсора" << std::endl;
        }
        catch (const std::exception &exc) {
            std::cout << "Ошибка записи в порт: " << exc.what() << std::endl;
        }
    }

    void SerialCommunicationHandler::MoveToPosition(uint8_t x, uint8_t y) {
        if (((y >= 0x01) && (y <= 0x02)) && ((x >= 0x01) && (x <= 0x14))) {
            uint8_t payload[] = {0x1B, 0x6C, static_cast<uint8_t>(x), static_cast<uint8_t>(y)};
            if (!port.is_open()) {
                throw std::runtime_error("Порт не открыт.");
            }
            try {
                port.write_some(boost::asio::buffer(payload, sizeof(payload)));
                std::cout << "Перемещение курсора" << std::endl;
            }
            catch (const std::exception &exc) {
                std::cout << "Ошибка записи в порт: " << exc.what() << std::endl;
            }
        } else {
            std::cout << "Неверная координата" << std::endl;
        }
    }

    void SerialCommunicationHandler::DisplayInit() {
        uint8_t payload[] = {0x1B, 0x40};
        if (!port.is_open()) {
            throw std::runtime_error("Порт не открыт.");
        }
        try {
            port.write_some(boost::asio::buffer(payload, sizeof(payload)));
            std::cout << "Инициализация дисплея" << std::endl;
        }
        catch (const std::exception &exc) {
            std::cout << "Ошибка записи в порт: " << exc.what() << std::endl;
        }
    }

    void SerialCommunicationHandler::ClearDisplay() {
        uint8_t payload[] = {0x0C};
        if (!port.is_open()) {
            throw std::runtime_error("Порт не открыт.");
        }
        try {
            port.write_some(boost::asio::buffer(payload, sizeof(payload)));
            std::cout << "Очистка экрана" << std::endl;
        }
        catch (const std::exception &exc) {
            std::cout << "Ошибка записи в порт: " << exc.what() << std::endl;
        }
    }

    void SerialCommunicationHandler::ClearLine() {
        uint8_t payload[] = {0x18};
        if (!port.is_open()) {
            throw std::runtime_error("Порт не открыт.");
        }
        try {
            port.write_some(boost::asio::buffer(payload, sizeof(payload)));
            std::cout << "Очистка строки" << std::endl;
        }
        catch (const std::exception &exc) {
            std::cout << "Ошибка записи в порт: " << exc.what() << std::endl;
        }
    }

    void SerialCommunicationHandler::WriteLine(Line line, std::string message) {
        std::vector<uint8_t> arr = {0x1B, 0x51, static_cast<uint8_t>(line)};
        std::vector<uint8_t> converted = {};
        std::cout << "Input message:" << message << std::endl;
        hex_dump2(std::cout, message.data(), message.length() * sizeof(message.front()));
        for (uint8_t item: message) {
            // TODO: Символы ьЬ перепутаны с ъЪ - нужно перезалить их в EEPROM контроллера
            if (item == static_cast<uint8_t>(0xD0) || item == static_cast<uint8_t>(0xD1)) {
                continue;
            }
            if ((item >= static_cast<uint8_t>(0x80)) && (item <= static_cast<uint8_t>(0xFF))) {

                if (item == static_cast<uint8_t>(0xA8)) {
                    //Ё записываем как Е
                    converted.push_back(0x85);
                } else if (item == static_cast<uint8_t>(0xB8)) {
                    //ё записываем как е
                    converted.push_back(0xA5);
                } else
                    //для диапазона р-я
                if ((item >= static_cast<uint8_t>(0x80)) && (item <= static_cast<uint8_t>(0x8F))) {
                    converted.push_back(item + 0x60);
                } else {
                    converted.push_back(item - 0x10);
                }
            } else {
                converted.push_back(item);
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

        if (!port.is_open()) {
            throw std::runtime_error("Порт не открыт.");
        }

        try {
            port.write_some(boost::asio::buffer(arr.data(), arr.size()));
            std::cout << "Выполнен вывод строки" << std::endl;
        }
        catch (const std::exception &exc) {
            std::cout << "Ошибка записи в порт: " << exc.what() << std::endl;
        }
    }

    void SerialCommunicationHandler::ChangeCursorMode(CursorMode cursorMode) {
        uint8_t payload[] = {0x1B, 0x5F, static_cast<uint8_t>(cursorMode)};
        if (!port.is_open()) {
            throw std::runtime_error("Порт не открыт.");
        }
        try {
            port.write_some(boost::asio::buffer(payload, sizeof(payload)));
            std::cout << "Настройка курсора " << std::endl;
        }
        catch (const std::exception &exc) {
            std::cout << "Ошибка записи в порт: " << exc.what() << std::endl;
        }
    }
}