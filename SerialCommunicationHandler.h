#pragma once
#ifndef SERIAL_COMMUNICATION_HANDLER_HPP_
#define SERIAL_COMMUNICATION_HANDLER_HPP_

#include <string>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/io_service.hpp>

namespace RobotControl {
    namespace Al5dLowLevelDriver
    {
        enum class Suit { Off = 0, Blink = 1, Filled = 3 };
        enum class Line { First = 0x41, Second = 0x42};
        class SerialCommunicationHandler
        {
        public:
            /**
             * @brief Construct a new Serial Communication Handler object
             * @author Ties Klappe
             *
             * @param serialPortPath the path to the serial device (SSC32U)
             */
            explicit SerialCommunicationHandler(const std::string& serialPortPath);

            /**
             * @brief Destroy the Serial Communication Handler object
             * @author Ties Klappe
             */
            ~SerialCommunicationHandler();

            void ClearDisplay();


            void WriteLine(Line line, std::string message);

            /**
             * @brief write a string message to the attached SSC32U serial device (or the simulation version)
             * @author Ties Klappe
             *
             * @param message the message to be written
             * @return true if the message was successfully written
             * @return false if the message was not successfully written
             */

            bool write(const std::string& message);

            /**
            * @brief Перемещает курсор в крайнюю правую позицию текущей строки
            * @author Sergey Smirnov
            *
            * @param отсутствют
            * @return отсутствют
            */
            void MoveCursorEnd();
            /**
            * @brief Перемещает курсор в крайнюю левую позицию текущей строки.
            * @author Sergey Smirnov
            *
            * @param отсутствют
            * @return отсутствют
            */
            void MoveCursorStart();

            /**
            * @brief Устанавливает режим моргания курсора
            * @author Sergey Smirnov
            *
            * @param mode режим курсора
            * @return void
            */
            void setBlinkMode(Suit mode);
            /**
             * @brief function that reads a character from a serial device and throws an exception if the
             * function call timedout before data was read
             *
             * @param timeout time in milliseconds to wait before timeout (and throwing the exception)
             * @return std::string returns the data that was read
             * @exception if no character was read before the timeout
             */
            std::string timedRead(long timeout);

        private:
            /** IOservice for the boost::asio serial communication */
            boost::asio::io_service ioservice;
            /** Port: the port the SSC32U is connected to */
            boost::asio::serial_port port;
        };

    } // namespace RobotControl
}
#endif //SERIAL_COMMUNICATION_HANDLER_HPP_