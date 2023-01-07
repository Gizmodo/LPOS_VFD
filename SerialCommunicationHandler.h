#pragma once
#ifndef SERIAL_COMMUNICATION_HANDLER_HPP_
#define SERIAL_COMMUNICATION_HANDLER_HPP_

#include <string>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/io_service.hpp>

namespace RobotControl {
    namespace Al5dLowLevelDriver {
        enum class CursorMode {
            Off = 0, Blink = 1, Filled = 3
        };
        enum class Line {
            First = 0x41, Second = 0x42, FirstScroll = 0x44
        };
        enum class Direction {
            Up = 0x41,
            Down = 0x42,
            Right = 0x43,
            Left = 0x44,
            TopLeft = 0x48,
            CurrentLineStart = 0x4C,
            CurrentLineEnd = 0x52,
            BottomEnd = 0x4B
        };

        class SerialCommunicationHandler {
        public:
            /**
             * @brief Construct a new Serial Communication Handler object
             * @author Ties Klappe
             *
             * @param serialPortPath the path to the serial device (SSC32U)
             */
            explicit SerialCommunicationHandler(const std::string &serialPortPath);

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

            /**
             * @brief function that reads a character from a serial device and throws an exception if the
             * function call timedout before data was read
             *
             * @param timeout time in milliseconds to wait before timeout (and throwing the exception)
             * @return std::string returns the data that was read
             * @exception if no character was read before the timeout
             */
            void ScrollOverwrite();

            void ScrollVertical();

            void ScrollHorizontal();

            void MoveTo(Direction direction);

            void MoveToPosition(uint8_t x, uint8_t y);

            void DisplayInit();

            void ClearLine();

            void ChangeCursorMode(CursorMode cursorMode);

        private:
            /** IOservice for the boost::asio serial communication */
            boost::asio::io_service ioservice;
            /** Port: the port the SSC32U is connected to */
            boost::asio::serial_port port;


        };

    } // namespace RobotControl
}
#endif //SERIAL_COMMUNICATION_HANDLER_HPP_