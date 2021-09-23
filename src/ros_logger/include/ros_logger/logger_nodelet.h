#ifndef LOGGER_NODELET_H_
#define LOGGER_NODELET_H_

#include <nodelet/nodelet.h>
#include "ros_logger/logger.h"

namespace ros_logger {

    class LoggerNodelet : public nodelet::Nodelet {
        public:
          virtual void onInit();

        private:
          ros_logger::Server* server;
        };

    }

#endif // LOGGER_NODELET_H_