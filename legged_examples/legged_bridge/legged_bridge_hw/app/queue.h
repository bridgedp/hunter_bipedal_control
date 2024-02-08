
#ifndef SOEM_QUEUE_H
#define SOEM_QUEUE_H

#include <boost/lockfree/spsc_queue.hpp>
#include <memory>
#include <atomic>
#include <thread>

extern "C" {
#include "transmit.h"
};

using boost::lockfree::capacity;
using boost::lockfree::spsc_queue;
typedef std::shared_ptr<EtherCAT_Msg> EtherCAT_Msg_ptr;

extern spsc_queue<EtherCAT_Msg_ptr, capacity<10>> messages[SLAVE_NUMBER];
extern std::atomic<bool> running;
extern std::thread runThread;

#endif  // SOEM_QUEUE_H
