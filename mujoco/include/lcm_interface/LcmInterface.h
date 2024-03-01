
#pragma once
#include <iostream>
#include <lcm/lcm-cpp.hpp>

#include "lcm_msg/low_state_t.hpp"
#include "lcm_msg/low_cmd_t.hpp"
#include "lcm_msg/full_state_t.hpp"
#include <pthread.h>

#include <mujoco/mujoco.h>
#include <chrono>


class MujocoLcm{

private:
    lcm::LCM lcm_;

    low_cmd_t recvCmd_;
    low_state_t sendState_;
    full_state_t sendFullState_;

    uint64_t last_timestamp = 0;
    bool new_msg =false;

    pthread_mutex_t recvMutex_;
    pthread_mutex_t sendMutex_;

    pthread_t lcmThread_;
public:
    MujocoLcm(/* args */);
    ~MujocoLcm();

    void HandleLowCmd(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const low_cmd_t* msg);
    void GetRecv(low_cmd_t& cmd);
    void SetSend(const mjData * d);
    void Send();

    static void* lcmThreadFunc(void* handler) {
        MujocoLcm* handlerObject = static_cast<MujocoLcm*>(handler);
        while (true) {
            handlerObject->lcm_.handle();
        }
        return NULL;
    }

    void startLCMThread() {
        pthread_create(&lcmThread_, NULL, &MujocoLcm::lcmThreadFunc, this);
    }

    void joinLCMThread() {
        pthread_join(lcmThread_, NULL);
    }
};
