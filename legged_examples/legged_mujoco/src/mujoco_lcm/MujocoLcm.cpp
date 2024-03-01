#include "mujoco_lcm/MujocoLcm.h"

MujocoLcm::MujocoLcm(/* args */)
{
    pthread_mutex_init(&recvMutex_, NULL);
    pthread_mutex_init(&sendMutex_, NULL);
    lcm_.subscribe("LOWSTATE", &MujocoLcm::HandleLowState, this);
}

MujocoLcm::~MujocoLcm()
{
    pthread_mutex_destroy(&recvMutex_);
    pthread_mutex_destroy(&sendMutex_);
}

void MujocoLcm::HandleLowState(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const low_state_t* msg)
{
    pthread_mutex_lock(&recvMutex_);
    if (msg->timestamp > last_timestamp) {
        recvState_ = *msg;
        last_timestamp = msg->timestamp;
    }
    pthread_mutex_unlock(&recvMutex_);
}

void MujocoLcm::GetRecv(low_state_t& state)
{
    pthread_mutex_lock(&recvMutex_);
    state = recvState_;
    pthread_mutex_unlock(&recvMutex_);
}

void MujocoLcm::SetSend(low_cmd_t& cmd)
{
    pthread_mutex_lock(&sendMutex_);
    sendCommand_ = cmd;
    pthread_mutex_unlock(&sendMutex_);
}

void MujocoLcm::Send()
{
    pthread_mutex_lock(&sendMutex_);
    lcm_.publish("LOWCMD", &sendCommand_);
    pthread_mutex_unlock(&sendMutex_);
}
