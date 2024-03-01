#include "lcm_interface/LcmInterface.h"


MujocoLcm::MujocoLcm(/* args */)
{
    if (pthread_mutex_init(&recvMutex_, NULL) != 0) {
        std::cerr << "Mutex initialization failed" << std::endl;
        // 处理初始化失败的情况
    }
    if (pthread_mutex_init(&sendMutex_, NULL) != 0) {
        std::cerr << "Mutex initialization failed" << std::endl;
        // 处理初始化失败的情况
    }
    lcm_.subscribe("LOWCMD", &MujocoLcm::HandleLowCmd, this);
}

MujocoLcm::~MujocoLcm()
{
    pthread_mutex_destroy(&recvMutex_); 
    pthread_mutex_destroy(&sendMutex_); 
}

void MujocoLcm::HandleLowCmd(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const low_cmd_t* msg) {
    // 处理接收到的消息
    pthread_mutex_lock(&recvMutex_);
    // printf("Received message on channel \"%s\":\n", chan.c_str());
    if (msg->timestamp > last_timestamp) {
        recvCmd_ = *msg;
        last_timestamp = msg->timestamp;
        new_msg = true;
    }
    pthread_mutex_unlock(&recvMutex_);
}

void MujocoLcm::GetRecv(low_cmd_t& cmd) {
    pthread_mutex_lock(&recvMutex_);
    if(new_msg)
    {
        cmd = recvCmd_;
        new_msg = false;
    }
    pthread_mutex_unlock(&recvMutex_);
}

void MujocoLcm::SetSend(const mjData * d) {
    pthread_mutex_lock(&sendMutex_);

    auto current_time_point = std::chrono::system_clock::now();
    sendState_.timestamp = static_cast<int64_t> (current_time_point.time_since_epoch().count()) ;
    sendFullState_.timestamp =  static_cast<int64_t> ( current_time_point.time_since_epoch().count() );

    sendState_.quaternion[0] = d->qpos[3];
    sendState_.quaternion[1] = d->qpos[4];
    sendState_.quaternion[2] = d->qpos[5];
    sendState_.quaternion[3] = d->qpos[6];

    sendState_.accelerometer[0] = d->sensordata[0];
    sendState_.accelerometer[1] = d->sensordata[1];
    sendState_.accelerometer[2] = d->sensordata[2];

    sendState_.gyroscope[0] = d->sensordata[3];
    sendState_.gyroscope[1] = d->sensordata[4];
    sendState_.gyroscope[2] = d->sensordata[5];


    for (size_t i = 0; i < 10; i++)
    {
        sendState_.joint_pos[i] = d->qpos[7+i];
        sendState_.joint_vel[i] = d->qvel[6+i];
        sendState_.joint_torque[i] = d->qfrc_applied[i];
    }

    // for (size_t i = 0; i < 4; i++)
    // {
    //     sendState_.foot_force[i] = d->sensordata[i+6];
    // }

    sendFullState_.quaternion[0] = d->qpos[3];
    sendFullState_.quaternion[1] = d->qpos[4];
    sendFullState_.quaternion[2] = d->qpos[5];
    sendFullState_.quaternion[3] = d->qpos[6];

    sendFullState_.gyroscope[0] = d->qvel[3];
    sendFullState_.gyroscope[1] = d->qvel[4];
    sendFullState_.gyroscope[2] = d->qvel[5];

    sendFullState_.position[0] = d->qpos[0];
    sendFullState_.position[1] = d->qpos[1];
    sendFullState_.position[2] = d->qpos[2];

    sendFullState_.velocity[0] = d->qvel[0];
    sendFullState_.velocity[1] = d->qvel[1];
    sendFullState_.velocity[2] = d->qvel[2];

    for (size_t i = 0; i < 10; i++)
    {
        sendFullState_.joint_pos[i] = d->qpos[7+i];
        sendFullState_.joint_vel[i] = d->qvel[6+i];
        sendFullState_.joint_torque[i] = d->qfrc_applied[i];
    }


    pthread_mutex_unlock(&sendMutex_);
}

void MujocoLcm::Send() {
    pthread_mutex_lock(&sendMutex_);
    lcm_.publish("LOWSTATE", &sendState_);
    lcm_.publish("LOWSTATEFULL", &sendFullState_);
    pthread_mutex_unlock(&sendMutex_);
}
