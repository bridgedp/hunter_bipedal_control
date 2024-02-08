extern "C" {
#include "ethercat.h"  
#include "motor_control.h"
#include "transmit.h"
}
#include <iostream>
#include "queue.h"
#include <sys/time.h>
#include <cinttypes>
#include <cstdio>
#include <cstring>

#define EC_TIMEOUTM

spsc_queue<EtherCAT_Msg_ptr, capacity<10>> messages[SLAVE_NUMBER];
std::atomic<bool> running{ false };
std::thread runThread;
YKSMotorData motorDate_recv[12];
YKSIMUData imuData_recv;
char IOmap[4096];
OSAL_THREAD_HANDLE checkThread;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;
uint64_t num;
bool isConfig[SLAVE_NUMBER]{ false };

#define EC_TIMEOUTMON 500

void EtherCAT_Data_Get();

void EtherCAT_Command_Set();
void EtherCAT_Get_State();
void EtherCAT_Send_Command(YKSMotorData* data);
void EtherCAT_Send_Command_Position(YKSMotorData* data);
void EtherCAT_Send_Command_Speed(float* data);
void Revert_State(YKSMotorData* motor_data);

static void degraded_handler()
{
  printf("[EtherCAT Error] Logging error...\n");
  time_t current_time = time(NULL);
  char* time_str = ctime(&current_time);
  printf("ESTOP. EtherCAT became degraded at %s.\n", time_str);
  printf("[EtherCAT Error] Stopping RT process.\n");
}

static int run_ethercat(const char* ifname)
{
  int i;
  int oloop, iloop, chk;
  needlf = FALSE;
  inOP = FALSE;

  num = 1;

  /* initialise SOEM, bind socket to ifname */
  if (ec_init(ifname))
  {
    printf("[EtherCAT Init] Initialization on device %s succeeded.\n", ifname);
    /* find and auto-config slaves */

    if (ec_config_init(FALSE) > 0)
    {
      printf("[EtherCAT Init] %d slaves found and configured.\n", ec_slavecount);
      if (ec_slavecount < SLAVE_NUMBER)
      {
        printf("[RT EtherCAT] Warning: Expected %d slaves, found %d.\n", SLAVE_NUMBER, ec_slavecount);
      }

      for (int slave_idx = 0; slave_idx < ec_slavecount; slave_idx++)
        ec_slave[slave_idx + 1].CoEdetails &= ~ECT_COEDET_SDOCA;

      ec_config_map(&IOmap);
      ec_configdc();

      printf("[EtherCAT Init] Mapped slaves.\n");
      /* wait for all slaves to reach SAFE_OP state */
      ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * SLAVE_NUMBER);

      for (int slave_idx = 0; slave_idx < ec_slavecount; slave_idx++)
      {
        printf("[SLAVE %d]\n", slave_idx);
        printf("  IN  %d bytes, %d bits\n", ec_slave[slave_idx].Ibytes, ec_slave[slave_idx].Ibits);
        printf("  OUT %d bytes, %d bits\n", ec_slave[slave_idx].Obytes, ec_slave[slave_idx].Obits);
        printf("\n");
      }

      oloop = ec_slave[0].Obytes;
      if ((oloop == 0) && (ec_slave[0].Obits > 0))
        oloop = 1;
      if (oloop > 8)
        oloop = 8;
      iloop = ec_slave[0].Ibytes;
      if ((iloop == 0) && (ec_slave[0].Ibits > 0))
        iloop = 1;
      if (iloop > 8)
        iloop = 8;

      printf("[EtherCAT Init] segments : %d : %d %d %d %d\n", ec_group[0].nsegments, ec_group[0].IOsegment[0],
             ec_group[0].IOsegment[1], ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

      printf("[EtherCAT Init] Requesting operational state for all slaves...\n");
      expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
      printf("[EtherCAT Init] Calculated workcounter %d\n", expectedWKC);
      ec_slave[0].state = EC_STATE_OPERATIONAL;
      /* send one valid process data to make outputs in slaves happy*/
      ec_send_processdata();
      ec_receive_processdata(EC_TIMEOUTRET);
      /* request OP state for all slaves */
      ec_writestate(0);
      chk = 40;
      /* wait for all slaves to reach OP state */
      do
      {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
      } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

      if (ec_slave[0].state == EC_STATE_OPERATIONAL)
      {
        printf("[EtherCAT Init] Operational state reached for all slaves.\n");
        inOP = TRUE;
        return 1;
      }
      else
      {
        printf("[EtherCAT Error] Not all slaves reached operational state.\n");
        ec_readstate();
        for (i = 1; i <= ec_slavecount; i++)
        {
          if (ec_slave[i].state != EC_STATE_OPERATIONAL)
          {
            printf("[EtherCAT Error] Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n", i, ec_slave[i].state,
                   ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
          }
        }
      }
    }
    else
    {
      printf("[EtherCAT Error] No slaves found!\n");
    }
  }
  else
  {
    printf("[EtherCAT Error] No socket connection on %s - are you running run.sh?\n", ifname);
  }
  return 0;
}

static int err_count = 0;
static int err_iteration_count = 0;
/**@brief EtherCAT errors are measured over this period of loop iterations */
#define K_ETHERCAT_ERR_PERIOD 100

/**@brief Maximum number of etherCAT errors before a fault per period of loop iterations */
#define K_ETHERCAT_ERR_MAX 20

static OSAL_THREAD_FUNC ecatcheck(void* ptr)
{
  (void)ptr;
  int slave = 0;
  while (1)
  {
    // count errors
    if (err_iteration_count > K_ETHERCAT_ERR_PERIOD)
    {
      err_iteration_count = 0;
      err_count = 0;
    }

    if (err_count > K_ETHERCAT_ERR_MAX)
    {
      // possibly shut down
      printf("[EtherCAT Error] EtherCAT connection degraded.\n");
      printf("[Simulink-Linux] Shutting down....\n");
      degraded_handler();
      break;
    }
    err_iteration_count++;

    if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
    {
      if (needlf)
      {
        needlf = FALSE;
        printf("\n");
      }
      /* one ore more slaves are not responding */
      ec_group[currentgroup].docheckstate = FALSE;
      ec_readstate();
      for (slave = 1; slave <= ec_slavecount; slave++)
      {
        if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
        {
          ec_group[currentgroup].docheckstate = TRUE;
          if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
          {
            printf("[EtherCAT Error] Slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
            ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
            ec_writestate(slave);
            err_count++;
          }
          else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
          {
            printf("[EtherCAT Error] Slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
            ec_slave[slave].state = EC_STATE_OPERATIONAL;
            ec_writestate(slave);
            err_count++;
          }
          else if (ec_slave[slave].state > 0)
          {
            if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
            {
              ec_slave[slave].islost = FALSE;
              printf("[EtherCAT Status] Slave %d reconfigured\n", slave);
            }
          }
          else if (!ec_slave[slave].islost)
          {
            /* re-check state */
            ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
            if (!ec_slave[slave].state)
            {
              ec_slave[slave].islost = TRUE;
              printf("[EtherCAT Error] Slave %d lost\n", slave);
              err_count++;
            }
          }
        }
        if (ec_slave[slave].islost)
        {
          if (!ec_slave[slave].state)
          {
            if (ec_recover_slave(slave, EC_TIMEOUTMON))
            {
              ec_slave[slave].islost = FALSE;
              printf("[EtherCAT Status] Slave %d recovered\n", slave);
            }
          }
          else
          {
            ec_slave[slave].islost = FALSE;
            printf("[EtherCAT Status] Slave %d found\n", slave);
          }
        }
      }
      if (!ec_group[currentgroup].docheckstate)
        printf("[EtherCAT Status] All slaves resumed OPERATIONAL.\n");
    }
    osal_usleep(50000);
  }
}

int EtherCAT_Init(char* ifname)
{
  int i;
  int rc;
  printf("[EtherCAT] Initializing EtherCAT\n");
  osal_thread_create((void*)&checkThread, 128000, (void*)&ecatcheck, (void*)&ctime);
  for (i = 1; i < 10; i++)
  {
    printf("[EtherCAT] Attempting to start EtherCAT, try %d of 10.\n", i);
    rc = run_ethercat(ifname);
    if (rc)
      break;
    osal_usleep(1000000);
  }
  if (rc)
    printf("[EtherCAT] EtherCAT successfully initialized on attempt %d \n", i);
  else
  {
    printf("[EtherCAT Error] Failed to initialize EtherCAT after 10 tries. \n");
  }
  return ec_slavecount;
}

static int wkc_err_count = 0;
static int wkc_err_iteration_count = 0;

EtherCAT_Msg Rx_Message[SLAVE_NUMBER];
EtherCAT_Msg Tx_Message[SLAVE_NUMBER];

void EtherCAT_Run()
{
  if (wkc_err_iteration_count > K_ETHERCAT_ERR_PERIOD)
  {
    wkc_err_count = 0;
    wkc_err_iteration_count = 0;
  }
  if (wkc_err_count > K_ETHERCAT_ERR_MAX)
  {
    printf("[EtherCAT Error] Error count too high!\n");
    degraded_handler();
  }
  // send
  EtherCAT_Command_Set();
  ec_send_processdata();
  // receive
  wkc = ec_receive_processdata(EC_TIMEOUTRET);
  EtherCAT_Data_Get();
  //  check for dropped packet
  if (wkc < expectedWKC)
  {
    printf("\x1b[31m[EtherCAT Error] Dropped packet (Bad WKC!)\x1b[0m\n");
    wkc_err_count++;
  }
  else
  {
    needlf = TRUE;
  }
  wkc_err_iteration_count++;
}

void EtherCAT_Data_Get()
{
  for (int slave = 0; slave < ec_slavecount; ++slave)
  {
    EtherCAT_Msg* slave_src = (EtherCAT_Msg*)(ec_slave[slave + 1].inputs);
    if (slave_src)
      Rx_Message[slave] = *(EtherCAT_Msg*)(ec_slave[slave + 1].inputs);
    RV_can_data_repack(&Rx_Message[slave], comm_ack, slave);
  }
}
void Revert_State(YKSMotorData* motor_data)
{
  YKSMotorData tmp;

  for (int i = 3; i < 6; i++)
  {
    tmp = motor_data[i];
    motor_data[i] = motor_data[i + 6];  // 0-3
    motor_data[i + 6] = tmp;
  }
}

// *****************************
// *****************************
// *****************************
/*函数功能：获取EtherCAT总线上从设备的状态信息*/
void EtherCAT_Get_State()
{
  wkc = ec_receive_processdata(EC_TIMEOUTRET);

  for (int slave = 0; slave < ec_slavecount; ++slave)
  {
    EtherCAT_Msg* slave_src = (EtherCAT_Msg*)(ec_slave[slave + 1].inputs);
    if (slave_src)
    {
      Rx_Message[slave] = *(EtherCAT_Msg*)(ec_slave[slave + 1].inputs);
    }
    if (slave < 2)
      RV_can_data_repack(&Rx_Message[slave], comm_ack, slave);
    else
      RV_can_imu_data_repack(&Rx_Message[slave]);
    if (slave == 0)
    {
      for (int motor_index = 0; motor_index < 5; motor_index++)
      {
        motorDate_recv[motor_index].pos_ = rv_motor_msg[motor_index].angle_actual_rad;
        motorDate_recv[motor_index].vel_ = rv_motor_msg[motor_index].speed_actual_rad;
        if (motor_index == 2 || motor_index == 3)
        {
          motorDate_recv[motor_index].tau_ = rv_motor_msg[motor_index].current_actual_float * 2.1;
        }
        else
        {
          motorDate_recv[motor_index].tau_ = rv_motor_msg[motor_index].current_actual_float * 1.4;
        }
      }
    }
    else if (slave == 1)
    {
      for (int motor_index = 0; motor_index < 5; motor_index++)
      {
        motorDate_recv[motor_index + 5].pos_ = rv_motor_msg[motor_index].angle_actual_rad;
        motorDate_recv[motor_index + 5].vel_ = rv_motor_msg[motor_index].speed_actual_rad;
        if (motor_index == 2 || motor_index == 3)
        {
          motorDate_recv[motor_index + 5].tau_ = rv_motor_msg[motor_index].current_actual_float * 2.1;
        }
        else
        {
          motorDate_recv[motor_index + 5].tau_ = rv_motor_msg[motor_index].current_actual_float * 1.4;
        }
      }
    }
    else if (slave == 2)
    {
      memcpy(imuData_recv.angle_float, imu_msg.angle_float, 3 * 4);
      memcpy(imuData_recv.gyro_float, imu_msg.gyro_float, 3 * 4);
      memcpy(imuData_recv.accel_float, imu_msg.accel_float, 3 * 4);
      memcpy(imuData_recv.mag_float, imu_msg.mag_float, 3 * 4);
      memcpy(imuData_recv.quat_float, imu_msg.quat_float, 4 * 4);
    }

    for (int motor_id = 0; motor_id < 5; motor_id++)
    {
      rv_motor_msg[motor_id].angle_actual_rad = 0;
    }
  }

  //  check for dropped packet
  if (wkc < expectedWKC)
  {
    printf("\x1b[31m[EtherCAT Error] Dropped packet (Bad WKC!)\x1b[0m\n");
    wkc_err_count++;
  }
  else
  {
    needlf = TRUE;
  }
  wkc_err_iteration_count++;
}

void EtherCAT_Send_Command(YKSMotorData* mot_data)
{
  uint16_t slave;

  if (wkc_err_iteration_count > K_ETHERCAT_ERR_PERIOD)
  {
    wkc_err_count = 0;
    wkc_err_iteration_count = 0;
  }
  if (wkc_err_count > K_ETHERCAT_ERR_MAX)
  {
    printf("[EtherCAT Error] Error count too high!\n");
    degraded_handler();
  }
  for (int index = 0; index < 12; index++)
  {
    if (index == 0)
    {
      slave = 0;
      send_motor_ctrl_cmd(&Tx_Message[slave], 1, 1, mot_data[0].kp_, mot_data[0].kd_, mot_data[0].pos_des_,
                          mot_data[0].vel_des_, mot_data[0].ff_);
    }
    else if (index == 1)
    {
      slave = 0;
      send_motor_ctrl_cmd(&Tx_Message[slave], 2, 2, mot_data[1].kp_, mot_data[1].kd_, mot_data[1].pos_des_,
                          mot_data[1].vel_des_, mot_data[1].ff_);
    }
    else if (index == 2)
    {
      slave = 0;
      send_motor_ctrl_cmd(&Tx_Message[slave], 4, 3, mot_data[2].kp_, mot_data[2].kd_, mot_data[2].pos_des_,
                          mot_data[2].vel_des_, mot_data[2].ff_);
    }
    else if (index == 3)
    {
      slave = 0;
      send_motor_ctrl_cmd(&Tx_Message[slave], 5, 4, mot_data[3].kp_, mot_data[3].kd_, mot_data[3].pos_des_,
                          mot_data[3].vel_des_, mot_data[3].ff_);
    }
    else if (index == 4)
    {
      slave = 0;
      send_motor_ctrl_cmd(&Tx_Message[slave], 6, 5, mot_data[4].kp_, mot_data[4].kd_, mot_data[4].pos_des_,
                          mot_data[4].vel_des_, mot_data[4].ff_);
    }
    else if (index == 5)
    {
    }
    else if (index == 6)
    {
      slave = 1;
      send_motor_ctrl_cmd(&Tx_Message[slave], 1, 1, mot_data[5].kp_, mot_data[5].kd_, mot_data[5].pos_des_,
                          mot_data[5].vel_des_, mot_data[5].ff_);
    }
    else if (index == 7)
    {
      slave = 1;
      send_motor_ctrl_cmd(&Tx_Message[slave], 2, 2, mot_data[6].kp_, mot_data[6].kd_, mot_data[6].pos_des_,
                          mot_data[6].vel_des_, mot_data[6].ff_);
    }
    else if (index == 8)
    {
      slave = 1;
      send_motor_ctrl_cmd(&Tx_Message[slave], 4, 3, mot_data[7].kp_, mot_data[7].kd_, mot_data[7].pos_des_,
                          mot_data[7].vel_des_, mot_data[7].ff_);
    }
    else if (index == 9)
    {
      slave = 1;
      send_motor_ctrl_cmd(&Tx_Message[slave], 5, 4, mot_data[8].kp_, mot_data[8].kd_, mot_data[8].pos_des_,
                          mot_data[8].vel_des_, mot_data[8].ff_);
    }
    else if (index == 10)
    {
      slave = 1;
      send_motor_ctrl_cmd(&Tx_Message[slave], 6, 5, mot_data[9].kp_, mot_data[9].kd_, mot_data[9].pos_des_,
                          mot_data[9].vel_des_, mot_data[9].ff_);
    }
    else if (index == 11)
    {
    }

    if (index == 5 || index == 11)
    {
      EtherCAT_Msg* slave_dest = (EtherCAT_Msg*)(ec_slave[slave + 1].outputs);
      if (slave_dest)
        *(EtherCAT_Msg*)(ec_slave[slave + 1].outputs) = Tx_Message[slave];
    }
  }
  ec_send_processdata();
}

void EtherCAT_Command_Set()
{
  for (int slave = 0; slave < ec_slavecount; ++slave)
  {
    EtherCAT_Msg_ptr msg;
    if (messages[slave].pop(msg))
    {
      memcpy(&Tx_Message[slave], msg.get(), sizeof(EtherCAT_Msg));
      isConfig[slave] = true;
    }
    EtherCAT_Msg* slave_dest = (EtherCAT_Msg*)(ec_slave[slave + 1].outputs);
    if (slave_dest)
      *(EtherCAT_Msg*)(ec_slave[slave + 1].outputs) = Tx_Message[slave];
  }
}

void runImpl()
{
  while (running)
  {
    EtherCAT_Run();
  }
}

void startRun()
{
  running = true;
  runThread = std::thread(runImpl);
}
