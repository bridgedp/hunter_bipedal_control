/*
 * @Description:
 * @Author: kx zhang
 * @Date: 2022-09-13 19:00:55
 * @LastEditTime: 2022-11-13 17:09:03
 */


#include <cstdio>
#include "Console.hpp"
#include "command.h"

extern "C" {
#include "ethercat.h"
}

namespace cr = CppReadline;
using ret = cr::Console::ReturnCode;

int main()
{
  printf("SOEM 主站测试\n");

  EtherCAT_Init("enp9s0f3u1u3");

  if (ec_slavecount <= 0)
  {
    printf("未找到从站, 程序退出！");
    return 1;
  }
  else
    printf("从站数量： %d\r\n", ec_slavecount);

  startRun();

  cr::Console cli("[Command] > ");
  cli.registerCommand("help", help);
  cli.registerCommand("MotorIdGet", motorIdGet);
  cli.registerCommand("MotorIdSet", motorIdSet);
  cli.registerCommand("MotorSpeedSet", motorSpeedSet);
  cli.registerCommand("MotorPositionSet", motoPositionSet);

  int retCode;
  do
  {
    retCode = cli.readLine();
  } while (retCode != ret::Quit);

  runThread.join();
  return 0;
}
