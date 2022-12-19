/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

/**
 * read_load
 */

#include <cmath>
#include <iostream>
#include <fstream>
#include <functional>
#include <Eigen/Dense>

#include "ini.h"
#include "joint_motion.h"
#include "print_rci.h"
#include "rci_data/command_types.h"
#include "rci_data/robot_datas.h"
#include "robot.h"
#include "unistd.h"
#include "move.h"
#include "model.h"

using namespace Eigen;
using namespace xmate;
using JointControl = std::function<JointPositions(RCI::robot::RobotState robot_state)>;
int main(int argc, char *argv[]) {
    std::string ipaddr = "192.168.0.160";
    uint16_t port = 1338;

    std::string file = "../../xmate.ini";
    INIParser ini;
    if (ini.ReadINI(file)) {
        ipaddr = ini.GetString("network", "ip");
        port = static_cast<uint16_t>(ini.GetInt("network", "port"));
    }
    
    // RCI连接机器人
    xmate::Robot robot(ipaddr, port,XmateType::XMATE7_PRO);
    XmateModel xmatemodel(&robot, XmateType::XMATE7_PRO);
    //防止网络连接失败
    sleep(1);
    
    return 0;
}
