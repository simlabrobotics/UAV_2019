/* RoboticsLab, Copyright 2008-2016 Wonik Robotics. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics.
 */
#include "stdafx.h"

#include <rlab/math/rMath.h>
#include <rlab/command/rCmdDefine.h>
#include <rlab/utils/rPerformanceProbe.h>
#include <rlab/utils/rCustomDraw.h>
#include <rxSDK/rxSDK.h>
using namespace rlab;
using namespace rlab::rxsdk;
#include "UAV_protocol.h"
#include "UAV_cmd.h"
#include "UAV_conf.h"
#include "UAV.h"
extern UAVConf appConf;

