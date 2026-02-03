from .base import DobotApi

import requests
import json

'''
Note for more functions to explore from the origial Dobot Dashboard API:
ClearError, DO, GetError
'''

class DobotDashboard(DobotApi):
    """
    Dobot Dashboard API class.

    This class provides methods to interact with the Dobot's dashboard functionalities.
    It extends the DobotBase class to utilize common communication methods.
    """

    def __init__(self, ip: str = "127.0.0.1", port: int = 29999, *args):
        super().__init__(ip, port, *args)
    
    ###############################
    # Initialization and Shutdown #
    ###############################
    def EnableRobot(self, load=0.0, centerX=0.0, centerY=0.0, centerZ=0.0, isCheck=-1,):
        """
            Parameter     Description
            load          Load weight. The value range should not exceed the load range of corresponding robot models. Unit: kg.
            centerX       X-direction eccentric distance. Range: -999 to 999, unit: mm.
            centerY       Y-direction eccentric distance. Range: -999 to 999, unit: mm.
            centerZ       Z-direction eccentric distance. Range: -999 to 999, unit: mm.
            isCheck       Check the load or not. 1: check, 0: not check. If set to 1, the robot arm will check actual load with load parameter
            The number of parameters that can be contained is as follows:
            0: no parameter (not set load weight and eccentric parameters when enabling the robot).
            1: one parameter (load weight).
            4: four parameters (load weight and eccentric parameters).
            5: five parameters (load weight, eccentric parameters, check the load or not).
        """
        string = 'EnableRobot('
        if load != 0:
            string = string + "{:f}".format(load)
            if centerX != 0 or centerY != 0 or centerZ != 0:
                string = string + ",{:f},{:f},{:f}".format(
                    centerX, centerY, centerZ)
                if isCheck != -1:
                    string = string + ",{:d}".format(isCheck)
        string = string + ')'
        return self.sendRecvMsg(string)

    def DisableRobot(self):
        string = "DisableRobot()"
        return self.sendRecvMsg(string)
    
    def ClearError(self):
        """
        Clear controller alarm information
        Clear the alarms of the robot. After clearing the alarm, you can judge whether the robot is still in the alarm status according to RobotMode.
        Some alarms cannot be cleared unless you resolve the alarm cause or restart the controller.
        """
        string = "ClearError()"
        return self.sendRecvMsg(string)
    
    ##################################
    # Administrative Motion Commands #
    ##################################
    def Stop(self):
        """
        Stop the delivered motion command queue or the RunScript command from running.
        """
        string = "Stop()"
        return self.sendRecvMsg(string)

    def Pause(self):
        """
        Pause the delivered motion command queue or the RunScript command from running.
        """
        string = "Pause()"
        return self.sendRecvMsg(string)

    def Continue(self):
        """
        Continue the paused motion command queue or the RunScript command from running.
        """
        string = "Continue()"
        return self.sendRecvMsg(string)
    
    def VelJ(self, speed):
        """
        Set the speed ratio of joint motion.
        Defaults to 100 if not set.
        """
        string = "VelJ({:d})".format(speed)
        return self.sendRecvMsg(string)
    
    def SpeedFactor(self, speed):
        """
        Set the global speed ratio.
        e.g. joint speed = joint speed * speed factor / 100
        e.g. linear speed = linear speed * speed factor / 100
        Range: [1, 100].
        """
        string = "SpeedFactor({:d})".format(speed)
        return self.sendRecvMsg(string)
    
    ##################
    # Robot Maneuver #
    ##################
    def RobotMode(self):
        """
        Get the current status of the robot.
        1 ROBOT_MODE_INIT  Initialized status
        2 ROBOT_MODE_BRAKE_OPEN  Brake switched on
        3 ROBOT_MODE_POWEROFF  Power-off status
        4 ROBOT_MODE_DISABLED  Disabled (no brake switched on
        5 ROBOT_MODE_ENABLE  Enabled and idle
        6 ROBOT_MODE_BACKDRIVE  Drag mode
        7 ROBOT_MODE_RUNNING  Running status (project, TCP queue)
        8 ROBOT_MODE_SINGLE_MOVE  Single motion status (jog, RunTo)
        9 ROBOT_MODE_ERROR
             There are uncleared alarms. This status has the highest priority. It returns 9 when there is an alarm, regardless of the status of the robot arm.
        10 ROBOT_MODE_PAUSE  Pause status
        11 ROBOT_MODE_COLLISION  Collision status
        """
        string = "RobotMode()"
        return self.sendRecvMsg(string)
    
    def GetAngle(self):
        """
        Get the joint coordinates of current posture.
        """
        string = "GetAngle()"
        return self.sendRecvMsg(string)
    
    def GetToolDO(self, index):
        """
        Set the status of tool digital output port (immediate command)
        Required parameter:
        Parameter name     Type    Description
        index               int     index of the tool DO
        status              int     status of the tool DO, 1: ON, 0: OFF
        """
        string = "GetToolDO({:d})".format(index)
        return self.sendRecvMsg(string)
    
    def StartDrag(self):
        string = "StartDrag()"
        return self.sendRecvMsg(string)

    def StopDrag(self):
        string = "StopDrag()"
        return self.sendRecvMsg(string)
    
    def CP(self, ratio):
        """
        Set smooth transition ratio
        ratio : Smooth transition ratio (Value range:1~100)
        """
        string = "CP({:d})".format(ratio)
        return self.sendRecvMsg(string)
    
    # Main function for VLA to perform joint motion in high frequency.
    def ServoJ(self, j1, j2, j3, j4, j5, j6, t=0.1, lookahead_time=50, gain=500):
        # string = "ServoJ({:f},{:f},{:f},{:f},{:f},{:f},t={:f},lookahead_time={:f},gain={:f})".format(
        #     j1, j2, j3, j4, j5, j6, t, lookahead_time, gain)
        
        string = "ServoJ({:f},{:f},{:f},{:f},{:f},{:f})".format(
            j1, j2, j3, j4, j5, j6)
        
        return self.sendRecvMsg(string)
    
    # Main function for VLA to perform relative joint motion.
    def RelJointMovJ(self, j1, j2, j3, j4, j5, j6, a=-1, v=-1):
        """
        Description
        Perform relative motion along the joint coordinate system, and the end motion is joint motion.
        Required parameter:
        Parameter name     Type     Description
        offset1     double     J1-axis offset, unit: °
        offset2     double     J2-axis offset, unit: °
        offset3     double     J3-axis offset, unit: °
        offset4     double     J4-axis offset, unit: °
        offset5     double     J5-axis offset, unit: °
        offset6     double     J6-axis offset, unit: °
        Optional parameter:
        Parameter name     Type     Description
        a     int     acceleration rate of the robot arm when executing this command. Range: (0,100].
        v     int     velocity rate of the robot arm when executing this command. Range: (0,100].
        """
        string = "RelJointMovJ({:f},{:f},{:f},{:f},{:f},{:f}".format(
            j1, j2, j3, j4, j5, j6)
        params = []
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1:
            params.append('v={:d}'.format(v))
        for ii in params:
            print(ii)
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)
    
    # We will only use this for move to home position.
    def MovJ(self, a1, b1, c1, d1, e1, f1, coordinateMode, user=-1, tool=-1, a=-1, v=-1, cp=-1):
        """
        Description
        Move from the current position to the target position through joint motion.
        
        Required parameter:
        Parameter name     Type        Description
        P                   string      Target point (joint variables or posture variables)
        coordinateMode      int         Coordinate mode of the target point, 0: pose, 1: joint
        
        Optional parameter:
        Parameter name     Type        Description
        user                int         user coordinate system
        tool                int         tool coordinate system
        a                   int         acceleration rate of the robot arm when executing this command. Range: (0,100].
        v                   int         velocity rate of the robot arm when executing this command. Range: (0,100].
        cp                  int         continuous path rate. Range: [0,100].
        """
        string = ""
        if coordinateMode == 0:
            # string = "MovJ(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(a1, b1, c1, d1, e1, f1)

            # Our dobot is in the firmware version that uses this format:
            string = "MovJ({:f},{:f},{:f},{:f},{:f},{:f}".format(
                a1, b1, c1, d1, e1, f1)
        else:
            print("coordinateMode param is wrong")
            return ""
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1:
            params.append('v={:d}'.format(v))
        if cp != -1:
            params.append('cp={:d}'.format(cp))
        for ii in params:
            string = string + ','+ii
        string = string + ')'
        return self.sendRecvMsg(string)
    
    # Control End Effector Digital Output
    def ToolDOInstant(self, index, status):
        """
        Set the status of tool digital output port (immediate command)
        Required parameter:
        Parameter name     Type     Description
        index     int     index of the tool DO
        status     int     status of the tool DO, 1: ON, 0: OFF
        """
        string = "ToolDOInstant({:d},{:d})".format(index, status)
        return self.sendRecvMsg(string)
    
    ##################
    # Error Handling #
    ##################
    def GetError(self, language="en"):
        """
        Get robot alarm information.
        Parameters:
        language: language setting. Supported values:
                 "zh_cn" - Simplified Chinese
                 "zh_hant" - Traditional Chinese
                 "en" - English
                 "ja" - Japanese
                 "de" - German
                 "vi" - Vietnamese
                 "es" - Spanish
                 "fr" - French
                 "ko" - Korean
                 "ru" - Russian
        Returns:
        dict: dictionary containing alarm information, format:
        {
            "errMsg": [
                {
                    "id": xxx,
                    "level": xxx,
                    "description": "xxx",
                    "solution": "xxx",
                    "mode": "xxx",
                    "date": "xxxx",
                    "time": "xxxx"
                }
            ]
        }
        """
        try:
            language_url = f"http://{self.ip}:22000/interface/language"
            language_data = {"type": language}
            
            response = requests.post(language_url, json=language_data, timeout=5)
            if response.status_code != 200:
                print(f"Failed to set language: HTTP {response.status_code}")
            
            alarm_url = f"http://{self.ip}:22000/protocol/getAlarm"
            response = requests.get(alarm_url, timeout=5)
            
            if response.status_code == 200:
                return response.json()
            else:
                print(f"Failed to get alarm information: HTTP {response.status_code}")
                return {"errMsg": []}
                
        except requests.exceptions.RequestException as e:
            print(f"HTTP request error: {e}")
            return {"errMsg": []}
        except json.JSONDecodeError as e:
            print(f"JSON parse error: {e}")
            return {"errMsg": []}
        except Exception as e:
            print(f"Unknown error while getting alarm information: {e}")
            return {"errMsg": []}
        
    def GetErrorID(self):
        """
        Get the joint coordinates of current posture.
        """
        string = "GetErrorID()"
        return self.sendRecvMsg(string)