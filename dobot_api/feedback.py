from .base import DobotApi
import time
import numpy as np

MyType = np.dtype([('len', np.uint16,),
                   ('reserve', np.byte, (6, )),
                   ('DigitalInputs', np.uint64,),
                   ('DigitalOutputs', np.uint64,),
                   ('RobotMode', np.uint64,),
                   ('TimeStamp', np.uint64,),
                   ('RunTime', np.uint64,),
                   ('TestValue', np.uint64,),
                   ('reserve2', np.byte, (8, )),
                   ('SpeedScaling', np.float64,),
                   ('reserve3', np.byte, (16, )),
                   ('VRobot', np.float64, ),      
                   ('IRobot', np.float64,),
                   ('ProgramState', np.float64,),
                   ('SafetyOIn', np.uint16,),
                   ('SafetyOOut', np.uint16,),
                   ('reserve4', np.byte, (76, )),
                   ('QTarget', np.float64, (6, )),
                   ('QDTarget', np.float64, (6, )),
                   ('QDDTarget', np.float64, (6, )),
                   ('ITarget', np.float64, (6, )),
                   ('MTarget', np.float64, (6, )),
                   ('QActual', np.float64, (6, )),
                   ('QDActual', np.float64, (6, )),
                   ('IActual', np.float64, (6, )),
                   ('ActualTCPForce', np.float64, (6, )),
                   ('ToolVectorActual', np.float64, (6, )),
                   ('TCPSpeedActual', np.float64, (6, )),
                   ('TCPForce', np.float64, (6, )),
                   ('ToolVectorTarget', np.float64, (6, )),
                   ('TCPSpeedTarget', np.float64, (6, )),
                   ('MotorTemperatures', np.float64, (6, )),
                   ('JointModes', np.float64, (6, )),
                   ('VActual', np.float64, (6, )),
                   ('HandType', np.byte, (4, )),
                   ('User', np.byte,),
                   ('Tool', np.byte,),
                   ('RunQueuedCmd', np.byte,),
                   ('PauseCmdFlag', np.byte,),
                   ('VelocityRatio', np.byte,),
                   ('AccelerationRatio', np.byte,),
                   ('reserve5', np.byte, ),
                   ('XYZVelocityRatio', np.byte,),
                   ('RVelocityRatio', np.byte,),
                   ('XYZAccelerationRatio', np.byte,),
                   ('RAccelerationRatio', np.byte,),
                   ('reserve6', np.byte,(2,)),
                   ('BrakeStatus', np.byte,),
                   ('EnableStatus', np.byte,),
                   ('DragStatus', np.byte,),
                   ('RunningStatus', np.byte,),
                   ('ErrorStatus', np.byte,),
                   ('JogStatusCR', np.byte,),   
                   ('CRRobotType', np.byte,),
                   ('DragButtonSignal', np.byte,),
                   ('EnableButtonSignal', np.byte,),
                   ('RecordButtonSignal', np.byte,),
                   ('ReappearButtonSignal', np.byte,),
                   ('JawButtonSignal', np.byte,),
                   ('SixForceOnline', np.byte,),
                   ('CollisionState', np.byte,),
                   ('ArmApproachState', np.byte,),
                   ('J4ApproachState', np.byte,),
                   ('J5ApproachState', np.byte,),
                   ('J6ApproachState', np.byte,),
                   ('reserve7', np.byte, (61, )),
                   ('VibrationDisZ', np.float64,),
                   ('CurrentCommandId', np.uint64,),
                   ('MActual', np.float64, (6, )),
                   ('Load', np.float64,),
                   ('CenterX', np.float64,),
                   ('CenterY', np.float64,),
                   ('CenterZ', np.float64,),
                   ('UserValue[6]', np.float64, (6, )),
                   ('ToolValue[6]', np.float64, (6, )),
                   ('reserve8', np.byte, (8, )),
                   ('SixForceValue', np.float64, (6, )),
                   ('TargetQuaternion', np.float64, (4, )),
                   ('ActualQuaternion', np.float64, (4, )),
                   ('AutoManualMode', np.uint16, ),
                   ('ExportStatus', np.uint16, ),
                   ('SafetyState', np.byte, ),
                   ('reserve9', np.byte,(19,))
                   ])

class DobotApiFeedBack(DobotApi):
    def __init__(self, ip="127.0.0.1", port=30004, *args):
        super().__init__(ip, port, *args)
        self.__MyType = []
        self.last_recv_time = time.perf_counter()

    def feedBackData(self):
        """
        Return the robot status
        """
        self.socket_dobot.setblocking(True)  # Set to blocking mode
        data = bytes()
        current_recv_time = time.perf_counter() # Timing: get current time
        temp = self.socket_dobot.recv(144000) # Buffer
        if len(temp) > 1440:    
            temp = self.socket_dobot.recv(144000)
        #print("get:",len(temp))
        i=0
        if len(temp) < 1440:
            while i < 5 :
                #print("Receive again")
                temp = self.socket_dobot.recv(144000)
                print("get:",len(temp))
                if len(temp) >= 1440:
                    break
                i+=1
            if i >= 5:
                raise Exception("Packet loss while receiving data; please check the network environment")
        
        interval = (current_recv_time - self.last_recv_time) * 1000  # Convert to milliseconds
        self.last_recv_time = current_recv_time
        #print(f"Time interval since last receive: {interval:.3f} ms")
        
        data = temp[0:1440] # Slice 1440 bytes
        #print(len(data))
        #print(f"Single element size of MyType: {MyType.itemsize} bytes")
        self.__MyType = None   

        if len(data) == 1440:        
            self.__MyType = np.frombuffer(data, dtype=MyType)

        return self.__MyType