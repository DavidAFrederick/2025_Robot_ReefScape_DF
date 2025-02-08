# from enum import Enum
from commands2 import Subsystem, Command, RunCommand
from wpilib import SmartDashboard, RobotBase, RobotController, DutyCycleEncoder
from phoenix5 import TalonSRX, TalonSRXConfiguration, ControlMode, TalonSRXControlMode
import constants
from commands2.button import CommandXboxController


class Wrist(Subsystem):
    """
    Test class for shooter prototype
    """

    def __init__(self):
        super().__init__()
        self.Wrist_Motor: TalonSRX = TalonSRX(constants.WRIST_MOTOR)
        self.Wrist_Motor.configFactoryDefault()

        ##  Absolute Encoder Angle - Example code from Crescendo
        self._wrist_angle: DutyCycleEncoder = self.__configure_wrist_encoder()

        ##  Absolute Encoder Angle - Example code from Crescendo
    def __configure_wrist_encoder(self) -> DutyCycleEncoder:
        wrist_encoder: DutyCycleEncoder = DutyCycleEncoder(constants.WRIST_ANGLE_ENCODER)  # DIO port [0]
        return wrist_encoder


    def drive_motor(self, speed: float):
        self.Wrist_Motor.set(TalonSRXControlMode.PercentOutput, speed)
        SmartDashboard.putNumber("Wrist_Speed", speed)

    def stop_motor(self) -> None:
        self.Wrist_Motor.set(TalonSRXControlMode.PercentOutput, 0)

    def getAbsolutePosition(self) -> float:
        return (360 * self._wrist_angle.get())
        

    def periodic(self) -> None:
        ##  Absolute Encoder Angle - Example code from Crescendo
        if self._wrist_angle.isConnected():
            SmartDashboard.putNumber(
                "Encoder Pos", self.getAbsolutePosition()
            )

    def move_wrist_to_angle(self, target_angle: float) -> None:
        speed = 0
        current_angle = self.getAbsolutePosition()
        if (current_angle <= target_angle):
            speed = 1
        elif (current_angle > target_angle):
            speed = -1
        else:
            speed = 0
        self.Wrist_Motor.set(ControlMode.PercentOutput, speed)

    def wrist_at_angle(self, target_angle: float) -> bool:
        current_difference = (self._wrist_angle.getAbsolutePosition() - target_angle)
        SmartDashboard.putNumber("Wrist angle error", current_difference)
        return abs(current_difference) < 1   #  TBD - tune this number

#======================================================================
class SetWristAngle(Command):
    def __init__(self, Wrist: Wrist, angle: float):
        self._Wrist = Wrist
        self.angle = angle
        self.addRequirements(self._Wrist)

    def initialize(self):
        pass 

    def execute(self):
        self._Wrist.move_wrist_to_angle(self.angle)
       
    def isFinished(self) -> bool:
        return self._Wrist.wrist_at_angle(self.angle)
    
    def end(self, interrupted: bool):
        self._Wrist.stop_motor()

#=========================================

class ManualControlWristAngle(Command):
    def __init__(self, Wrist: Wrist, speed: float):
        self._Wrist = Wrist
        self.speed = speed
        self.addRequirements(self._Wrist)

    def initialize(self):
        pass 

    def execute(self):
        self._Wrist.drive_motor(self.speed)
       
    def isFinished(self) -> bool:
        return True
    
    def end(self, interrupted: bool):
        self._Wrist.stop_motor()

#=========================================