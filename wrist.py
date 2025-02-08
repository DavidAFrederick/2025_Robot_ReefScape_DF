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
        encoder: DutyCycleEncoder = DutyCycleEncoder(constants.WRIST_ANGLE_ENCODER)  # DIO port [0]
        return encoder


    def drive_motor(self, speed: float):
        self.Wrist_Motor.set(TalonSRXControlMode.PercentOutput, speed)
        SmartDashboard.putNumber("Wrist_Speed", speed)

    def stop_motor(self) -> None:
        self.Wrist_Motor.set(TalonSRXControlMode.PercentOutput, 0)

    def periodic(self) -> None:
        ##  Absolute Encoder Angle - Example code from Crescendo
        if self._wrist_angle.isConnected():
            SmartDashboard.putNumber(
                "Encoder Pos", self._wrist_angle.getAbsolutePosition()
            )

    ##  Absolute Encoder Angle - Example code from Crescendo

    def move_ramp_to_location(self) -> None:
        speed = 0
        curr_location = self._shooter_ramp_angle.getAbsolutePosition()
        if curr_location < (self._curr_location.value[LOCATION] - 0.001):
            # Ramp needs to move up
            speed = 1
        elif curr_location > (self._curr_location.value[LOCATION] - 0.001):
            # Ramp needs to move down
            speed = -1
        else:
            speed = 0
        self._shooter_ramp.set(ControlMode.PercentOutput, speed)



    def shooter_at_angle(self) -> bool:
        curr_diff = (
            self._shooter_ramp_angle.getAbsolutePosition()
            - self._curr_location.value[LOCATION]
        )
        SmartDashboard.putNumber("ShooterDiff", curr_diff)
        return abs(curr_diff) < 0.0015

class SetWristAngle(Command):
    def __init__(self, Wrist: Wrist, angle: float):
        self._Wrist = Wrist
        self.speed = angle
        self.addRequirements(self._Wrist)

    def initialize(self):
        pass 

    def execute(self):
        self._Wrist.drive_motor(self.speed)
       
    def isFinished(self) -> bool:
        return False
    
    def end(self, interrupted: bool):
        self._Wrist.stop_motor()

#=========================================
