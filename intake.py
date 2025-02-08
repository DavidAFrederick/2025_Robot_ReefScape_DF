# from enum import Enum
from commands2 import Subsystem, Command, RunCommand
from wpilib import SmartDashboard, RobotBase, RobotController, DutyCycleEncoder
from phoenix5 import TalonSRX, TalonSRXConfiguration, ControlMode, TalonSRXControlMode
import constants
from commands2.button import CommandXboxController


class Intake(Subsystem):
    """
    Test class for shooter prototype
    """

    def __init__(self):
        super().__init__()
        self.Intake_Motor: TalonSRX = TalonSRX(constants.INTAKE_MOTOR)
        self.Intake_Motor.configFactoryDefault()

    def drive_motor(self, speed: float):
        # self.Intake_Motor.set(ControlMode.PercentOutput, speed)
        self.Intake_Motor.set(TalonSRXControlMode.PercentOutput, speed)
        SmartDashboard.putNumber("Intake_Speed", speed)

    def stop_motor(self) -> None:
        self.Intake_Motor.set(TalonSRXControlMode.PercentOutput, 0)

    def periodic(self) -> None:
        pass

class SetIntake(Command):
    def __init__(self, Intake: Intake, speed: float):
        self._Intake = Intake
        self.speed = speed
        self.addRequirements(self._Intake)

    def initialize(self):
        pass 

    def execute(self):
        self._Intake.drive_motor(self.speed)
       
    def isFinished(self) -> bool:
        return False
    
    def end(self, interrupted: bool):
        self._Intake.stop_motor()

#=========================================
## getLeftTriggerAxis()
class SetIntakeUsingAnalogLeftTrigger(Command):
    def __init__(self, Intake: Intake, _controller: CommandXboxController):
        self._Intake = Intake
        self._controller = _controller
        self.addRequirements(self._Intake)

    def initialize(self):
        pass 

    def execute(self):
        self._Intake.drive_motor(self._controller.getLeftTriggerAxis())
       
    def isFinished(self) -> bool:
        return False
    
    def end(self, interrupted: bool):
        self._Intake.stop_motor()
