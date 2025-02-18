import wpilib
from commands2 import Subsystem, Command
from commands2.button import CommandXboxController
from wpilib import SmartDashboard


from enum import Enum

from phoenix6.configs import TalonFXConfiguration
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.signals.spn_enums import (InvertedValue, NeutralModeValue, FeedbackSensorSourceValue, ForwardLimitValue )
from phoenix6.controls import DutyCycleOut
from phoenix6 import signals

import constants

#==================================================================================
#===(SubSystems)===================================================================

class Elevator2(Subsystem):


    def __init__(self) -> None:
        super().__init__()         # Call the parent's (Super) initialization function
        # Configure the motor
        self._configure_elevator_motor_FX()
        print ("Elevator  Subsystem Initialization complete")
        
        if self.elevator_at_bottom():
            self.Elevator_started_at_bottom = True  # Remember if elevator started at the bottom
        else:
            self.Elevator_started_at_bottom = False 


    def _configure_elevator_motor_FX(self) -> None:
        self._elevator_motor = TalonFX(constants.ELEVATOR_CAN_ADDRESS)     #  the CAN bus address
        # Applying a new configuration will erase all other config settings since
        # we start with a blank config so each setting needs to be explicitly set
        # here in the config method
        config = TalonFXConfiguration()

        config.current_limits.supply_current_limit = 4
        config.current_limits.supply_current_limit_enable = True

        # Set the motor direction counter clockwise positive  (if needed)
        # config.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        config.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        # Set the motors to electrically stop instead of coast
        config.motor_output.neutral_mode = NeutralModeValue.BRAKE

        # This configuration item supports counting motor shaft rotations
        # This item sets the gear ratio between motor turns and elevator drive shaft
        config.feedback.feedback_sensor_source = FeedbackSensorSourceValue.ROTOR_SENSOR
        config.feedback.sensor_to_mechanism_ratio = -1

        # Apply the configuration to the motors
        for i in range(6):  # Try 5 times
            ret = self._elevator_motor.configurator.apply(config)

        #  Reset the encoder to  zero
        self._elevator_motor.set_position(0)    

   ########################### Elevator Drive methods #######################

    def DriveElevatorSpeed(self, speed: float):

        # speed = float(speed)
        # Limit the speed of the motor
        if (speed > 0.0):
            speed = speed * constants.ELEVATOR_UP_SPEED_SCALER
        elif (speed < 0.0):
            speed = speed * constants.ELEVATOR_DOWN_SPEED_SCALER

        # Use percent output (-1 to +1 ) to control the motor
        self._motor_percent_out: DutyCycleOut = DutyCycleOut(0)
        self._motor_percent_out.output = speed
        self._elevator_motor.set_control(self._motor_percent_out)
        # print ("Moving: ", speed)

    def get_elevator_encoder_count(self) -> float:
        self._elevator_motor.get_rotor_position()
        return self._elevator_motor.get_position().value

     #  Reset the encoder to  zero
    def reset_elevator_encoder_count(self):
        self._elevator_motor.set_position(0)   

    def get_motor_current(self) -> float:
        return self._elevator_motor.get_stator_current().value_as_double

    def elevator_at_top(self) -> bool:
        reverse_limit = self._elevator_motor.get_reverse_limit()
        return (reverse_limit.value is signals.ReverseLimitValue.CLOSED_TO_GROUND)

    def elevator_at_bottom(self) -> bool:
        forward_limit = self._elevator_motor.get_forward_limit()
        return (forward_limit.value is signals.ForwardLimitValue.CLOSED_TO_GROUND)

    def periodic(self) -> None:
        SmartDashboard.putBoolean("Forward (top) Limit", self.elevator_at_top())  
        SmartDashboard.putBoolean("Reverse (Bottom) Limit", self.elevator_at_bottom())  
        SmartDashboard.putNumber("Motor Current",  self._elevator_motor.get_stator_current().value_as_double)
        SmartDashboard.putNumber("Counter",        self._elevator_motor.get_position().value_as_double)

    def set_flag_that_elevator_was_reset_at_the_bottom(self) -> None:
        self.Elevator_started_at_bottom = True

    def check_flag_that_elevator_was_reset_at_the_bottom(self) -> bool:
        return self.Elevator_started_at_bottom


#==================================================================================
#===(Commands)=====================================================================

class DriveElevatorManual(Command):
    def __init__(self, elevator: Elevator2, speed: float):
        self.elevator = elevator
        self.addRequirements(self.elevator)
        self.speed = speed
        # print ("DriveElevatorManual Command Instantiated")

    def initialize(self):
        # print ("DriveElevatorManual Command Initialized")
        pass
        
    def execute(self):
        # joystickValue = -self.controller.getLeftY()
        # self.elevator.DriveElevatorSpeed(self.controller.getLeftY())
        self.elevator.DriveElevatorSpeed(self.speed)
        
        SmartDashboard.putNumber("Joystick value", self.speed)
        

    def isFinished(self) -> bool:      # This command never finishes
        return False
  
    def end(self, interrupted: bool):    # If interupted by another command the motor is stopped
        self.elevator.DriveElevatorSpeed(0)

#==================================================================================

# class MoveElevatorUpXCounts(Command):
#     def __init__(self, elevator: Elevator2, counts: float):
#         self.elevator = elevator
#         self.counts = counts
#         self.addRequirements(self.elevator)
#         self.current_position = 0

#     def initialize(self):
#         # Get the current elevator position
#         self.current_position = self.elevator.get_elevator_encoder_count()
        
#     def execute(self):
#         self.elevator.DriveElevatorSpeed(0.2)

#     def isFinished(self) -> bool:      
#         return ( self.elevator.get_elevator_encoder_count() >= self.current_position + self.counts )
  
#     def end(self, interrupted: bool):    # If interupted by another command the motor is stopped
#         self.elevator.DriveElevatorSpeed(0)

#==================================================================================

# class ResetElevatorCount(Command):
#     def __init__(self, elevator: Elevator2):
#         self.elevator = elevator
#         self.addRequirements(self.elevator)

#     def initialize(self):
#         self.elevator.reset_elevator_encoder_count()
        
#     def execute(self):
#         pass

#     def isFinished(self) -> bool:      # This command never finishes
#         return True
  
#     def end(self, interrupted: bool):    # If interupted by another command the motor is stopped
#         pass

#==================================================================================

class MoveToLowerLimitAndResetCounter(Command):
    def __init__(self, elevator: Elevator2):
        self.elevator = elevator
        self.addRequirements(self.elevator)
        # print ("ResetElevatorCount Command Instantiated")

    def initialize(self):
        self.elevator.DriveElevatorSpeed(-constants.ELEVATOR_SEEK_LOWER_LIMIT_SPEED)        # Start the elevator downward slowly
        
    def execute(self):
        pass

    def isFinished(self) -> bool:      
        return (self.elevator.elevator_at_bottom())
  
    def end(self, interrupted: bool):    # If interupted by another command the motor is stopped
        self.elevator.reset_elevator_encoder_count()
        self.elevator.set_flag_that_elevator_was_reset_at_the_bottom()
        SmartDashboard.putNumber("Counter", 0)

#==================================================================================

# class AutonomousElevatorCommand(Command):
#     def __init__(self, elevator: Elevator2):
#         self.elevator = elevator
#         self.addRequirements(self.elevator)
#         # print ("AutonomousElevatorCommand Command Instantiated")

#     def initialize(self):
#         # print ("AutonomousElevatorCommand Command Initialized")
#         pass

#     def execute(self):
#         self.elevator.DriveElevatorSpeed(0.2)

#     def isFinished(self) -> bool:      # This command never finishes
#         return False
  
#     def end(self, interrupted: bool):    # If interupted by another command the motor is stopped
#         self.elevator.DriveElevatorSpeed(0)
#==================================================================================

class MoveElevatorToSetPointX(Command):
    def __init__(self, elevator: Elevator2, target_position: float):
        self.elevator = elevator
        self.target_position = target_position
        self.addRequirements(self.elevator)
        self.current_position = 0

    #  Need to add a check that the elevator was started at the bottom
    ####  check_flag_that_elevator_was_reset_at_the_bottom()

    def initialize(self):
        print ("Move Elevator To Set Point X Command   <<<<<<<<<<<<<<<<<<<<<<<<<<<<xx<< ", self.target_position)
        # Get the current elevator position
        self.current_position = self.elevator.get_elevator_encoder_count()
        
        if not self.elevator.check_flag_that_elevator_was_reset_at_the_bottom():
            print ("Elevator not started at the bottom !!!!!!!!!!!!!!!!!!")
            # MoveToLowerLimitAndResetCounter(self.elevator)
    
    def execute(self):
        self.current_position = self.elevator.get_elevator_encoder_count()

        if (self.current_position > self.target_position):          # Need to move down (FORWARD ON REAL ROBOT)
            self.elevator.DriveElevatorSpeed(1)
        else:
            self.elevator.DriveElevatorSpeed(-1)
        print ("Executing", self.target_position)

    def isFinished(self) -> bool:      
        stopBand = 2
        return ( abs(self.target_position - self.current_position) < stopBand)
    
    def end(self, interrupted: bool):    # If interupted by another command the motor is stopped
        self.elevator.DriveElevatorSpeed(0)

#==================================================================================

class MoveElevatorToSetPoint50(Command):
    def __init__(self, elevator: Elevator2):
        self.elevator = elevator
        # self.target_position = target_position
        self.target_position = 50
        self.addRequirements(self.elevator)
        self.current_position = 0

    def initialize(self):
        print ("Move Elevator To Set Point X Command   <<<<<<<<<<<<<<<<<<<<<<<<<<<<xx<< ", self.target_position)
        # Get the current elevator position
        self.current_position = self.elevator.get_elevator_encoder_count()
        
        
    def execute(self):
        self.current_position = self.elevator.get_elevator_encoder_count()

        if (self.current_position > self.target_position):          # Need to move down (FORWARD ON REAL ROBOT)
            self.elevator.DriveElevatorSpeed(1)
        else:
            self.elevator.DriveElevatorSpeed(-1)
        print ("Executing", self.target_position)

    def isFinished(self) -> bool:      
        stopBand = 2
        return ( abs(self.target_position - self.current_position) < stopBand)
    
    def end(self, interrupted: bool):    # If interupted by another command the motor is stopped
        self.elevator.DriveElevatorSpeed(0)

#==================================================================================
#==================================================================================

class Move_Elevator_L1(Command):
    def __init__(self, elevator: Elevator2):
        self.elevator = elevator
        self.target_position = constants.L1
        self.addRequirements(self.elevator)
        self.current_position = 0
        self.elevator_up_speed = 1
        self.elevator_down_speed = 0.7
        self.stopBand = 2

    def initialize(self):
        print ("Move Elevator To Set Point L1 Command ")
        # Get the current elevator position
        self.current_position = self.elevator.get_elevator_encoder_count()
                
    def execute(self):
        self.current_position = self.elevator.get_elevator_encoder_count()

        if (self.current_position > self.target_position):          # Need to move down (FORWARD ON REAL ROBOT)
            self.elevator.DriveElevatorSpeed(self.elevator_up_speed)
        else:
            self.elevator.DriveElevatorSpeed(-self.elevator_down_speed)
        # print ("Executing", self.target_position)

    def isFinished(self) -> bool:      
        return ( abs(self.target_position - self.current_position) < self.stopBand)
    
    def end(self, interrupted: bool):    # If interupted by another command the motor is stopped
        self.elevator.DriveElevatorSpeed(0)

#==================================================================================
class Move_Elevator_L4(Command):
    def __init__(self, elevator: Elevator2):
        self.elevator = elevator
        self.target_position = constants.L4
        self.addRequirements(self.elevator)
        self.current_position = 0
        self.elevator_up_speed = 1
        self.elevator_down_speed = 0.7
        self.stopBand = 2

    def initialize(self):
        print ("Move Elevator To Set Point L3 Command ")
        # Get the current elevator position
        self.current_position = self.elevator.get_elevator_encoder_count()
                
    def execute(self):
        self.current_position = self.elevator.get_elevator_encoder_count()

        if (self.current_position > self.target_position):          # Need to move down (FORWARD ON REAL ROBOT)
            self.elevator.DriveElevatorSpeed(self.elevator_up_speed)
        else:
            self.elevator.DriveElevatorSpeed(-self.elevator_down_speed)
        # print ("Executing", self.target_position)

    def isFinished(self) -> bool:      
        return ( abs(self.target_position - self.current_position) < self.stopBand)
    
    def end(self, interrupted: bool):    # If interupted by another command the motor is stopped
        self.elevator.DriveElevatorSpeed(0)

#==================================================================================
class Move_Elevator_CS(Command):
    def __init__(self, elevator: Elevator2):
        self.elevator = elevator
        self.target_position = constants.CS
        self.addRequirements(self.elevator)
        self.current_position = 0
        self.elevator_up_speed = 1
        self.elevator_down_speed = 0.7
        self.stopBand = 2

    def initialize(self):
        print ("Move Elevator To Set Point Coral Station Command ")
        # Get the current elevator position
        self.current_position = self.elevator.get_elevator_encoder_count()
                
    def execute(self):
        self.current_position = self.elevator.get_elevator_encoder_count()

        if (self.current_position > self.target_position):          # Need to move down (FORWARD ON REAL ROBOT)
            self.elevator.DriveElevatorSpeed(self.elevator_up_speed)
        else:
            self.elevator.DriveElevatorSpeed(-self.elevator_down_speed)
        # print ("Executing", self.target_position)

    def isFinished(self) -> bool:      
        return ( abs(self.target_position - self.current_position) < self.stopBand)
    
    def end(self, interrupted: bool):    # If interupted by another command the motor is stopped
        self.elevator.DriveElevatorSpeed(0)

#==================================================================================

#==================================================================================
class AutonomouseModeComplete(Command):
    def __init__(self, elevator: Elevator2):
        self.elevator = elevator
        self.addRequirements(self.elevator)

    def initialize(self):
        pass
                
    def execute(self):
        pass

    def isFinished(self) -> bool:      
        return True
    
    def end(self, interrupted: bool):   
        print ("Autonomous Complete <=============")

#==================================================================================


#==================================================================================



#==================================================================================
