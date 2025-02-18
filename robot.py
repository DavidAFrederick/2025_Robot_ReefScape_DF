#!/usr/bin/env python3
# Feb 12, 2025   python3 -m robotpy installer niweb disable
import math
import wpilib
from wpilib import RobotBase, DriverStation
from commands2 import (
    TimedCommandRobot,
    CommandScheduler,
    Command,
    PrintCommand,
    RunCommand,
    WaitCommand,
    cmd,
)
from commands2.button import CommandXboxController, Trigger, JoystickButton
from wpimath.geometry import Pose2d
from pathplannerlib.auto import (
    NamedCommands,
    PathPlannerAuto,
)
from phoenix6 import SignalLogger
from drivetrain import DriveTrain,  TurnToAnglePID, AutonomouseModeComplete
from intake import Intake, SetIntake, SetIntakeUsingAnalogLeftTrigger, SetIntakeSpeedandTime
from wrist import Wrist, SetWristAngle
# from leds import LEDSubsystem, FlashLEDCommand
####>>> from vision import VisionSystem

from wpilib import SmartDashboard

from elevatorsubsystem import (
    Elevator2, 
    DriveElevatorManual, 
    MoveElevatorUpXCounts, 
    AutonomousElevatorCommand, 
    ResetElevatorCount, 
    MoveToLowerLimitAndResetCounter,
    MoveElevatorToSetPointX,
    Move_Elevator_L1,  
    Move_Elevator_L4,      
    Move_Elevator_CS,
    )

from wrist import Wrist, SetWristAngle, ManualControlWristAngle, SetWristAngle

import constants
from typing import Tuple, List


class MyRobot(TimedCommandRobot):
    """Class that defines the totality of our Robot"""

    def robotInit(self) -> None:
        """
        This method must eventually exit in order to ever have the robot
        code light turn green in DriverStation. So, this will create an
        instance of the Robot that contains all the subsystems,
        button bindings, and operator interface pieces like driver
        dashboards
        """
        # Disable the CTRE signal logger
        SignalLogger.stop()  # Disable for debugging later on

        # Setup the operator interface (typically CommandXboxController)
        self._driver_controller = CommandXboxController(
            constants.CONTROLLER_DRIVER_PORT
        )
        self._partner_controller = CommandXboxController(
            constants.CONTROLLER_PARTNER_PORT
        )

        # Remove the joystick warnings in sim
        if RobotBase.isSimulation():
            DriverStation.silenceJoystickConnectionWarning(True)

        #======== Instantiate any subystems  =============================
        self._drivetrain: DriveTrain = DriveTrain()
        wpilib.SmartDashboard.putData("Drivetrain", self._drivetrain)

        self._intake: Intake = Intake()
        wpilib.SmartDashboard.putData("Intake", self._intake)

        self.elevatorSubSys: Elevator2 = Elevator2()
        SmartDashboard.putData("Elevator", self.elevatorSubSys)

        self.wristSubSys: Wrist = Wrist()
        SmartDashboard.putData("Wrist", self.wristSubSys)

        # self._vision: VisionSystem = VisionSystem(True, True)
        # self._leds: LEDSubsystem = LEDSubsystem()

        self.__configure_default_commands()

        self.__configure_button_bindings()

        self.__configure_autonomous_commands()

        self.__configure_led_triggers()

        self._auto_command = None
        self._current_pose = Pose2d()

    def __configure_button_bindings(self) -> None:

        ######################## Driver controller controls #########################

        #   Driver Joysticks used for driving

        self._driver_controller.povUp().onTrue(SetWristAngle(self.wristSubSys, 10))
        self._driver_controller.povDown().onTrue(SetWristAngle(self.wristSubSys, 60))

        ######################## Partner controller controls #########################

        #==(Intake Control)========================
        self._partner_controller.rightTrigger().whileTrue(
            SetIntake(self._intake, 0.8).withName("Intake In")
        )
        self._partner_controller.leftTrigger().whileTrue(
            SetIntake(self._intake, -0.8).withName("Intake Out")
        )

        self._partner_controller.povUp().onTrue(SetIntakeSpeedandTime(self._intake, 1, 2))   # push out a coral at 100% speed for 2 seconds
        self._partner_controller.povDown().onTrue(SetIntakeSpeedandTime(self._intake, -1, 2))  # pull in a coral

        #==(Elevator Control)========================
        # Bumpers - Elevator Control

        self._partner_controller.rightBumper().whileTrue(
            DriveElevatorManual(self.elevatorSubSys, 0.4).withName("Elevator Up")
        )
        self._partner_controller.leftBumper().whileTrue(
            DriveElevatorManual(self.elevatorSubSys, -0.4).withName("Elevator Down")
        )

        # Bind the buttons to the commands
        self._partner_controller.y().onTrue(MoveElevatorToSetPointX(self.elevatorSubSys, constants.L1))
        self._partner_controller.x().onTrue(MoveElevatorToSetPointX(self.elevatorSubSys, constants.L2))
        self._partner_controller.b().onTrue(MoveElevatorToSetPointX(self.elevatorSubSys, constants.L3))
        self._partner_controller.a().onTrue(MoveElevatorToSetPointX(self.elevatorSubSys, constants.L4))
        
        self._partner_controller.start().onTrue(MoveToLowerLimitAndResetCounter(self.elevatorSubSys))

    # ====== (Configure default commands )=======================================

    def __configure_default_commands(self) -> None:
        # Setup the default commands for subsystems
        if wpilib.RobotBase.isSimulation():
            # Set the Drivetrain to arcade drive by default
            self._drivetrain.setDefaultCommand(
                # A split-stick arcade command, with forward/backward controlled by the left
                # hand, and turning controlled by the right.
                RunCommand(
                    lambda: self._drivetrain.drive_teleop(
                        -self._driver_controller.getRawAxis(
                            constants.CONTROLLER_FORWARD_SIM
                        ),
                        -self._driver_controller.getRawAxis(
                            constants.CONTROLLER_TURN_SIM
                        ),
                    ),
                    self._drivetrain,
                ).withName("DefaultDrive")
            )
        else:
            self._drivetrain.setDefaultCommand(
                # A split-stick arcade command, with forward/backward controlled by the left
                # hand, and turning controlled by the right.
                RunCommand(
                    lambda: self._drivetrain.drive_teleop(
                        -self._driver_controller.getLeftY(),
                        -self._driver_controller.getRightX(),
                    ),
                    self._drivetrain,
                ).withName("DefaultDrive")
            )

        # self._intake.setDefaultCommand(
        #     SetIntakeUsingAnalogLeftTrigger(self._intake,self._partner_controller )
        #     )
    
        # self.elevatorSubSys.setDefaultCommand(DriveElevatorManual(self.elevatorSubSys, 0.0))

        # Default command for wrist
        self.wristSubSys.setDefaultCommand(ManualControlWristAngle(self.wristSubSys, self._driver_controller ))


    def __configure_autonomous_commands(self) -> None:
        # Register the named commands used by the PathPlanner auto builder
        # These commands have to match exactly in the PathPlanner application
        # as we name them here in the registration

        NamedCommands.registerCommand(
            "Move_Elevator_L1",
            Move_Elevator_L1(self.elevatorSubSys).withTimeout(5).withName("Elevator L1"),
        )

        NamedCommands.registerCommand(
            "Move_Elevator_L4",
            Move_Elevator_L4(self.elevatorSubSys).withTimeout(5).withName("Elevator L4"),
        )

        NamedCommands.registerCommand(
            "Move_Elevator_CS",
            Move_Elevator_CS(self.elevatorSubSys).withTimeout(5).withName("Elevator CS"),
        )


        NamedCommands.registerCommand(
            "AutonomouseModeComplete", AutonomouseModeComplete(self._drivetrain))

        # To configure the Autonomous routines use PathPlanner to define the auto routines
        # Then, take all of the path planner created routines and add them to the auto
        # chooser so the drive team can select the starting auto.
        self._auto_chooser: wpilib.SendableChooser = wpilib.SendableChooser()
        self._auto_chooser.setDefaultOption("DSquareRT", PathPlannerAuto("SquareRT"))
        self._auto_chooser.addOption("L1-CoralStation", PathPlannerAuto("L1-CoralStation"))
        self._auto_chooser.addOption("L4-Stop", PathPlannerAuto("L4-Stop"))        
        wpilib.SmartDashboard.putData("AutoChooser", self._auto_chooser)

    def __configure_led_triggers(self) -> None:
        pass

    def getAutonomousCommand(self) -> Command:
        return self._auto_chooser.getSelected()

    def teleopInit(self) -> None:
        if self._auto_command is not None:
            self._auto_command.cancel()

    def testInit(self) -> None:
        CommandScheduler.getInstance().cancelAll()

    def autonomousInit(self) -> None:
        # If we're starting on the blue side, offset the Navx angle by 180
        # so 0 degrees points to the right for NWU
        self._drivetrain.set_alliance_offset()
        self._drivetrain.reset_encoders()

        # Set the proper April Tag ID target
        # self._vision.set_target_tag(ShooterPosition.SUBWOOFER_2)
        self._auto_command = self.getAutonomousCommand()

        if self._auto_command is not None:
            self._auto_command.schedule()

    def disabledPeriodic(self) -> None:
        pass

    def autonomousPeriodic(self) -> None:
        pass

    def testPeriodic(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        return super().teleopPeriodic()
