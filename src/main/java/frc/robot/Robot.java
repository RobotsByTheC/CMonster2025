// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.epilogue.logging.FileBackend;
import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.sim.SimulationContext;
import frc.robot.subsystems.coral.Coral;
import frc.robot.subsystems.coral.RealCoralIO;
import frc.robot.subsystems.coral.SimCoralIO;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.MAXSwerveIO;
import frc.robot.subsystems.drive.SimSwerveIO;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.RealElevatorIO;
import frc.robot.subsystems.elevator.SimElevatorIO;

@Logged
public class Robot extends TimedRobot {
  private Command autonomousCommand;
  @Logged private final CommandScheduler scheduler = CommandScheduler.getInstance();

  // The robot's subsystems
  private final DriveSubsystem drive;
  private final Elevator elevator;
  private final Coral coral;

  // Driver and operator controls
  private final CommandXboxController driverController; // NOPMD
  private final Joystick lStick; // NOPMD
  private final Joystick rStick; // NOPMD

  public Robot() {
    // Initialize our subsystems. If our program is running in simulation mode (either from the
    // simulate command in vscode or from running in unit tests), then we use the simulation IO
    // layers. Otherwise, the IO layers that interact with real hardware are used.

    if (Robot.isSimulation()) {
      drive = new DriveSubsystem(new SimSwerveIO());
      elevator = new Elevator(new SimElevatorIO());
      coral = new Coral(new SimCoralIO());
    } else {
      // Running on real hardware
      drive = new DriveSubsystem(new MAXSwerveIO());
      elevator = new Elevator(new RealElevatorIO());
      coral = new Coral(new RealCoralIO());
    }

    driverController = new CommandXboxController(Constants.OIConstants.driverControllerPort);
    lStick = new Joystick(Constants.OIConstants.leftJoystickPort);
    rStick = new Joystick(Constants.OIConstants.rightJoystickPort);

    // Configure the button bindings and automatic bindings
    configureButtonBindings();
    configureAutomaticBindings();

    // Configure default commands
    /*
     * Use driveWithFlightSticks() to use flight stick driving
     * Use driveWithXbox() to drive solely with the xbox controller
     * Note: Right joystick drives, left joystick turns for both
     * xbox and flight sticks. Refer to Constants.java (OIConstants)
     * for the correct Driver Station inputs.
     */
    drive.setDefaultCommand(driveWithFlightSticks());
    elevator.setDefaultCommand(elevator.stop());
    //TODO add coral stow default command

    // Start data logging

    Epilogue.configure(
        config ->
            config.backend =
                EpilogueBackend.multi(
                    new FileBackend(DataLogManager.getLog()),
                    new NTEpilogueBackend(NetworkTableInstance.getDefault())));

    Epilogue.bind(this);
  }

  private Command driveWithXbox() {
    return drive.driveWithJoysticks(
        driverController::getLeftY, driverController::getLeftX, driverController::getRightX);
  }

  private Command driveWithFlightSticks() {
    return drive.driveWithJoysticks(rStick::getY, rStick::getX, lStick::getTwist);
  }

  private void configureButtonBindings() {
    driverController
        .a()
        .whileTrue(
            elevator
                .goToL1Height()
                .andThen(coral.scoreL1().alongWith(elevator.holdCurrentPosition()))
                .andThen(elevator.home().alongWith(coral.stow()))
                .withName("Score L1"));
    driverController
        .b()
        .whileTrue(
            elevator
                .goToL2Height()
                .andThen(coral.scoreL2().alongWith(elevator.holdCurrentPosition()))
                .andThen(elevator.home().alongWith(coral.stow()))
                .withName("Score L2"));
    driverController
        .x()
        .whileTrue(
            elevator
                .goToL3Height()
                .andThen(coral.scoreL3().alongWith(elevator.holdCurrentPosition()))
                .andThen(elevator.home().alongWith(coral.stow()))
                .withName("Score L3"));
    driverController
        .y()
        .whileTrue(
            elevator
                .goToL4Height()
                .andThen(coral.scoreL4().alongWith(elevator.holdCurrentPosition()))
                .andThen(elevator.home().alongWith(coral.stow()))
                .withName("Score L4"));

    //    driverController.rightBumper().whileTrue(elevator.home());

    driverController
        .leftBumper()
        .and(RobotModeTriggers.test())
        .whileTrue(elevator.runSysIdRoutine());
    driverController.rightBumper().and(RobotModeTriggers.test()).whileTrue(coral.runSysIdRoutine());
  }

  private void configureAutomaticBindings() {
    //    elevator.atMaxHeight.onTrue(elevator.holdCurrentPosition());
    //    elevator.atMinHeight.onTrue(elevator.holdCurrentPosition());
    elevator.isStalling.whileTrue(elevator.stop());
  }

  /**
   * Gets the command to run in autonomous based on user selection in a dashboard.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // TODO
    return Commands.none();
  }

  @Logged(name = "Battery Voltage")
  public double getBatteryVoltage() {
    return RobotController.getBatteryVoltage();
  }

  @Logged(name = "Current Draw")
  public double getBatteryCurrentDraw() {
    return RobotController.getInputCurrent();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  @Override
  public void simulationPeriodic() {
    SimulationContext.getDefault().update(getPeriod());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by the {@link #autoChooser}. */
  @Override
  public void autonomousInit() {
    autonomousCommand = getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
