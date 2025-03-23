// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Feet;
import static frc.robot.Constants.CoralLevel.INTAKE;
import static frc.robot.Constants.CoralLevel.L1;
import static frc.robot.Constants.CoralLevel.L2;
import static frc.robot.Constants.CoralLevel.L3;
import static frc.robot.Constants.CoralLevel.L4;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.epilogue.logging.FileBackend;
import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.CoralLevel;
import frc.robot.sim.SimulationContext;
import frc.robot.subsystems.algae.Algae;
import frc.robot.subsystems.algae.RealAlgaeIO;
import frc.robot.subsystems.algae.SimAlgaeIO;
import frc.robot.subsystems.coral.Coral;
import frc.robot.subsystems.coral.RealCoralIO;
import frc.robot.subsystems.coral.SimCoralIO;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.MAXSwerveIO;
import frc.robot.subsystems.drive.SimSwerveIO;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.RealElevatorIO;
import frc.robot.subsystems.elevator.SimElevatorIO;
import java.util.function.Supplier;

@Logged
public class Robot extends TimedRobot {
  private Command autonomousCommand;
  @Logged private final CommandScheduler scheduler = CommandScheduler.getInstance();

  // The robot's subsystems
  private final DriveSubsystem drive;
  private final Elevator elevator;
  private final Coral coral;
  private final Algae algae;
  private final Vision vision;

  private int turnMultiplier = 1;

  // Driver and operator controls
  @NotLogged private final CommandXboxController operatorController; // NOPMD
  @NotLogged private final CommandJoystick rStick; // NOPMD
  @NotLogged private final CommandJoystick lStick; // NOPMD

  @SuppressWarnings("FieldCanBeLocal")
  private final SendableChooser<Supplier<Command>> odometryTestChooser = new SendableChooser<>();

  public Robot() {
    // Initialize our subsystems. If our program is running in simulation mode (either from the
    // simulate command in vscode or from running in unit tests), then we use the simulation IO
    // layers. Otherwise, the IO layers that interact with real hardware are used.

    if (Robot.isSimulation()) {
      drive = new DriveSubsystem(new SimSwerveIO());
      elevator = new Elevator(new SimElevatorIO());
      coral = new Coral(new SimCoralIO());
      algae = new Algae(new SimAlgaeIO());

    } else {
      // Running on real hardware
      drive = new DriveSubsystem(new MAXSwerveIO());
      elevator = new Elevator(new RealElevatorIO());
      coral = new Coral(new RealCoralIO());
      algae = new Algae(new RealAlgaeIO());
    }
    vision = new Vision();

    odometryTestChooser.setDefaultOption("Do Nothing", Commands::none);
    odometryTestChooser.addOption("Drive 1 Foot", () -> drive.driveDistance(Feet.of(1)));
    odometryTestChooser.addOption("Drive 2 Feet", () -> drive.driveDistance(Feet.of(2)));
    odometryTestChooser.addOption("Drive 3 Feet", () -> drive.driveDistance(Feet.of(3)));
    odometryTestChooser.addOption("Drive 5 Feet", () -> drive.driveDistance(Feet.of(5)));
    odometryTestChooser.addOption("Drive 8 Feet", () -> drive.driveDistance(Feet.of(8)));
    odometryTestChooser.addOption("Drive 13 Feet", () -> drive.driveDistance(Feet.of(13)));
    odometryTestChooser.addOption("Drive 21 Feet", () -> drive.driveDistance(Feet.of(21)));

    Shuffleboard.getTab("Test").add("Drive Distance Selection", odometryTestChooser);

    operatorController = new CommandXboxController(Constants.OIConstants.driverControllerPort);
    rStick = new CommandJoystick(Constants.OIConstants.leftJoystickPort);
    lStick = new CommandJoystick(Constants.OIConstants.rightJoystickPort);

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
    drive.setDefaultCommand(driveFastWithFlightSticks());

    //    rStick.trigger().whileTrue(driveSlowWithFlightSticks()); // Disabled because William is a
    // shit driver

    elevator.setDefaultCommand(elevator.stop());
    coral.setDefaultCommand(coral.stow());
    algae.setDefaultCommand(algae.stow());
    // Start data logging

    SignalLogger.start();

    DriverStation.startDataLog(DataLogManager.getLog(), true);

    Epilogue.configure(
        config ->
            config.backend =
                EpilogueBackend.multi(
                    new FileBackend(DataLogManager.getLog()),
                    new NTEpilogueBackend(NetworkTableInstance.getDefault())));

    // Disable joystick warnings by default
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  @SuppressWarnings("unused")
  private Command driveWithXbox() {
    //noinspection SuspiciousNameCombination
    return drive.driveFastWithJoysticks(
        operatorController::getLeftY, operatorController::getLeftX, operatorController::getRightX);
  }

  private Command driveFastWithFlightSticks() {
    //noinspection SuspiciousNameCombination
    return drive.driveFastWithJoysticks(
        lStick::getY, lStick::getX, () -> rStick.getTwist() * turnMultiplier);
  }

  @SuppressWarnings("unused")
  private Command driveSlowWithFlightSticks() {
    //noinspection SuspiciousNameCombination
    return drive.driveSlowWithJoysticks(lStick::getY, lStick::getX, rStick::getTwist);
  }

  private void configureButtonBindings() {
    /*
    BA: Score L1
    BB: Score L2
    BX: Score L3
    BY: Score L4
    LB: Elevator Home
    RB: Coral Intake
    LT:
    RT: [BROKEN]
    ST:
    UA: Algae Intake Ground
    RA: Algae Intake L3
    DA: Algae Score
    LA: Algae Intake L2
    */

    configureTeleopBindings();
    configureTestBindings();
  }

  /**
   * Configures controls that will be used in teleop mode in the main competition. Primarily used
   * for controlling subsystems.
   */
  private void configureTeleopBindings() {
    rStick.button(7).onTrue(drive.zeroGyro());

    bindVision();

    bindElevator();

    bindCoral();

    bindAlgae();
  }

  private void bindVision() {
    rStick
        .button(2)
        .whileTrue(
            drive.defer(
                () -> {
                  turnMultiplier = 0;
                  return drive.rotateToHeading(
                      vision
                          .getLastRealValue()
                          .toPose2d()
                          .getRotation()
                          .rotateBy(Rotation2d.k180deg)
                          .plus(drive.getHeading()));
                }));

    rStick.button(2).onFalse(Commands.runOnce(() -> turnMultiplier = 1));
  }

  private void bindElevator() {
    operatorController.leftBumper().whileTrue(elevator.goToBottom());
  }

  private void bindCoral() {
    operatorController.rightBumper().whileTrue(controlCoralAtLevel(INTAKE));
    operatorController.rightBumper().onFalse(finishControlCoral());

    operatorController.a().whileTrue(controlCoralAtLevel(L1));
    operatorController.a().onFalse(finishControlCoral());

    operatorController.b().whileTrue(controlCoralAtLevel(L2));
    operatorController.b().onFalse(finishControlCoral());

    operatorController.x().whileTrue(controlCoralAtLevel(L3));
    operatorController.x().onFalse(finishControlCoral());

    operatorController.y().whileTrue(controlCoralAtLevel(L4));
    operatorController.y().onFalse(finishControlCoral());
  }

  private Command controlCoralAtLevel(CoralLevel level) {
    return switch (level) {
      case L1 ->
          elevator
              .goToL1Height()
              .deadlineFor(coral.stow())
              .andThen(coral.scoreL1().alongWith(elevator.holdTargetPosition()))
              .withName("Score L1");
      case L2 ->
          elevator
              .goToL2Height()
              .deadlineFor(coral.stow())
              .andThen(coral.scoreL2().alongWith(elevator.holdTargetPosition()))
              .withName("Score L2");
      case L3 ->
          elevator
              .goToL3Height()
              .deadlineFor(coral.stow())
              .andThen(coral.scoreL3().alongWith(elevator.holdTargetPosition()))
              .withName("Score L3");
      case L4 ->
          elevator
              .goToL4Height()
              .deadlineFor(coral.stowAndHold())
              .andThen(coral.scoreL4().alongWith(elevator.holdTargetPosition()))
              .withName("Score L4");
      case INTAKE ->
          elevator
              .home()
              .deadlineFor(coral.stow())
              .andThen(coral.intake())
              .withName("Coral Intake");
    };
  }

  private Command finishControlCoral() {
    return elevator.goToBottom().alongWith(coral.stowUntilDone()).withName("Return to bottom");
  }

  private void bindAlgae() {

    // Intake Ground
    operatorController
        .povUp()
        .whileTrue(
            elevator
                .goToAlgaeIntakeHeight()
                .andThen(algae.intakeGround())
                .withName("Algae Intake Ground"));

    operatorController
        .povUp()
        .onFalse(
            algae
                .stowUntilDone()
                .deadlineFor(elevator.goToBottom())
                .withName("Finish Algae Ground Intake"));

    // Score Processor
    operatorController
        .povDown()
        .whileTrue(
            elevator
                .goToAlgaeScoreHeight()
                .andThen(algae.scoreProcessor())
                .withName("Score Algae"));

    operatorController
        .povDown()
        .onFalse(
            algae.stowUntilDone().andThen(elevator.goToBottom()).withName("Finish Algae Score"));

    // Algae L2
    operatorController
        .povLeft()
        .debounce(0.2)
        .whileTrue(
            elevator
                .goToAlgaeL2Height()
                .andThen(algae.intakeReef().deadlineFor(elevator.goToAlgaeL2Height()))
                .withName("Intake Algae L2"));

    operatorController
        .povLeft()
        .debounce(0.2)
        .onFalse(
            algae
                .stowUntilDone()
                .andThen(elevator.goToBottom())
                .withName("Finish Algae Intake L2"));

    // Algae L3
    operatorController
        .povRight()
        .debounce(0.2)
        .whileTrue(
            elevator
                .goToAlgaeL3Height()
                .andThen(algae.intakeReef().deadlineFor(elevator.holdTargetPosition()))
                .withName("Intake Algae L3"));

    operatorController
        .povRight()
        .debounce(0.2)
        .onFalse(
            algae
                .stowUntilDone()
                .andThen(elevator.goToBottom())
                .withName("Finish Algae Intake L3"));
  }

  /**
   * Configures additional controls that are only active in test mode. Primarily used for sysid
   * routines.
   */
  @SuppressWarnings("unused")
  private void configureTestBindings() {
    //    operatorController
    //        .leftTrigger()
    //        .and(RobotModeTriggers.test())
    //        .whileTrue(elevator.findFeedforwardTerms().withName("Run Elevator Sysid Routine"));
    //    operatorController
    //        .back()
    //        .and(RobotModeTriggers.test())
    //        .whileTrue(drive.defer(() -> odometryTestChooser.getSelected().get()));
    //        operatorController
    //            .rightBumper()
    //            .and(RobotModeTriggers.test())
    //            .whileTrue(coral.runSysIdRoutine().withName("Run Coral Sysid Routine"));
    //    operatorController
    //        .leftBumper()
    //        .and(RobotModeTriggers.test())
    //        .whileTrue(algae.runSysIdRoutine().withName("Run Algae Sysid Routine"));
  }

  private void configureAutomaticBindings() {
    //    elevator.isStalling.whileTrue(elevator.stop());
  }

  /**
   * Gets the command to run in autonomous based on user selection in a dashboard.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //    return drive
    //        .moveBackwardsUntilStopped()
    //        .andThen(
    //            elevator
    //                .goToL4Height()
    //                .andThen(
    //                    coral
    //                        .scoreL4()
    //                        .deadlineFor(elevator.holdCurrentPosition())
    //                        .withName("Score L4")));
    return drive.autoLeaveArea();
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
    if (DriverStation.isFMSAttached()) {
      // Enable joystick warnings on the field
      DriverStation.silenceJoystickConnectionWarning(false);
    }

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    vision.update();

    CommandScheduler.getInstance().run();

    Epilogue.update(this);
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

  /** This autonomous runs the autonomous command selected by the autoChooser. */
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
