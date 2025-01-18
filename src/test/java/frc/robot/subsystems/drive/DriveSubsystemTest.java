package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.sim.SimulationContext;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class DriveSubsystemTest {
  // NOTE: the drive method relies on real wall time for rate limiting to work, which means we
  //       either have to run tests in real time (which is slow) or avoid using rate limiting
  //       in our tests
  DriveSubsystem drivetrain;

  @BeforeAll
  static void wpilibSetup() {
    HAL.initialize(500, 0);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData(); // "enable" the robot so motor controllers will work
  }

  @BeforeEach
  void setup() {
    drivetrain = new DriveSubsystem(new SimSwerveIO());
  }

  @AfterEach
  void teardown() {
    drivetrain.close();
    SimulationContext.getDefault().removeAll();
  }

  @Test
  void startingConfig() {
    assertEquals(0, drivetrain.getHeading().getDegrees());
    assertEquals(0, drivetrain.getPose().getRotation().getDegrees());
    assertEquals(0, drivetrain.getPose().getX());
    assertEquals(0, drivetrain.getPose().getY());
  }

  @Test
  void headingIncreasesWithPositiveRotation() {
    // simulate 400ms of time
    for (int i = 0; i < 20; i++) {
      drivetrain.drive(
          MetersPerSecond.zero(),
          MetersPerSecond.zero(),
          RotationsPerSecond.of(1),
          DriveSubsystem.ReferenceFrame.ROBOT);
      tick();
    }
    var heading = drivetrain.getHeading();
    assertEquals(90, heading.getDegrees(), 10);

    // odometry should update to match the heading (only relevant for sim - odometry won't
    // be 100% accurate in real life)
    assertEquals(heading, drivetrain.getPose().getRotation());
  }

  @Test
  void headingDecreasesWithNegativeRotation() {
    // simulate 400ms of time
    for (int i = 0; i < 20; i++) {
      drivetrain.drive(
          MetersPerSecond.zero(),
          MetersPerSecond.zero(),
          RotationsPerSecond.of(-1),
          DriveSubsystem.ReferenceFrame.ROBOT);
      tick();
    }
    var heading = drivetrain.getHeading();
    assertEquals(-90, heading.getDegrees(), 10);

    // odometry should update to match the heading (only relevant for sim - odometry won't
    // be 100% accurate in real life)
    assertEquals(heading, drivetrain.getPose().getRotation());
  }

  @Test
  void setX() {
    drivetrain.setX();

    // simulate 400ms of time
    for (int i = 0; i < 20; i++) {
      tick();
    }

    // ... robot shouldn't move ...
    var pose = drivetrain.getPose();
    assertEquals(0, pose.getRotation().getDegrees());
    assertEquals(0, pose.getX());
    assertEquals(0, pose.getY());

    // ... module speeds should all be 0 ...
    assertEquals(0, drivetrain.getModuleStates()[0].speedMetersPerSecond, 1e-6);
    assertEquals(0, drivetrain.getModuleStates()[1].speedMetersPerSecond, 1e-6);
    assertEquals(0, drivetrain.getModuleStates()[2].speedMetersPerSecond, 1e-6);
    assertEquals(0, drivetrain.getModuleStates()[3].speedMetersPerSecond, 1e-6);

    // ... and be rotated 45° inward towards the center of the robot, within ±1°
    assertEquals(45, drivetrain.getModuleStates()[0].angle.getDegrees(), 1);
    assertEquals(-45, drivetrain.getModuleStates()[1].angle.getDegrees(), 1);
    assertEquals(-45, drivetrain.getModuleStates()[2].angle.getDegrees(), 1);
    assertEquals(45, drivetrain.getModuleStates()[3].angle.getDegrees(), 1);
  }

  private void tick() {
    SimulationContext.getDefault().update(0.020);
    drivetrain.periodic();
  }
}
