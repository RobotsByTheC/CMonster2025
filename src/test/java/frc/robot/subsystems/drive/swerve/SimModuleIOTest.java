package frc.robot.subsystems.drive.swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.sim.MAXSwerveModuleSim;
import frc.robot.sim.SimulationContext;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class SimModuleIOTest {
  MAXSwerveModuleSim sim;
  SimModuleIO io;

  @BeforeAll
  static void wpilibSetup() {
    HAL.initialize(500, 0);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData(); // "enable" the robot so motor controllers will work
  }

  @BeforeEach
  void setup() {
    sim = new MAXSwerveModuleSim();
    io = new SimModuleIO(sim);
  }

  @AfterEach
  void teardown() {
    SimulationContext.getDefault().removeAll();
  }

  @Test
  void convergesOnPositiveVelocity() {
    double targetMps = 4;
    io.setDesiredState(new SwerveModuleState(targetMps, new Rotation2d()));
    for (int i = 0; i < 50; i++) {
      SimulationContext.getDefault().update(0.02);
    }
    assertEquals(
        targetMps,
        io.getWheelVelocity().in(MetersPerSecond),
        0.075,
        "Wheel velocity did not converge within 1 second");
  }

  @Test
  void convergesOnNegativeVelocity() {
    double targetMps = -4;
    io.setDesiredState(new SwerveModuleState(targetMps, new Rotation2d()));
    for (int i = 0; i < 50; i++) {
      SimulationContext.getDefault().update(0.02);
    }
    assertEquals(
        targetMps,
        io.getWheelVelocity().in(MetersPerSecond),
        0.075,
        "Wheel velocity did not converge within 1 second");
  }

  @Test
  void testConvergesOnPositiveAngle() {
    io.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(90)));
    for (int i = 0; i < 50; i++) {
      SimulationContext.getDefault().update(0.02);
    }
    double angle = io.getModuleRotation().getDegrees();
    assertEquals(90, angle, 1e-3, "Module angle did not converge");
  }

  @Test
  void testConvergesOnNegativeAngle() {
    io.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-90)));
    for (int i = 0; i < 50; i++) {
      SimulationContext.getDefault().update(0.02);
    }
    double angle = io.getModuleRotation().getDegrees();
    assertEquals(-90, angle, 1e-3, "Module angle did not converge");
  }
}
