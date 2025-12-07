// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  SwerveDrive swerveDrive;
  Joystick driverJoystick = new Joystick(0);
  double maxXYV = 3;
  double maxRv = 3;

  public Robot() {
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        try {
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maxXYV);
        } catch (IOException e) {
            e.printStackTrace();
        }
  }

  @Override
  public void robotPeriodic() {
    swerveDrive.addVisionMeasurement(null, kDefaultPeriod);
  }

  @Override
  public void autonomousInit() {
    swerveDrive.resetOdometry(new Pose2d(1, 1, new Rotation2d()));
  }

  @Override
  public void autonomousPeriodic() {
    swerveDrive.drive(new Translation2d(1, 0),0, true, false);
  }

  @Override
  public void teleopInit() {
    swerveDrive.resetOdometry(new Pose2d(1, 1, new Rotation2d()));
  }

  @Override
  public void teleopPeriodic() {
    double xV = driverJoystick.getRawAxis(1);
    double yV = driverJoystick.getRawAxis(0);
    double rV = -driverJoystick.getRawAxis(4);

    swerveDrive.drive(new Translation2d(maxXYV * xV, maxXYV * yV), maxRv * rV, true, false);

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
