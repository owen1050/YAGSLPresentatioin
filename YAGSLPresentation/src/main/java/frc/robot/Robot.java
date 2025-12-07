// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  SwerveDrive swerveDrive;
  PathPlannerPath path;
  PathPlannerTrajectory trajectory;
  Double pathStartTime;
  Field2d pathPlannerGoalPose = new Field2d();

  PIDController pidX = new PIDController(10, 0.25, 0);
  PIDController pidY = new PIDController(10, 0.25, 0);
  PIDController pidR = new PIDController(2, 0, 0);

  public Robot() {
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
    try {
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(3);
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  @Override
  public void robotPeriodic() {
    
  }

  @Override
  public void autonomousInit() {

    try {
      path = PathPlannerPath.fromPathFile("Example Path");
    } catch (Exception e) {
      System.out.println("Error in loading path");
      e.printStackTrace();
    }

    swerveDrive.resetOdometry(path.getStartingHolonomicPose().get());

    try {
      trajectory = path.generateTrajectory(new ChassisSpeeds(), path.getInitialHeading(), RobotConfig.fromGUISettings());
    } catch (Exception e) {
      e.printStackTrace();
    }

    pathStartTime = Timer.getFPGATimestamp();

    SmartDashboard.putData("PathPlannerGoalPose", pathPlannerGoalPose);
  }

  @Override
  public void autonomousPeriodic() {
    PathPlannerTrajectoryState goalState = trajectory.sample(Timer.getFPGATimestamp() - pathStartTime);
    Translation2d goalTranslation2d = new Translation2d(goalState.fieldSpeeds.vxMetersPerSecond, goalState.fieldSpeeds.vyMetersPerSecond);
    
    double xError = pidX.calculate(swerveDrive.getPose().getX(), goalState.pose.getX());
    double yError = pidY.calculate(swerveDrive.getPose().getY(), goalState.pose.getY());
    double rError = pidR.calculate(swerveDrive.getPose().getRotation().minus(goalState.pose.getRotation()).getRadians(), 0);
    
    Translation2d errorTranslation2d = new Translation2d(xError, yError);

    goalTranslation2d = goalTranslation2d.plus(errorTranslation2d);

    pathPlannerGoalPose.setRobotPose(goalState.pose); 
    swerveDrive.drive(goalTranslation2d, goalState.fieldSpeeds.omegaRadiansPerSecond + rError, true, false);
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
