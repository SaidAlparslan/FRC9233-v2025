// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.LimelightHelpers; 

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;


  private CvSource outputStream;
  private MjpegServer mjpegServer;
  private final RobotContainer m_robotContainer;

  private final boolean kUseLimelight = true;
  Field2d fieldSim = new Field2d();

  public Robot() {
    m_robotContainer = new RobotContainer();
    
    outputStream = CameraServer.putVideo("Limelight", 320, 240);
    mjpegServer = new MjpegServer("LimelightServer", 5800);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    //updateLimelightPose();

    /*SmartDashboard.putData("Field", fieldSim);

    SmartDashboard.putNumber("Limelight X", LimelightHelpers.getTX("limelight"));
    SmartDashboard.putNumber("Limelight Y", LimelightHelpers.getTY("limelight"));
        
    SmartDashboard.putNumber("Limelight Distance", LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight").pose.getX());


    Mat image = Imgcodecs.imread("http://limelight.local:5800/stream.mjpg"); // Limelight’ın IP’sinden görüntüyü al
        outputStream.putFrame(image);*/

        //Pose2d limelightPose = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight").pose;
        //fieldSim.setRobotPose(limelightPose);*/

        



    /* 
     * This example of adding Limelight is very simple and may not be sufficient for on-field use.
     * Users typically need to provide a standard deviation that scales with the distance to target
     * and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible, though exact implementation
     * of how to use vision should be tuned per-robot and to the team's specification.
     */
    if (kUseLimelight) {
      var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      if (llMeasurement != null) {
        m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, Utils.fpgaToCurrentTime(llMeasurement.timestampSeconds));
      }
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationInit(){
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  @Override
  public void simulationPeriodic() {}
  /*public void updateLimelightPose() {
    NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    double[] botPose = limelightTable.getEntry("botpose").getDoubleArray(new double[6]);

    if (botPose.length == 6) {
        Pose2d robotPose = new Pose2d(botPose[0], botPose[1], new Rotation2d(botPose[5]));
        fieldSim.setRobotPose(robotPose);
    }
}*/
}
