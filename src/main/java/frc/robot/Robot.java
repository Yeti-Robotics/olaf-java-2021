// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.replay.InitiateRecordingCommand;
import frc.robot.commands.replay.PlayRecordingCommand;
import frc.robot.commands.replay.RobotInput;
import frc.robot.commands.replay.TerminateAndSaveRecordingCommand;
import frc.robot.utils.GalacticSearch;
import frc.robot.utils.Limelight;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public static List<RobotInput> inputSequence = new ArrayList<RobotInput>();
	public static List<RobotInput> recentInputSequence = new ArrayList<RobotInput>();
  // private double maxRPM = 0.0;
  // private double maxEncoder = 0.0;

  private RobotContainer m_robotContainer;

  //gal search viz
  private VisionThread visionThread;
  public enum PathState {
    RED, BLUE, NONE
  }
  public static PathState pathSetUp = PathState.BLUE;
  // private final Object imgLock = new Object();
  

  public static void resetPath(){
    pathSetUp = PathState.NONE;
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();   

    //gal search viz
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setResolution(640, 480);
    visionThread = new VisionThread(camera, new GalacticSearch(), pipeline -> {
      while (true) {
        Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
        // System.out.println("area: " + r.area());
        if(!pipeline.filterContoursOutput().isEmpty()){
          // for (int i=0; i< pipeline.filterContoursOutput().size(); i++){
          //   synchronized (imgLock){
          //     if(r.area() >= AutoConstants.RED_BALL_AREA_THRESHOLD){
                pathSetUp = PathState.RED;
          //     }
          //   }
          // }
        } else {
          pathSetUp = PathState.BLUE;
        }
      }
    });
    visionThread.start();

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // if(m_robotContainer.shooterSubsystem.getFlywheelRPM() > maxRPM){
    //   maxRPM = m_robotContainer.shooterSubsystem.getFlywheelRPM();
    // }
    // if(m_robotContainer.shooterSubsystem.getAverageEncoder() > maxEncoder){
    //   maxEncoder = m_robotContainer.shooterSubsystem.getAverageEncoder();
    // }

    // System.out.println("Flywheel RPM, Max, Setpoint: " + m_robotContainer.shooterSubsystem.getFlywheelRPM() + " - " + m_robotContainer.shooterSubsystem.setPoint);
    // System.out.println("Flywheel Enc, Max, Setpoint: " + m_robotContainer.shooterSubsystem.getAverageEncoder() + " - " + m_robotContainer.shooterSubsystem.getVelocityUnitsFromRPM(m_robotContainer.shooterSubsystem.setPoint) + "\n.");
    // System.out.println("Distance: " + Limelight.getHorDistance() + "; Angle: " + m_robotContainer.hoodSubsystem.hoodAngleFromEncoder(m_robotContainer.hoodSubsystem.getEncoder()) + "; RPM: " + m_robotContainer.shooterSubsystem.calcFlywheelRPM() + "; calc hood angle: " + m_robotContainer.hoodSubsystem.calcHoodAngle(Limelight.getHorDistance()));
    //  System.out.println(Limelight.getHorDistance() + ", " + m_robotContainer.hoodSubsystem.hoodAngleFromEncoder(m_robotContainer.hoodSubsystem.getEncoder()) + ", " + m_robotContainer.shooterSubsystem.getFlywheelRPM());

    // System.out.println("gyro:" + m_robotContainer.drivetrainSubsystem.getAngle());
//good line
    // System.out.println("drive mode: "+m_robotContainer.drivetrainSubsystem.getDriveMode()+ "; ShiftStatus: " + m_robotContainer.shiftingGearSubsystem.shiftStatus + "; AvgEnc: " + m_robotContainer.drivetrainSubsystem.getAverageEncoder());

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    // System.out.println("Distance: " + m_robotContainer.drivetrainSubsystem.getLeftEncoder() + "; Encoder: " + m_robotContainer.drivetrainSubsystem.getRawEncoder() + "; gear status: " + m_robotContainer.shiftingGearSubsystem.shiftStatus);
    // PathState pathSetUp;
    // synchronized (imgLock){
    //   pathSetUp = this.pathSetUp;
    // }

    System.out.println("PathSetUp: " + pathSetUp.toString());

    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    // maxRPM = 0.0;
    // maxEncoder = 0.0;
    if(m_robotContainer.turretSubsystem.getReverseLimit()){
      m_robotContainer.turretSubsystem.resetEncoder();
    }
    if (m_robotContainer.hoodSubsystem.getBeamBreak()){
      m_robotContainer.hoodSubsystem.resetEncoder();
    }

    // System.out.println("whore distance: " + Limelight.getHorDistance() + " & estimated angle to shoot from: " + m_robotContainer.hoodSubsystem.calcHoodAngle());

    // System.out.println("Hood encoder: " + m_robotContainer.hoodSubsystem.getEncoder());
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      
      m_autonomousCommand.schedule();
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
    m_robotContainer.drivetrainSubsystem.resetGyro();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (RobotInput.getRecordingState()) {
      RobotInput currentInput = new RobotInput();
      switch (m_robotContainer.drivetrainSubsystem.getDriveMode()) {
        case TANK:
          currentInput.setJoystickYAxis(RobotInput.Joystick.LEFT, m_robotContainer.getLeftY());
          currentInput.setJoystickYAxis(RobotInput.Joystick.RIGHT, m_robotContainer.getRightY());
          break;
        case CHEEZY:
        case ARCADE:
          currentInput.setJoystickYAxis(RobotInput.Joystick.LEFT, m_robotContainer.getLeftY());
          currentInput.setJoystickXAxis(RobotInput.Joystick.RIGHT, m_robotContainer.getRightX());
          break;
      }
      currentInput.setJoystickYAxis(RobotInput.Joystick.LEFT, m_robotContainer.getLeftY());
      currentInput.setJoystickXAxis(RobotInput.Joystick.RIGHT, m_robotContainer.getRightX());
      for (int i = 1; i <= m_robotContainer.getButtonMap().size(); i++){
        if (m_robotContainer.driverStationJoystick.getRawButton(i)){
          if (m_robotContainer.getButtonMap().get(i).getClass() != PlayRecordingCommand.class &&
                  m_robotContainer.getButtonMap().get(i).getClass() != TerminateAndSaveRecordingCommand.class &&
                  m_robotContainer.getButtonMap().get(i).getClass() != InitiateRecordingCommand.class){
            currentInput.setButtonState(i, true);
          }
        }
      }
      inputSequence.add(currentInput);
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    CommandScheduler.getInstance().run();
  }
}
