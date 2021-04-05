// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.galsearch;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import frc.robot.Robot.PathColor;
import frc.robot.Robot.PathType;
import frc.robot.utils.GalacticSearch;

public class DetectGalacticSearchCommand extends CommandBase {
  /** Creates a new GalacticSearchCommand. */
  private VisionThread visionThread;
  private int largestContourIndex = 0;
  public DetectGalacticSearchCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
//    visionThread = new VisionThread(Robot.camera, new GalacticSearch(), pipeline -> {
//      for(int i = 0; i < pipeline.filterContoursOutput().size(); i++){
//        Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(i));
//        if(i!= 0 && r.area() > Imgproc.boundingRect(pipeline.filterContoursOutput().get(i-1)).area()){
//          largestContourIndex = i;
//        }
//      }
//      Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(largestContourIndex));
//      Robot.pathColor = r.area() >= AutoConstants.RED_BALL_AREA_THRESHOLD ? PathColor.RED : PathColor.BLUE;
//      Robot.pathType = (r.x + r.width/2)>320 ? PathType.PATHA : PathType.PATHB;
//    });
//    visionThread.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Path Color: " + Robot.pathColor.toString() + "| Path Type: " + Robot.pathType.toString());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
