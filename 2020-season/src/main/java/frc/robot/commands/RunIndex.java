/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Index;

public class RunIndex extends CommandBase {

  Index index = new Index();

  /**
   * Creates a new RunIndex.
   */
  public RunIndex(Index index_p) {
    index = index_p;
    addRequirements(index);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double leftX = Robot.controllerSecondary.getX(Hand.kLeft);
    if(leftX > -0.05 && leftX < 0.05) leftX = 0.0;

    double rightX = Robot.controllerSecondary.getX(Hand.kRight);
    if(rightX > -0.05 && rightX < 0.05) rightX = 0.0;

    index.firstStage.set(rightX);
    index.secondStage.set(leftX);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
