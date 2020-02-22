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
import frc.robot.subsystems.Drivetrain;

public class DriveWithJoystick extends CommandBase {

  Drivetrain drivetrain;

  /**
   * Creates a new DriveWithJoystick.
   */
  public DriveWithJoystick(Drivetrain p_drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = p_drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.drivetrian.setDeadband(0.08);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftX = Robot.controllerDrive.getX(Hand.kLeft);
    double leftY = Robot.controllerDrive.getY(Hand.kLeft);

    drivetrain.drivetrian.arcadeDrive(leftX * 0.8, leftY * 0.8);
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
