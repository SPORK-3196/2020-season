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
  private final Drivetrain drivetrain;
  
  /**
   * Creates a new DriveWithJoystick.
   */
  public DriveWithJoystick(Drivetrain drivetrain_p) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = drivetrain_p;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotation = Robot.controllerDrive.getX(Hand.kLeft);
    double speed = Robot.controllerDrive.getY(Hand.kLeft);

    drivetrain.drivetrain.arcadeDrive(speed*-1.0, rotation*0.7);

    drivetrain.current0.setDouble(Robot.pdp.getCurrent(0));
    drivetrain.current1.setDouble(Robot.pdp.getCurrent(1));
    drivetrain.current2.setDouble(Robot.pdp.getCurrent(2));
    drivetrain.current3.setDouble(Robot.pdp.getCurrent(3));

    boolean cool = Robot.controllerDrive.getAButton();
    drivetrain.driveCooler.set(cool);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drivetrain.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
