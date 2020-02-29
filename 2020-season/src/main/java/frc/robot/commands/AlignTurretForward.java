/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Turret;

public class AlignTurretForward extends CommandBase {

  Turret turret;
  int target = 1640;

  /**
   * Creates a new AlignTurretForward.
   */
  public AlignTurretForward(Turret p_turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    turret = p_turret;
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.setSetpoint(target);
    turret.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.turretEncoderDashboard.setDouble(turret.getPWMPosition());
    turret.turret.set(turret.lastTurretOutput * 0.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.disable();
    System.out.println("Turret aligned\n");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Robot.turretError = (int)turret.getController().getPositionError();
    return Math.abs( Robot.turretError ) < 20;
  }
}
