/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.Robot;
import com.revrobotics.ControlType;

public class AlignTurret extends CommandBase {

  Turret turret;
  int turretTarget;
  double hoodTarget;

  /**
   * Creates a new AlignTurret.
   */
  public AlignTurret(Turret p_turret, int p_turretTarget, double p_hoodTarget) {
    // Use addRequirements() here to declare subsystem dependencies.
    turret = p_turret;
    turretTarget = p_turretTarget;
    hoodTarget = p_hoodTarget;
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.setSetpoint(turretTarget);
    turret.enable();

    turret.hoodPID.setReference(hoodTarget, ControlType.kPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double hoodValue = turret.hood.getEncoder().getPosition();
    turret.hoodPos.setDouble(hoodValue);

    turret.turretEncoderDashboard.setDouble(turret.getPWMPosition());
    turret.turret.set(0.5 * turret.lastTurretOutput);
    Robot.turretError = (int)turret.getController().getPositionError();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.disable();
    // TODO: Test turret.turret.set(0.0);
    System.out.println("Turret aligned!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
