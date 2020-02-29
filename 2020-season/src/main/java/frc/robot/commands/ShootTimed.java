/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Flywheel;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import com.revrobotics.ControlType;

public class ShootTimed extends CommandBase {

  Index index;
  Turret turret;
  Flywheel flywheel;

  public Timer shootTimer = new Timer();
  public double time = 12.0;

  /**
   * Creates a new ShootTimed.
   */
  public ShootTimed(Index p_index, Turret p_turret, Flywheel p_flywheel) {
    // Use addRequirements() here to declare subsystem dependencies.
    index = p_index;
    turret = p_turret;
    flywheel = p_flywheel;
    addRequirements(index, turret, flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    index.loaded = true;

    flywheel.setSetpoint(270);
    flywheel.enable();

    turret.hoodPID.setReference(3.5, ControlType.kPosition);
    Robot.shooting = true;

    shootTimer.reset();
    shootTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.flywheelVel = flywheel.flywheel1.getSelectedSensorVelocity();

    index.run();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheel.disable();
    Robot.shooting = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shootTimer.get() > time;
  }
}
