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
import frc.robot.subsystems.Turret;
import com.revrobotics.ControlType;

public class RunTurret extends CommandBase {

  private final Turret turret;

  /**
   * Creates a new RunTurret.
   */
  public RunTurret(Turret newTurret) {
    // Use addRequirements() here to declare subsystem dependencies.
    turret = newTurret;
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.flywheel2.follow(turret.flywheel);

    turret.hoodPID.setP(0.03);
    turret.hoodPID.setI(0.0);
    turret.hoodPID.setD(0.0);
    turret.hoodPID.setOutputRange(-0.3, 0.3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rightY = Robot.controllerSecondary.getY(Hand.kRight);
    double rightTrigger = Robot.controllerSecondary.getTriggerAxis(Hand.kRight);
    double leftX = Robot.controllerSecondary.getX(Hand.kLeft);
  
    double hoodValue = turret.hood.getEncoder().getPosition();
    turret.hoodPos.setDouble(hoodValue);
    turret.hoodPID.setP(turret.hoodP.getDouble(0.0));
    turret.hoodPID.setI(turret.hoodI.getDouble(0.0));
    turret.hoodPID.setD(turret.hoodD.getDouble(0.0));

    if(Robot.controllerSecondary.getBumper(Hand.kRight)) {
      turret.flywheel.set(0.8);
    } else {
      turret.flywheel.set(0.0);
    }

    turret.flywheelSpeedDashboard.setDouble(turret.flywheel.getSelectedSensorVelocity());



    if(Robot.controllerSecondary.getXButton()) {
      turret.hoodPID.setReference(10.0, ControlType.kPosition);
      //turret.setSetpoint(320.0);
      //turret.enable();
    } else {
      turret.hood.set(-0.3*rightY);
      turret.disable();
      turret.turret.set(0.5*leftX);
    }
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
