/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableEntry;

public class Drivetrain extends SubsystemBase {
  // Drivetrain Falcon motor controllers
  WPI_TalonFX frontLeft = new WPI_TalonFX(1);
  WPI_TalonFX rearLeft = new WPI_TalonFX(2);
  WPI_TalonFX frontRight = new WPI_TalonFX(3);
  WPI_TalonFX rearRight = new WPI_TalonFX(4);

  SpeedControllerGroup left = new SpeedControllerGroup(frontLeft, rearLeft);
  SpeedControllerGroup right = new SpeedControllerGroup(frontRight, rearRight);

  public DifferentialDrive drivetrain = new DifferentialDrive(left, right);

  public Solenoid driveCooler = new Solenoid(50, 0);

  public NetworkTableEntry current0 = Shuffleboard.getTab("Default").add("Current 0", 0.0).getEntry();
  public NetworkTableEntry current1 = Shuffleboard.getTab("Default").add("Current 1", 0.0).getEntry();
  public NetworkTableEntry current2 = Shuffleboard.getTab("Default").add("Current 2", 0.0).getEntry();
  public NetworkTableEntry current3 = Shuffleboard.getTab("Default").add("Current 3", 0.0).getEntry();

  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
