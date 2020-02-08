/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Index extends SubsystemBase {

  public CANSparkMax firstStage = new CANSparkMax(9, MotorType.kBrushless);
  public CANSparkMax secondStage = new CANSparkMax(10, MotorType.kBrushless);
  
  /**
   * Creates a new Index.
   */
  public Index() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
