/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;


public class Turret extends SubsystemBase {

  public WPI_TalonSRX flywheel = new WPI_TalonSRX(5);
  public WPI_VictorSPX flywheel2 = new WPI_VictorSPX(6);

  public WPI_TalonSRX turret = new WPI_TalonSRX(7);

  public CANSparkMax hood = new CANSparkMax(8, MotorType.kBrushless);

  /**
   * Creates a new Turret.
   */
  public Turret() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
