/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;


public class Turret extends SubsystemBase {

  public WPI_TalonSRX flywheel1 = new WPI_TalonSRX(5);
  public WPI_VictorSPX flywheel2 = new WPI_VictorSPX(6);
  public SpeedControllerGroup flywheel = new SpeedControllerGroup(flywheel1, flywheel2);

  public WPI_TalonSRX turret = new WPI_TalonSRX(7);

  public CANSparkMax hood = new CANSparkMax(8, MotorType.kBrushless);
  public CANPIDController hoodPID = hood.getPIDController();
  
  public NetworkTableEntry hoodPos = Shuffleboard.getTab("Default").add("Hood Position", 0.0).getEntry();
  public NetworkTableEntry hoodP = Shuffleboard.getTab("Default").add("Hood P", 0.2).getEntry();
  public NetworkTableEntry hoodI = Shuffleboard.getTab("Default").add("Hood I", 0.0).getEntry();
  public NetworkTableEntry hoodD = Shuffleboard.getTab("Default").add("Hood D", 0.15).getEntry();

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
