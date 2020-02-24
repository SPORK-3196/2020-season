/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANPIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Robot;

public class Turret extends PIDSubsystem {

  public WPI_TalonSRX flywheel1 = new WPI_TalonSRX(9);
  public WPI_VictorSPX flywheel2 = new WPI_VictorSPX(10);
  //public SpeedControllerGroup flywheel = new SpeedControllerGroup(flywheel1, flywheel2);

  public WPI_TalonSRX turret = new WPI_TalonSRX(8);

  public CANSparkMax hood = new CANSparkMax(11, MotorType.kBrushless);
  public CANPIDController hoodPID = hood.getPIDController();
  
  public NetworkTableEntry hoodPos = Shuffleboard.getTab("Default").add("Hood Position", 0.0).getEntry();
  public NetworkTableEntry hoodP = Shuffleboard.getTab("Default").add("Hood P", 0.2).getEntry();
  public NetworkTableEntry hoodI = Shuffleboard.getTab("Default").add("Hood I", 0.0).getEntry();
  public NetworkTableEntry hoodD = Shuffleboard.getTab("Default").add("Hood D", 0.15).getEntry();

  public NetworkTableEntry turretEncoderDashboard = Shuffleboard.getTab("Default").add("Turret Position", 0).getEntry();

  /**
   * Creates a new Turret.
   */
  public Turret() {
    super(
        // The PIDController used by the subsystem
        new PIDController(0.005, 0.0, 0.0004));
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    if(output < -0.3) {
      output = -0.3;
    }
    if(output > 0.3) {
      output = 0.3;
    }
    turret.set(-output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return Robot.camX;
  }
}
