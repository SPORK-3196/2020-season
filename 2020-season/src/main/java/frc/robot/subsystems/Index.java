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
import frc.robot.Robot;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.NetworkTableEntry;

public class Index extends SubsystemBase {

  public CANSparkMax intake = new CANSparkMax(5, MotorType.kBrushless);
  public CANSparkMax firstStage = new CANSparkMax(6, MotorType.kBrushless);
  public CANSparkMax secondStage = new CANSparkMax(7, MotorType.kBrushless);

  public static Solenoid intakeSolenoid1 = new Solenoid(50, 0);
  public static Solenoid intakeSolenoid2 = new Solenoid(50, 7);

  public boolean intakeOut = false;

  public static DigitalInput[] sensor = new DigitalInput[10];
  int counter = 0;
  boolean waiting = false;
  public boolean lastSensor0Value = false;
  boolean lastSensor1Value = false;
  boolean lastSensor5Value = false;

  public static NetworkTableEntry counter_D = Shuffleboard.getTab("Default").add("Index", 0).getEntry();
  public static NetworkTableEntry waiting_D = Shuffleboard.getTab("Default").add("Waiting", false).getEntry();
  


  public boolean getSensorValue(int sensorNumber) {
    return !sensor[sensorNumber].get();
  }

  public void reset() {
    counter = 0;
    waiting = false;
    lastSensor0Value = false;
    lastSensor1Value = false;
    lastSensor5Value = false;
  }

  public void runMotors() {
    firstStage.set(0.4);
    secondStage.set(0.3);
  }

  public void stopMotors() {
    firstStage.set(0.0);
    secondStage.set(0.0);
  }

  public void runIntake() {
    if(Robot.controllerSecondary.getXButton()) {
      intake.set(-0.5);
    } else if(intakeOut) {
      intake.set(0.8);
    } else {
      intake.set(0.0);
    }
  }

  public void stopIntake() {
    intake.set(0.0);
  }

  public void index() {
    counter++;
    waiting = true;
  }

  public void run() {
    if(waiting) {
      runMotors();
      stopIntake();
      if((counter < 5) && (!lastSensor1Value) && getSensorValue(1)) {
        waiting = false;
        stopMotors();
      } else if((counter == 5) && !getSensorValue(5) && lastSensor5Value) {
        waiting = false;
        stopMotors();
      }
    } else {
      stopMotors();
      runIntake();
    }

    if(Robot.controllerSecondary.getBButton()) {
      if(Robot.flywheelVel > 254) {
        runMotors();
      } else {
        stopMotors();
      }
    }

    lastSensor0Value = getSensorValue(0);
    lastSensor1Value = getSensorValue(1);
    lastSensor5Value = getSensorValue(5);
    counter_D.setNumber(counter);
    waiting_D.setBoolean(waiting);
  }
  
  /**
   * Creates a new Index.
   */
  public Index() {
    for(int i = 0; i < 10; i++){
      if(sensor[i] == null) { 
        sensor[i] = new DigitalInput(i);
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
