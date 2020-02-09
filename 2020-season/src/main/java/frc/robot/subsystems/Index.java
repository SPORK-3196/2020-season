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
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.NetworkTableEntry;

public class Index extends SubsystemBase {

  public CANSparkMax firstStage = new CANSparkMax(9, MotorType.kBrushless);
  public CANSparkMax secondStage = new CANSparkMax(10, MotorType.kBrushless);

  public static DigitalInput[] sensor = new DigitalInput[10];
  int counter = 0;
  boolean waiting = false;
  public boolean lastSensor0Value = false;
  boolean lastSensor1Value = false;

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
  }

  public void index() {
    counter++;
    waiting = true;
  }

  public void run() {
    if(waiting) {
      firstStage.set(0.4);
      secondStage.set(0.45);
      if((counter < 5) && (!lastSensor1Value) && getSensorValue(1)) {
        waiting = false;
        firstStage.set(0.0);
        secondStage.set(0.0);
      } else if((counter == 5) && getSensorValue(3)) {
        waiting = false;
        firstStage.set(0.0);
        secondStage.set(0.0);
      }
    } else {
      firstStage.set(0.0);
      secondStage.set(0.0);
    }

    lastSensor0Value = getSensorValue(0);
    lastSensor1Value = getSensorValue(1);
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
