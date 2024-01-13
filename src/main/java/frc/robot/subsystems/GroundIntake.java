// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root 
//directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class GroundIntake extends SubsystemBase {
  /** Creates a new GroundIntake. */
  private static CANSparkMax intakeMotor;

  private static CANSparkMax indexingLeft;
  private static CANSparkMax indexingRight;

  private static double tooHotTemperatureHigh = 80.0; // change later?
  private static final double tooHotTemperatureLow = 65.0; // change later?
  private boolean tooHotAlertActive = false;
  private final String tooHotAlert = "Intake motor disabled due to very high temperature.";
  public double motorTemperautre;


  public GroundIntake() {

    intakeMotor = new CANSparkMax( Constants.INTAKE_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);

    indexingLeft = new CANSparkMax(Constants.INDEXING_LEFT_ID, CANSparkLowLevel.MotorType.kBrushless);
    indexingRight = new CANSparkMax(Constants.INDEXING_RIGHT_ID, CANSparkLowLevel.MotorType.kBrushless);


    intakeMotor.restoreFactoryDefaults();
    indexingLeft.restoreFactoryDefaults();
    indexingRight.restoreFactoryDefaults();

    indexingRight.follow(indexingLeft, true);

    intakeMotor.setSmartCurrentLimit(40, 30);
    motorTemperautre = intakeMotor.getMotorTemperature();

  }

  public void intake(double intakeSpeed, double indexingSpeed){

    intakeMotor.set(intakeSpeed);
    indexingLeft.set(indexingSpeed);

  }

  public void stop(){
    intakeMotor.stopMotor();
    indexingLeft.stopMotor();
  
  }

  @Override
  public void periodic() {
    motorTemperautre = intakeMotor.getMotorTemperature();
    if(tooHotAlertActive){
      intakeMotor.setVoltage(0.0);
      System.out.println(tooHotAlert);
    }
    // Update too hot alert
    if(motorTemperautre >= tooHotTemperatureHigh){tooHotAlertActive = true;}
    
    if (tooHotAlertActive) {
      if (motorTemperautre < tooHotTemperatureLow) {
        tooHotAlertActive = false;
      }
    } 
    else if (motorTemperautre < tooHotTemperatureHigh){
       tooHotAlertActive = false;
    }
  }
}
