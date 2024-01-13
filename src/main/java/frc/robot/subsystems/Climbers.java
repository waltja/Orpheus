// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climbers extends SubsystemBase {
  CANSparkMax climberPullR;
  CANSparkMax climberPullL;
  /** Creates a new Climbers. */
  public Climbers() {
    climberPullR = new CANSparkMax(Constants.Climbers.CLIMBER_MOTOR_PULL_R, MotorType.kBrushless);
    climberPullL = new CANSparkMax(Constants.Climbers.CLIMBER_MOTOR_PULL_L, MotorType.kBrushless);

    climberPullR.restoreFactoryDefaults();
    climberPullL.restoreFactoryDefaults();

    climberPullL.follow(climberPullR);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climberUp(double speedR){
    climberPullR.set(speedR);
  }

  public void climberDown(double speedR){
    climberPullR.set(speedR);
  }

  public void climbersStop(){
    climberPullR.stopMotor();
    climberPullL.stopMotor();
  }
}
