// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climbers extends SubsystemBase {
  TalonFX climberPullR;
  TalonFX climberPullL;
  /** Creates a new Climbers. */
  public Climbers() {
    climberPullR = new TalonFX(Constants.Climbers.CLIMBER_MOTOR_PULL_R);
    climberPullL = new TalonFX(Constants.Climbers.CLIMBER_MOTOR_PULL_L);
    climberPullL.setNeutralMode(NeutralModeValue.Brake);
    climberPullR.setNeutralMode(NeutralModeValue.Brake);

    climberPullL.setControl(new Follower(climberPullR.getDeviceID(), false));
    
    climberPullL.setSafetyEnabled(true);
    climberPullR.setSafetyEnabled(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climberMove(double speedR){
    climberPullR.set(speedR);
  }

  public void climbersStop(){
    climberPullR.stopMotor();
    climberPullL.stopMotor();
  }
}
