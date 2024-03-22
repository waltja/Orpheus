// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LeftClimber extends SubsystemBase {
  /** Creates a new LeftClimber. */
   TalonFX climberPullL;
   TalonFXConfiguration climberConfig;
  public LeftClimber() {
     climberPullL = new TalonFX(Constants.Climbers.CLIMBER_MOTOR_PULL_L);
     climberPullL.setSafetyEnabled(true);
     climberPullL.setNeutralMode(NeutralModeValue.Brake);
     climberConfig = new TalonFXConfiguration();
     climberConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
     climberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
     climberPullL.getConfigurator().apply(climberConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climberMove(double speedR){
    climberPullL.set(speedR);
  }

  public void climbersStop(){
    climberPullL.stopMotor();
  }
}
