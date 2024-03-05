// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  TalonFX ShooterMotorLeft;
  TalonFX ShooterMotorRight;
  
    public Shooter() {

        ShooterMotorLeft = new TalonFX(Constants.Shooter.ShooterMotorLeftID);
        ShooterMotorRight = new TalonFX(Constants.Shooter.ShooterMotorRightID);
        ShooterMotorLeft.setSafetyEnabled(true);
        ShooterMotorRight.setSafetyEnabled(true);
        ShooterMotorLeft.setNeutralMode(NeutralModeValue.Coast);
        ShooterMotorLeft.setNeutralMode(NeutralModeValue.Coast);

    }
    public void setShooterLSpeed(Double Speed) {
      ShooterMotorLeft.set(Speed);
    }
    public void setShooterRSpeed(Double Speed) {
      ShooterMotorRight.set(Speed);
    }
    public void stopL() {
      ShooterMotorLeft.stopMotor();
    }
    public void stopR(){
      ShooterMotorRight.stopMotor();
    }
  @Override
  public void periodic() {
  }
}