// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  CANSparkMax ShooterMotorLeft;
  CANSparkMax ShooterMotorRight;
  
    public Shooter() {

        ShooterMotorLeft = new CANSparkMax(Constants.Shooter.ShooterMotorLeftID, MotorType.kBrushless);
        ShooterMotorRight = new CANSparkMax(Constants.Shooter.ShooterMotorRightID, MotorType.kBrushless);
        ShooterMotorLeft.restoreFactoryDefaults();
        ShooterMotorRight.restoreFactoryDefaults();
        ShooterMotorLeft.setIdleMode(IdleMode.kCoast);
        ShooterMotorRight.setIdleMode(IdleMode.kCoast);
       ShooterMotorLeft.follow(ShooterMotorRight, true);

    }
    public void setShooterSpeed(Double Speed) {

        ShooterMotorRight.set(Speed);
       

    }
    public void stop() {

        ShooterMotorRight.stopMotor();
        

    }
  @Override
  public void periodic() {
  }
}