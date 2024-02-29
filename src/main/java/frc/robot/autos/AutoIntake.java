package frc.robot.autos;



import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.Shooter;

public class AutoIntake extends Command {
  
  private GroundIntake intake;
  private Timer timer;
  public AutoIntake(GroundIntake intake) {
    this.intake = intake;
    
    timer = new Timer();

    addRequirements(intake);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
        intake.intake(-.7);
  

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 2.0;
  }
}
