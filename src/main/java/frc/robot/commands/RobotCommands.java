package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotState;
import frc.robot.RobotState.ShooterState;
import frc.robot.sensors.Vision;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakeRoller;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Turret;

public class RobotCommands {
  private final IntakePivot m_intakePivot;
  private final IntakeRoller m_intakeRoller;
  private final Turret m_turret;
  private final Feeder m_feeder;
  private final Indexer m_indexer;
  private final Flywheel m_shooter;
  private final Hood m_hood;
  private final RobotState m_state;
  private final DriveCommands m_driveCommands;

  public RobotCommands(
      Drive drive,
      IntakePivot intakePivot,
      IntakeRoller intakeRoller,
      Turret turret,
      Feeder feeder,
      Vision vision,
      Indexer indexer,
      Flywheel shooter,
      Hood hood,
      RobotState state,
      Supplier<CommandXboxController> controls) {
    m_intakePivot = intakePivot;
    m_intakeRoller = intakeRoller;
    m_turret = turret;
    m_feeder = feeder;
    m_indexer = indexer;
    m_shooter = shooter;
    m_hood = hood;
    m_state = state;
    m_driveCommands = new DriveCommands(drive, () -> -controls.get().getLeftX(), () -> -controls.get().getLeftY(),
        () -> -controls.get().getRightX(), m_state);
  }

  public Command stowIntake() {
    return Commands.parallel(m_intakePivot.stow(), m_intakeRoller.stow())
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf).withName("stowIntake");
  }

  public Command idle() {
    return Commands
        .parallel(m_shooter.stop(), m_feeder.stop(), stowIntake(), m_state.setShooterStateCommand(ShooterState.kIdle))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf).withName("idle");
  }

  public Command driveCommand() {
    return m_driveCommands.driveCommand().withName("driveCommand");
  }

  public Command zeroTurretHood() {
    return m_hood.setLowerSoftLimitEnabledCommand(false)
        .andThen(m_hood.setVoltage(Volts.of(-3.5)).until(m_hood.getCurrentSpikeTrigger()).withTimeout(Seconds.of(4)))
        .finallyDo(() -> {
          m_hood.setLowerSoftLimitEnabled(true);
          m_hood.setEncoderPosition(Degrees.zero());
        }).withName("zeroTurretHood");
  }

	public Command zeroTurretAngle() {
		return m_turret.zeroEncoderAngleCommand().ignoringDisable(true);
	}

  public Command zeroIntake() {
    return m_intakePivot.setLowerSoftLimitEnabledCommand(false)
        .andThen(m_intakePivot.setVoltage(Volts.of(-3.5)).until(m_intakePivot.getCurrentSpikeTrigger())
            .withTimeout(Seconds.of(4)))
        .finallyDo(() -> {
          m_intakePivot.setLowerSoftLimitEnabled(true);
          m_intakePivot.setEncoderPosition(Degrees.zero());
        }).withName("zeroIntake");
  }

	public Command intake() {
		return Commands.parallel(
			m_intakePivot.intake(),
			m_intakeRoller.intake()
		);
	}

	public Command shoot(Supplier<AngularVelocity> speed) {
		return Commands.parallel(
			m_indexer.runForward(),
			m_feeder.runForward(),
			m_shooter.setSpeedCommand(speed)
		);
	}

	public Command setHoodAngle(Supplier<Angle> angle) {
		return m_hood.setAngleCommand(angle);
	}

	public Command setTurretAngleFieldRelative(Supplier<Angle> angle) {
		return m_turret.setAngleCommand(() -> m_state.getRobotPose().getRotation().getMeasure().minus(angle.get()));
	}

	public Command stowHood() {
		return m_hood.setAngleCommand(Degrees.zero());
	}

	public Command stowIntakeAndHaltTurretMovement() {
    return Commands.parallel(idle(), m_turret.stop(), zeroTurretHood().andThen(stowHood()))
        .withTimeout(Seconds.of(0.5))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming).withName("stowIntakeAndHaltTurretMovement");
  }
}