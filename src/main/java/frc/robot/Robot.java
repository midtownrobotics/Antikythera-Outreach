package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import java.net.InetAddress;
import java.net.UnknownHostException;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.LoggedCommandScheduler;
import frc.lib.Watchdawg;
import frc.robot.ShootingParameters.ShootingParametersMode;
import frc.robot.commands.RobotCommands;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.Ports;
import frc.robot.generated.TunerConstants;
import frc.robot.sensors.Vision;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.feeder.FeederIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakePivotIOSim;
import frc.robot.subsystems.intake.IntakePivotIOTalonFX;
import frc.robot.subsystems.intake.IntakeRoller;
import frc.robot.subsystems.intake.IntakeRollerIOSim;
import frc.robot.subsystems.intake.IntakeRollerIOTalonFX;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.FlywheelIOSim;
import frc.robot.subsystems.shooter.FlywheelIOTalonFX;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.HoodIOSim;
import frc.robot.subsystems.shooter.HoodIOTalonFX;
import frc.robot.subsystems.shooter.Turret;
import frc.robot.subsystems.shooter.TurretIOSim;
import frc.robot.subsystems.shooter.TurretIOTalonFX;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private final Drive m_drive;
  private final Vision m_vision;

  private final IntakePivot m_intakePivot;
  private final IntakeRoller m_intakeRoller;

  private final Turret m_turret;
  private final Flywheel m_shooter;
  private final Hood m_hood;

  private final Feeder m_feeder;
  private final Indexer m_indexer;

  private final RobotState m_state;
  private final RobotViz m_viz;

  private final Watchdawg m_watchdog;

  private final RobotCommands m_robotCommands;

  private final LoggedDashboardChooser<Integer> m_cameraPipelineChooser;

	private final CommandXboxController m_masterController;
	private final CommandXboxController m_driveController;
	private final CommandXboxController m_operatorController;

	private boolean masterControl = true;
	private Angle hoodAngle = Degrees.of(0);
	private Angle turretAngle = Degrees.of(0);
	private	AngularVelocity speed = RPM.of(1250);

  public Robot() {

		m_masterController = new CommandXboxController(0);
		m_driveController = new CommandXboxController(1);
		m_operatorController = new CommandXboxController(2);

		RobotController.setBrownoutVoltage(6.5);

    DriverStation.silenceJoystickConnectionWarning(Robot.isSimulation());

    // m_pdh.setSwitchableChannel(true);

    // AdvantageKit Logger setup
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });
    try {
      Logger.recordMetadata(
          "Hostname", InetAddress.getLocalHost().getHostName().replaceAll("\\.local$", ""));
    } catch (UnknownHostException e) {
      Logger.recordMetadata("Hostname", "Unknown");
    }

    // Set up data receivers & replay source
    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter());
      Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
      Logger.addDataReceiver(new NT4Publisher());
    } else {
      // Running a physics simulator, log to NT
      Logger.addDataReceiver(new NT4Publisher());
    }

    Logger.start();

    if (isReal()) {
      m_drive = new Drive(
          new GyroIOPigeon2(),
          new ModuleIOTalonFX(TunerConstants.FrontLeft),
          new ModuleIOTalonFX(TunerConstants.FrontRight),
          new ModuleIOTalonFX(TunerConstants.BackLeft),
          new ModuleIOTalonFX(TunerConstants.BackRight));

      m_intakePivot = new IntakePivot(new IntakePivotIOTalonFX());
      m_intakeRoller = new IntakeRoller(new IntakeRollerIOTalonFX());
      m_feeder = new Feeder(new FeederIOTalonFX());
      m_indexer = new Indexer(new IndexerIOTalonFX());
      m_hood = new Hood(new HoodIOTalonFX());
      m_shooter = new Flywheel(new FlywheelIOTalonFX());
      m_turret = new Turret(new TurretIOTalonFX());
    } else {
      m_drive = new Drive(
          new GyroIOSim(),
          new ModuleIOSim(TunerConstants.FrontLeft),
          new ModuleIOSim(TunerConstants.FrontRight),
          new ModuleIOSim(TunerConstants.BackLeft),
          new ModuleIOSim(TunerConstants.BackRight));

      m_intakePivot = new IntakePivot(new IntakePivotIOSim());
      m_intakeRoller = new IntakeRoller(new IntakeRollerIOSim());
      m_feeder = new Feeder(new FeederIOSim());
      m_indexer = new Indexer(new IndexerIOSim());
      m_hood = new Hood(new HoodIOSim());
      m_shooter = new Flywheel(new FlywheelIOSim());
      m_turret = new Turret(new TurretIOSim());
    }

    m_vision = new Vision(
        (observation) -> m_drive.addVisionMeasurement(
            observation.pose(), observation.timestamp(), observation.standardDevs()),
        m_drive::getPose,
        m_drive::resetPose
			);

    m_state = new RobotState(
        m_drive,
        m_intakePivot,
        m_intakeRoller,
        m_turret,
        m_feeder,
        m_vision,
        m_indexer,
        m_shooter,
        m_hood);

    m_robotCommands = new RobotCommands(
        m_drive,
        m_intakePivot,
        m_intakeRoller,
        m_turret,
        m_feeder,
        m_vision,
        m_indexer,
        m_shooter,
        m_hood,
        m_state,
				() -> (masterControl ? m_masterController : m_driveController));

    m_viz = new RobotViz(m_state);

    m_cameraPipelineChooser = new LoggedDashboardChooser<Integer>("Vision Pipeline");
    m_cameraPipelineChooser.addDefaultOption("Main Field", 0);
    m_cameraPipelineChooser.addOption("Practice Field", 2);
    m_cameraPipelineChooser.addOption("Johnson", 1);

    m_cameraPipelineChooser.onChange(x -> {
      m_vision.setPipelinesToIndex(x);
    });
		
		SmartDashboard.putData("Vision/setToMainFieldPipeline", Commands.runOnce(() -> {m_vision.setPipelinesToIndex(0); }).ignoringDisable(true).withName("setToMainFieldPipeline"));

		m_hood.setDefaultCommand(m_hood.setAngleCommand(() -> hoodAngle));
		m_turret.setDefaultCommand(m_turret.setAngleCommand(() -> turretAngle));

    // m_hood.setDefaultCommand(m_hood.setAngleCommand(Degrees.zero()).withName("hoodDefaultStow"));

    // m_turret.setDefaultCommand(m_robotCommands.turretTrackShootingParameters());

    m_drive.setDefaultCommand(m_robotCommands.driveCommand());

    // m_state.isPreparedToShootTrigger().or(m_state.isFeedingTrigger())
    //     .whileTrue(m_robotCommands.feedFuel());

    m_watchdog = new Watchdawg(getClass());

    configureBindings();

    LoggedCommandScheduler.init(CommandScheduler.getInstance());

    m_state.inAllianceZoneTrigger().and(RobotModeTriggers.disabled().negate())
        .and(RobotModeTriggers.autonomous().negate())
        .whileTrue(m_state.getShootingParameters()
            .setTargetCommand(FieldConstants::getHubPosition2d, ShootingParametersMode.kShoot)
            .withName("setTargetCommandHubPosition"))
        .whileFalse(
            m_state.getShootingParameters().setTargetCommand(m_state::calculateFeedTarget, ShootingParametersMode.kPass)
                .withName("setTargetCommandFeed"));

    RobotModeTriggers.teleop().onTrue(m_robotCommands.stowIntakeAndHaltTurretMovement());

    SmartDashboard.putData("StartSignalLogger", Commands.runOnce(() -> SignalLogger.start()));
    SmartDashboard.putData("StopSignalLogger", Commands.runOnce(() -> SignalLogger.stop()));

		
		SmartDashboard.putData("Commands/ZeroTurretAngle", m_robotCommands.zeroTurretAngle());
  }

	public boolean masterActive() {
		return masterControl;
	}

	public boolean masterInactive() {
		return !masterControl;
	}

	public void setMaster(boolean to) {
		masterControl = to;
	}

  public void configureBindings() {
		Trigger bothBumpers = m_masterController.rightBumper().and(m_masterController.leftBumper());
		bothBumpers.onTrue(Commands.runOnce(() -> setMaster(false)));
		bothBumpers.onFalse(Commands.parallel(Commands.runOnce(() -> setMaster(true)), m_robotCommands.idle()));

    m_masterController.a().onTrue(m_robotCommands.idle());
		m_masterController.b().onTrue(Commands.runOnce(() -> m_drive.resetPose(new Pose2d())));
		m_masterController.povUp().onTrue(Commands.runOnce(() -> speed = speed.plus(RPM.of(250))));
		m_masterController.povDown().onTrue(Commands.runOnce(() -> speed = speed.minus(RPM.of(250))));

		m_masterController.rightTrigger().whileTrue(m_robotCommands.intake()).onFalse(m_robotCommands.idle());
		m_operatorController.rightTrigger().and(this::masterInactive).whileTrue(m_robotCommands.intake()).onFalse(m_robotCommands.idle());

		m_masterController.leftTrigger().onTrue(m_robotCommands.shoot(() -> speed)).onFalse(m_robotCommands.idle());
		m_operatorController.leftTrigger().and(this::masterInactive).onTrue(m_robotCommands.shoot(() -> speed)).onFalse(m_robotCommands.idle());
	}

	public void axisBindingLoop() {
		if (!masterControl && Math.abs(m_operatorController.getLeftY()) > 0.1) {
      hoodAngle = Degrees.of(MathUtil.clamp(hoodAngle.in(Degrees) + 20.0 /** <- deg/sec */ * 0.02 * -m_operatorController.getLeftY(), 0, 40));
    }

		if (!masterControl && (Math.abs(m_operatorController.getRightX()) > 0.1 || Math.abs(m_operatorController.getRightY()) > 0.1)) {
			turretAngle = (Radians.of(Math.atan2(-m_operatorController.getRightY(), m_operatorController.getRightX()))).minus(Degrees.of(90)).minus(m_drive.getPose().getRotation().getMeasure());
		}
	}

  @Override
  public void robotPeriodic() {

    Logger.recordOutput("Vision/isSOTMEnabled", m_state.isShootOnTheMoveEnabled());

    m_watchdog.start();
    CommandScheduler.getInstance().run();
    m_watchdog.end("commandScheduler");

    m_watchdog.start();
    m_viz.periodic();
    m_watchdog.end("robotVizPeriodic");

    m_state.periodic();
		
		Logger.recordOutput("CanBusUsage/Drive", Ports.driveCanBus.getStatus().BusUtilization);
		Logger.recordOutput("CanBusUsage/Mechs", Ports.primaryCanBus.getStatus().BusUtilization);

		Logger.recordOutput("matchTime", DriverStation.getMatchTime());

    LoggedCommandScheduler.periodic();

		axisBindingLoop();
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void driverStationConnected() {
    CommandScheduler.getInstance()
        .schedule(m_state.getShootingParameters().setTargetCommand(FieldConstants.getHubPosition2d()));
  }
}
