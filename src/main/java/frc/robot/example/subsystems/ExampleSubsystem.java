package frc.robot.example.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.example.subsystems.ExampleIO.ExampleIOInputs;
import java.util.function.Supplier;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.constants.GenericPID;
import org.frc5010.common.constants.MotorFeedFwdConstants;
import org.frc5010.common.motors.MotorConstants.Motor;
import org.frc5010.common.motors.MotorFactory;
import org.frc5010.common.motors.function.AngularControlMotor;
import org.frc5010.common.sensors.absolute_encoder.RevAbsoluteEncoder;

public class ExampleSubsystem extends GenericSubsystem {
  protected ExampleIO io;
  protected ExampleIOInputs inputs = new ExampleIOInputs();
  protected int scoredNotes = 0;
  protected Rotation2d rotation = new Rotation2d(Degrees.of(180));

  public ExampleSubsystem() {
    super("example.json");
    devices.put("angular_motor", angularControlledMotor());
    if (RobotBase.isSimulation()) {
      io = new ExampleIOSim(devices, this);
    } else {
      io = new ExampleIOReal(devices, this);
    }
  }

  public Command trackTargetCommand(Supplier<Translation2d> targetSupplier) {
    return Commands.run(
        () -> {
          Translation2d targetPose = targetSupplier.get();
          targetPose.getNorm();
          io.setTurretRotation(targetPose.getAngle().getMeasure());
          io.setHoodAngle(Degrees.of(45));
        });
  }

  public Command stopTrackingCommand() {
    return Commands.runOnce(
        () -> {
          io.setTurretRotation(Degrees.of(0));
          io.setHoodAngle(Degrees.of(0));
        });
  }

  public Command intakeCommand() {
    return Commands.runOnce(() -> io.setPercentMotor(0.5), this);
  }

  public Command stopIntakeCommand() {
    return Commands.runOnce(() -> io.setPercentMotor(0.0), this);
  }

  public Command sysIdShooter() {
    return io.sysIdShooter();
  }

  public Command addBallToRobot() {
    return io.addBallToRobot();
  }

  public Command launchBall() {
    return io.launchBall();
  }

  public Command sysIdArm() {
    return io.sysIdArm();
  }

  public Command sysIdPivot() {
    return io.sysIdPivot();
  }

  public Command sysIdTurret() {
    return io.sysIdTurret();
  }

  @Override
  public void periodic() {
    super.periodic();
    io.updateInputs(inputs);
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();
    io.updateSimulation();
  }

  public AngularControlMotor angularControlledMotor() {
    AngularControlMotor angularMotor =
        new AngularControlMotor(
                MotorFactory.Spark(13, Motor.Neo), "angular", getDisplayValuesHelper())
            .setupSimulatedMotor(
                (5.0 * 68.0 / 24.0) * (80.0 / 24.0),
                Units.lbsToKilograms(22),
                Inches.of(19),
                Degrees.of(0),
                Degrees.of(360),
                false,
                0,
                Degrees.of(0),
                false,
                0.1)
            .setVisualizer(mechanismSimulation, new Pose3d(0.75, 0, 0.25, new Rotation3d()));
    angularMotor.setEncoder(new RevAbsoluteEncoder((SparkMax) angularMotor.getMotor(), 360));
    angularMotor.setValues(new GenericPID(0.01, 0.000025, 0.003));
    angularMotor.setMotorFeedFwd(new MotorFeedFwdConstants(0.0, 0.01, 0.0, false));
    angularMotor.setIZone(3);
    angularMotor.setOutputRange(-12, 12);
    return angularMotor;
  }

  // public Command setVelocityControlMotorReference(DoubleSupplier reference) {
  //   return Commands.runOnce(
  //       () -> {
  //         double speed = reference.getAsDouble();
  //         if (speed <= 0.0 && !noteIsInsideIntake().getAsBoolean()) {
  //           controlledMotor.setReference(speed);
  //         } else if (speed > 3000
  //             && noteIsInsideIntake().getAsBoolean()
  //             && obtainedGamePieceToScore().getAsBoolean()) {
  //           controlledMotor.setReference(speed);
  //           if (RobotBase.isSimulation()) {
  //             Pose2d worldPose = YAGSLSwerveDrivetrain.getSwerveDrive().getPose();
  //             gamePieceProjectile =
  //                 new ReefscapeAlgaeOnFly(
  //                     worldPose.getTranslation(),
  //                     controlledMotor.getRobotToMotor().getTranslation().toTranslation2d(),
  //                     YAGSLSwerveDrivetrain.getSwerveDrive().getFieldVelocity(),
  //                     worldPose.getRotation(),
  //                     Meters.of(0.45),
  //                     MetersPerSecond.of(speed / 6000 * 20),
  //                     Degrees.of(55));
  //             SimulatedArena.getInstance().addGamePieceProjectile(gamePieceProjectile);
  //           }
  //         } else if (speed < 3000
  //             && speed > 1000
  //             && noteIsInsideIntake().getAsBoolean()
  //             && obtainedGamePieceToScore().getAsBoolean()) {
  //           controlledMotor.setReference(speed);
  //           if (RobotBase.isSimulation()) {
  //             Pose2d worldPose = YAGSLSwerveDrivetrain.getSwerveDrive().getPose();
  //             gamePieceProjectile =
  //                 new ReefscapeAlgaeOnFly(
  //                     worldPose.getTranslation(),
  //                     controlledMotor.getRobotToMotor().getTranslation().toTranslation2d(),
  //                     YAGSLSwerveDrivetrain.getSwerveDrive().getFieldVelocity(),
  //                     worldPose.getRotation(),
  //                     Meters.of(0.45),
  //                     MetersPerSecond.of(speed / 6000 * 20),
  //                     Degrees.of(55));
  //             SimulatedArena.getInstance().addGamePieceProjectile(gamePieceProjectile);
  //           }
  //         } else {
  //           controlledMotor.setReference(speed);
  //         }
  //       },
  //       this);
  // }

  // public Command setAngularMotorReference(DoubleSupplier reference) {
  //   return Commands.runOnce(() -> angularMotor.setReference(reference.getAsDouble()), this);
  // }

}
