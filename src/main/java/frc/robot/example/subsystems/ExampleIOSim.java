// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.example.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Map;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.drive.swerve.YAGSLSwerveDrivetrain;
import org.frc5010.lobbinloco.FRC5010BallOnTheFly;
import org.littletonrobotics.junction.Logger;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation.IntakeSide;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.gamepieces.GamePieceProjectile;
import swervelib.simulation.ironmaple.simulation.seasonspecific.crescendo2024.NoteOnFly;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

/** Add your docs here. */
public class ExampleIOSim extends ExampleIOReal {
  protected IntakeSimulation intakeSimulation;
  protected NoteOnFly noteOnFly;
  protected RebuiltFuelOnFly fuelOnFly;
  protected GamePieceProjectile gamePieceProjectile;

  public ExampleIOSim(Map<String, Object> devices, GenericSubsystem parent) {
    super(devices, parent);
    intakeSimulation =
        IntakeSimulation.InTheFrameIntake(
            "FRC5010Ball",
            GenericDrivetrain.getMapleSimDrive().get(),
            Inches.of(24.25),
            IntakeSide.FRONT,
            1);
  }

  @Override
  public void updateSimulation() {
    angularMotor.simulationUpdate();
  }

  @Override
  public void setPercentMotor(double speed) {
    if (speed > 0.0 && !noteIsInsideIntake().getAsBoolean()) {
      intakeSimulation.startIntake();
    } else {
      intakeSimulation.stopIntake();
    }
    super.setPercentMotor(speed);
  }

  public Trigger obtainedGamePieceToScore() {
    return new Trigger(
        () -> {
          return RobotBase.isSimulation()
              ? intakeSimulation.getGamePiecesAmount() == 1
                  && intakeSimulation.obtainGamePieceFromIntake()
              : false;
        });
  }

  public Trigger noteIsInsideIntake() {
    return new Trigger(
        () -> {
          return RobotBase.isSimulation() ? intakeSimulation.getGamePiecesAmount() > 0 : false;
        });
  }

  public Command addBallToRobot() {
    return Commands.runOnce(() -> intakeSimulation.addGamePieceToIntake());
  }

  @Override
  public Command launchBall() {
    return super.launchBall()
        .alongWith(
            Commands.runOnce(
                () -> {
                  if (RobotBase.isSimulation()) {
                    Pose2d worldPose = YAGSLSwerveDrivetrain.getSwerveDrive().getPose();
                    gamePieceProjectile =
                        new FRC5010BallOnTheFly(
                                worldPose.getTranslation(),
                                controlledMotor
                                    .getRobotToMotor()
                                    .getTranslation()
                                    .toTranslation2d(),
                                YAGSLSwerveDrivetrain.getSwerveDrive().getFieldVelocity(),
                                worldPose.getRotation(),
                                Meters.of(0.45),
                                MetersPerSecond.of(10),
                                Degrees.of(55))
                            .withProjectileTrajectoryDisplayCallBack(
                                (pose3ds) -> {
                                  Logger.recordOutput(
                                      parent.getName() + "/GPTrajectory",
                                      pose3ds.toArray(Pose3d[]::new));
                                });
                    SimulatedArena.getInstance().addGamePieceProjectile(gamePieceProjectile);
                  }
                }));
  }
}
