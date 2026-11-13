// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SimElevator;
import frc.robot.subsystems.SimHand;
import frc.robot.subsystems.SimIntake;

public class RobotContainer {

  private final TankSubsystem tank = new TankSubsystem();

  private final CommandXboxController driver = new CommandXboxController(0);
  private final XboxController driverget = new XboxController(0);

  private SimIntake mSimIntake = new SimIntake(20, 1.5, 3);
  private SimElevator mSimElevator = new SimElevator(400, 1.5);
  private SimHand mSimHand = new SimHand(-180, 180, 1.5, 3);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {

    autoChooser.setDefaultOption("Nenhum", Commands.none());
    autoChooser.addOption("Quadrado", Autos.quadrado(tank));
    autoChooser.addOption("Reto", Autos.reto(tank));
    autoChooser.addOption("autoZigZag", Autos.autoZigZag(tank));
    // autoChooser.addOption("Algae_3", Autos.Algae_3(tank));
    autoChooser.addOption("3 Algas", autoTresAlgas());

    SmartDashboard.putData("Auto Selector", autoChooser);

    configureBindings();

    tank.setDefaultCommand(tank.run(() -> {

      double leftX = driver.getLeftX();
      double leftY = driver.getLeftY();

      double joystickMagnitude = Math.hypot(leftX, leftY);

      // Joystick esquerdo sendo utilizado
      if (joystickMagnitude > 0.08) {

        tank.driveWithAngleControl(getLeftJoystickAngle(), driver.getRawAxis(3));

        // Tank normal
      } else {

        double forward = driver.getRawAxis(3);
        double backward = driver.getRawAxis(2);
        double rotation = driver.getRightX();

        tank.drive(forward, -backward, -rotation);
      }
    }));
  }

  private void configureBindings() {

    driver.start().onTrue(tank.runOnce(() -> tank.resetPose(new Pose2d(2.0, 4.0, new Rotation2d()))));

    driver.povUp().onTrue(mSimElevator.CMDsetPosition(400, 10));
    driver.povDown().onTrue(mSimElevator.CMDsetPosition(0, 3));

    driver.povLeft().onTrue(Commands.runOnce(() -> mSimHand.setAngle(-45, 3)));
    driver.povRight().onTrue(Commands.runOnce(() -> mSimHand.setAngle(45, 3)));
    driver.a().onTrue(Commands.runOnce(() -> mSimHand.setAngle(135, 3)));
    driver.b().onTrue(Commands.runOnce(() -> mSimHand.setAngle(-135, 3)));

    driver.back().onTrue(Commands.runOnce(() -> mSimHand.setAngle(0, 3)));

    /*
     * Positivo = Depositar
     * Negativo = Coletar
     */
    coletar_Ground();
    autoHand();
    // deposit_Pose();

    driver.y().onTrue(deposit());

    // Desafio 2 (equipe): botão X leva o robô até X 7,8 / Y 4,0, virado 60°
    driver.x().onTrue(tank.driveToPose(7.8, 4.0, 60));

  }

  void coletar_Ground() {
    driver.leftBumper().whileTrue(
        mSimIntake.CMDintakeOn().onlyIf(() -> !tank.isFuelInsideRobot() && SimElevator.getRealPosition() >= 200));
    driver.leftBumper().whileFalse(mSimIntake.CMDintakeOff());
  }

  void autoHand() {

    BooleanSupplier autoHandOk = () -> tank.isFuelInsideRobot() && !tank.isHandCollected();

    onCommandCondition(autoHandOk, Commands.sequence(
        mSimElevator.CMDsetPosition(400, 3),
        Commands.waitUntil(() -> SimElevator.getRealPosition() > 390),
        mSimHand.CMDsetAngle(0, 3),
        Commands.waitUntil(() -> Math.abs(SimHand.getRealAngle()) < 5),
        mSimElevator.CMDsetPosition(0, 3),
        mSimHand.CMDsetVelocity(-1),
        Commands.waitSeconds(1),
        mSimHand.CMDsetVelocity(0)));

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public double getLeftJoystickAngle() {

    double x = -driverget.getLeftX();
    double y = -driverget.getLeftY();

    if (Math.hypot(x, y) < 0.1) {
      return 0.0;
    }

    return Math.toDegrees(Math.atan2(x, y));
  }

  /**
   * Ativa o comando alvo, quando a condição é verdadeira.
   * 
   * @param null
   */
  private void onCommandCondition(BooleanSupplier condition, Command command) {
    new Trigger(condition).onTrue(command);
  }

  /**
   * Matém um comando ativo, enquanto a condição é verdade.
   * 
   * @param null
   */
  private void whileCommandCondition(BooleanSupplier condition, Command command) {
    new Trigger(condition).whileTrue(command);
  }

  Command deposit() {
    return Commands.sequence(
        Commands.runOnce(() -> mSimHand.setVelocity(1)),
        tank.depositFuelCommand(),
        Commands.waitSeconds(1),
        Commands.runOnce(() -> mSimHand.setVelocity(0)));
  }

  // ---------- Código da equipe: autônomo de 3 algas (Desafio 1) ----------

  /** Autônomo: começa em X 7,2 / Y 1,0 virado para as algas. Algas 1 e 2 na NET, alga 3 no PROCESSOR. */
  Command autoTresAlgas() {
    return Commands.sequence(
        tank.CMDresetPose(new Pose2d(7.2, 1.0, Rotation2d.fromDegrees(180))),
        // Alga 1 -> NET
        coletarAlga(1.223, 2.200),
        tank.driveToPose(7.0, 2.0, 90), // de lado para o barge
        arremessar(-135),
        // Alga 2 -> NET (passa por baixo do reef, na ida e na volta)
        tank.driveToPoint(2.4, 2.2),
        coletarAlga(1.223, 4.023),
        tank.driveToPoint(2.4, 2.2),
        tank.driveToPose(7.0, 2.0, 90),
        arremessar(-135),
        // Alga 3 -> PROCESSOR (parede de baixo, X 6,0)
        tank.driveToPoint(2.4, 2.2),
        coletarAlga(1.223, 5.858),
        tank.driveToPoint(2.4, 2.2),
        tank.driveToPose(5.99, 0.7, 180), // de lado para a parede
        arremessar(45));
  }

  /** Chega 1,2 m antes da alga já de frente, avança até ela entrar no coletor e espera o braço pegar (autoHand). */
  Command coletarAlga(double x, double y) {
    return Commands.sequence(
        mSimElevator.CMDsetPosition(400, 3),
        mSimIntake.CMDintakeOn(),
        tank.driveToPose(x + 1.2, y, 180),
        tank.driveToPoint(x + 0.55, y),
        Commands.waitUntil(tank::isHandCollected).withTimeout(4),
        mSimIntake.CMDintakeOff());
  }

  /** Sobe o elevador, só então gira o braço (proteção) e arremessa. */
  Command arremessar(double angulo) {
    return Commands.sequence(
        mSimElevator.CMDsetPosition(400, 3),
        Commands.waitUntil(() -> SimElevator.getRealPosition() > 390),
        mSimHand.CMDsetAngle(angulo, 3),
        Commands.waitUntil(() -> Math.abs(SimHand.getRealAngle() - angulo) < 5),
        deposit());
  }

}
