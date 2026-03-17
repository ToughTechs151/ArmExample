// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.sim;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.sim.Constants.ArmSim;

/** A robot arm simulation based on a linear system model with Mech2d display. */
public class ArmModel implements AutoCloseable {

  private final ArmSubsystem armSubsystem;
  private double simArmCurrent = 0.0;
  private SparkMaxSim sparkSimArm;

  // The arm gearbox represents a gearbox containing one motor.
  private final DCMotor armGearbox = DCMotor.getNEO(1);

  // This arm sim represents an arm that can rotate over the given mechanical range when
  // driven
  // by the motor under the effect of gravity.
  private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          armGearbox,
          ArmConstants.GEAR_RATIO,
          SingleJointedArmSim.estimateMOI(ArmSim.ARM_LENGTH_METERS, ArmSim.ARM_MASS_KG),
          ArmSim.ARM_LENGTH_METERS,
          ArmConstants.MIN_ANGLE_RADS,
          ArmConstants.MAX_ANGLE_RADS,
          true,
          ArmSim.START_ANGLE_RADS,
          ArmSim.ENCODER_DISTANCE_PER_PULSE,
          0.0 // Add noise with a std-dev of 1 tick
          );

  private SparkAbsoluteEncoderSim absoluteEncoderSim;

  // Create a Mechanism2d visualization of the arm.
  private final Mechanism2d mech2d = new Mechanism2d(40, 20);
  private final MechanismRoot2d mechArmPivot = mech2d.getRoot("ArmPivot", 20, 10);
  private final MechanismLigament2d mechArmTower =
      mechArmPivot.append(new MechanismLigament2d("ArmTower", 10, -90));
  private final MechanismLigament2d mechArm =
      mechArmPivot.append(
          new MechanismLigament2d(
              "Arm",
              Units.metersToInches(ArmSim.ARM_LENGTH_METERS),
              Units.radiansToDegrees(armSim.getAngleRads()) - 90,
              6,
              new Color8Bit(Color.kYellow)));

  /** Create a new ArmModel. */
  public ArmModel(ArmSubsystem armSubsystemToSimulate) {

    armSubsystem = armSubsystemToSimulate;
    simulationInit();

    // Put Mechanism 2d to SmartDashboard
    // To view the Arm visualization, select Network Tables -> SmartDashboard -> Arm Sim
    SmartDashboard.putData("Arm Sim", mech2d);
    mechArmTower.setColor(new Color8Bit(Color.kBlue));
  }

  /** Initialize the arm simulation. */
  public void simulationInit() {

    // Setup a simulation of the SparkMax motors and methods to set values
    sparkSimArm = new SparkMaxSim(armSubsystem.getMotor(), armGearbox);
    absoluteEncoderSim = sparkSimArm.getAbsoluteEncoderSim();
  }

  /** Update the simulation model. */
  public void updateSim() {
    // In this method, we update our simulation of what our arm are doing
    // First, we set our "inputs" (voltages)
    armSim.setInput(armSubsystem.getVoltageCommand());

    // Next, we update it. The standard loop time is 20ms.
    armSim.update(0.020);

    // Finally, we run the spark simulations and save the current so it can be retrieved later.
    sparkSimArm.iterate(armSim.getVelocityRadPerSec(), 12.0, 0.02);
    sparkSimArm.setPosition(armSim.getAngleRads() - ArmConstants.ARM_OFFSET_RADS);
    absoluteEncoderSim.setPosition(
        Units.radiansToRotations(armSim.getAngleRads())
            + Units.degreesToRotations(ArmConstants.ABSOLUTE_OFFSET_DEGREES));

    // Update arm visualization with angle
    mechArm.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));
  }

  /** Return the simulated arm motor current. */
  public double getSimArmCurrent() {
    return simArmCurrent;
  }

  @Override
  public void close() {
    mech2d.close();
    mechArmPivot.close();
    mechArm.close();
  }
}
