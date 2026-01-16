package frc.robot.subsystems.Climber;

import frc.robot.sim.PhysicsSim;
import frc.robot.subsystems.Climber.Climber.ClimberStats;

public class ClimberKraken {
    
private static ClimberKraken instance = null;
private double getAppliedVolts = 0.0;
private boolean useTorque = true;


public static ClimberKraken getInstance() {
    return instance = instance == null ? new ClimberKraken() : instance;
}

public void updateStats(ClimberStats stats){
    if(isSimulation){
        PhysicsSim.getInstance().run();
    }
}

        climberPosition = m_talonFX.getPosition();
        climberVelocity = m_talonFX.getVelocity();
        climberAppliedVolts = m_talonFX.getAppliedVolts();
        climberSupplyCurrent = m_talonFX.getSupplyCurrent();
        climberTorqueCurrent = m_talonFX.getTorqueCurrent();
        climberTempCelsius = m_talonFX.getTempCelsius();
}
