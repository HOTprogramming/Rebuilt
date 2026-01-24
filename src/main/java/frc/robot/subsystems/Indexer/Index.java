package frc.robot.subsystems.Indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Index.IndexKraken;

public class Index extends SubsystemBase{
    private final IndexStats ioStats = new IndexStats();
    private final IndexKraken m_IndexKraken = IndexKraken.getInstance();

    
}
