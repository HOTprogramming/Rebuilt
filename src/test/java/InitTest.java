import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import frc.robot.RobotContainer;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class InitTest {

    RobotContainer container;

    @BeforeEach
    void setup() {
        assert HAL.initialize(500, 0);
    }

    @Test
    void createRobotContainer() {
        try{
            container = new RobotContainer();
            assertEquals(true, true);
        } catch(Exception e){
            assertEquals(true, false, e.toString());
        }
    }
}