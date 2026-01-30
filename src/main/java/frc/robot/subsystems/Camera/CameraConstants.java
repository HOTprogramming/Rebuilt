package frc.robot.subsystems.Camera;

import java.security.PublicKey;
import java.util.EnumMap;
import java.util.Map;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.Camera.Camera.CameraPositions;

public class CameraConstants {
    public class CameraConstant{

        public CameraConstant(String name, 
                              Translation3d relativeTranslation, 
                              Rotation3d relativeRotation, 
                              Matrix<N3, N1> singleTagStdDevs,
                              Matrix<N3, N1> multiTagStdDevs) {
            this.name = name;
            this.transform = new Transform3d(relativeTranslation, relativeRotation);
            this.singleTagStdDevs = singleTagStdDevs;
            this.multiTagStdDevs = multiTagStdDevs;
        }

        private Matrix<N3, N1> singleTagStdDevs;
        public Matrix<N3, N1> getSingleTagStdDevs() {
            return singleTagStdDevs;
        }

        private Matrix<N3, N1> multiTagStdDevs;
        public Matrix<N3, N1> getMultiTagStdDevs() {
            return multiTagStdDevs;
        }

        private String name;
        public String getName() {
            return name;
        }
        private Transform3d transform;
        public Transform3d getTransform() {
            return transform;
        }
    }

    //Intake (Front): 12.483in forward of robot middle. On center. 8.625in above ground Perpendicular to floor
    //Shooting (Back): 12.425in reward of robot middle. On center. 6.008in above ground. Angled 8 degrees up from floor
    //Left Side: 10.696in left of robot middle. 30 degrees left of rear. 16.838in above ground. angled 5 degrees up from floor
    //Right Side: 10.696in right of robot middle. 30 degrees right of rear. 16.838in above ground. angled 5 degrees up from floor
    //(X, Y, Z)
    //X: Front and back (Front +)
    //Y: Left and right (Left +)
    //Z: Vertical distance from the floor to the camera (Up +)

    public double[] STDEV_GAIN = new double[] {.7, .7, .5};
    public double MAX_DISTANCE = 5.5;
    public static final int[] REEF_TAGS_RED = new int[] {6, 7, 8, 9, 10, 11};
    public static final int[] REEF_TAGS_BLUE = new int[] {17, 18, 19, 20, 21, 22};

    public static final int[] IGNORE_ALWAYS = new int[] {3, 16, 4, 5, 14, 15};
    
    public Map<CameraPositions, CameraConstant> cameraConstants = null;
    
    public CameraConstants(){
        cameraConstants = new EnumMap<>(CameraPositions.class);         
        switch (Constants.getRobot()) {
            case DEVBOT:
                cameraConstants.put(CameraPositions.LEFT, new CameraConstant("LEFT",
                        new Translation3d(Units.inchesToMeters(11.2), Units.inchesToMeters(-10.25), Units.inchesToMeters(7.752)), // without wheel squish
                        new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(-20.0), Units.degreesToRadians(-144.5)), // -145

                        VecBuilder.fill(4, 4, 8),
                        VecBuilder.fill(0.5, 0.5, 1)));

                
                cameraConstants.put(CameraPositions.RIGHT, new CameraConstant("RIGHT",
                        new Translation3d(Units.inchesToMeters(-11.2), Units.inchesToMeters(-10.25), Units.inchesToMeters(7.752)), // 10.84 6.516
                        new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(-20.0), Units.degreesToRadians(-35.5)),

                        VecBuilder.fill(4, 4, 8),
                        VecBuilder.fill(0.5, 0.5, 1)));

                cameraConstants.put(CameraPositions.TOP, new CameraConstant("TOP", // top
                        new Translation3d(Units.inchesToMeters(-7.0), Units.inchesToMeters(-8.76), Units.inchesToMeters(42)),
                        new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(-22.5), Units.degreesToRadians(62.77)),
                        VecBuilder.fill(4, 4, 8),
                        VecBuilder.fill(0.5, 0.5, 1)));
                break;
            case COMPBOT:
                cameraConstants.put(CameraPositions.LEFT, new CameraConstant("LEFT",
                        new Translation3d(Units.inchesToMeters(11.2), Units.inchesToMeters(-10.25), Units.inchesToMeters(7.752)), // without wheel squish
                        new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(-20.0), Units.degreesToRadians(-144.5)), // -145

                        VecBuilder.fill(4, 4, 8),
                        VecBuilder.fill(0.5, 0.5, 1)));

                
                cameraConstants.put(CameraPositions.RIGHT, new CameraConstant("RIGHT",
                        new Translation3d(Units.inchesToMeters(-11.2), Units.inchesToMeters(-10.25), Units.inchesToMeters(7.752)), // 10.84 6.516
                        new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(-20.0), Units.degreesToRadians(-35.5)),

                        VecBuilder.fill(4, 4, 8),
                        VecBuilder.fill(0.5, 0.5, 1)));

                cameraConstants.put(CameraPositions.TOP, new CameraConstant("TOP", // top
                        new Translation3d(Units.inchesToMeters(-8.5), Units.inchesToMeters(-9.72), Units.inchesToMeters(39.36)),
                        new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(25), Units.degreesToRadians(-90)),
                        VecBuilder.fill(4, 4, 8),
                        VecBuilder.fill(0.5, 0.5, 1)));
                break;
            case SIMBOT:
                cameraConstants.put(CameraPositions.FRONT, new CameraConstant("a",
                        new Translation3d(Units.inchesToMeters(0.0), Units.inchesToMeters(0.0), Units.inchesToMeters(0.0)),
                        new Rotation3d(0, 0, 0),
                        VecBuilder.fill(4, 4, 8),
                        VecBuilder.fill(0.5, 0.5, 1)));

                
                cameraConstants.put(CameraPositions.TOP, new CameraConstant("b",
                        new Translation3d(Units.inchesToMeters(0.0), Units.inchesToMeters(0.0), Units.inchesToMeters(0.0)),
                        new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(180)),
                        VecBuilder.fill(4, 4, 8),
                        VecBuilder.fill(0.5, 0.5, 1)));
                break;
            case COMP24:
                cameraConstants.put(CameraPositions.LEFT, new CameraConstant("left_camera",
                        new Translation3d(Units.inchesToMeters(2), Units.inchesToMeters(11.49), Units.inchesToMeters(16.74)),
                        new Rotation3d(Units.degreesToRadians(-5.77), Units.degreesToRadians(-9.92), Units.degreesToRadians(120.38)),
                        VecBuilder.fill(4, 4, 8),
                        VecBuilder.fill(0.5, 0.5, 1)));

                      
                
                cameraConstants.put(CameraPositions.RIGHT, new CameraConstant("right_camera",
                        new Translation3d(Units.inchesToMeters(2), Units.inchesToMeters(-11.49), Units.inchesToMeters(16.74)),
                        new Rotation3d(Units.degreesToRadians(5.77), Units.degreesToRadians(-9.92), Units.degreesToRadians(-120.38)),
                        VecBuilder.fill(4, 4, 8),
                        VecBuilder.fill(0.5, 0.5, 1)));

                
                cameraConstants.put(CameraPositions.FRONT, new CameraConstant("front_camera",
                new Translation3d(Units.inchesToMeters(12.483), Units.inchesToMeters(0), Units.inchesToMeters(8.625)),
                new Rotation3d(0, 0, 0),
                VecBuilder.fill(4, 4, 8),
                VecBuilder.fill(0.5, 0.5, 1)));

            
                cameraConstants.put(CameraPositions.BACK, new CameraConstant("back_camera",
                    new Translation3d(Units.inchesToMeters(-12), Units.inchesToMeters(0), Units.inchesToMeters(6.193)),
                    new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(180)),
                    VecBuilder.fill(4, 4, 8),
                    VecBuilder.fill(0.5, 0.5, 1)));

                break;

        }
        

    }
}