# Javadoc Guidelines for FRC5010Example

This document provides guidelines for writing effective Javadoc comments throughout the FRC5010Example codebase.

## Table of Contents

- [Overview](#overview)
- [Javadoc Standards](#javadoc-standards)
- [File Categories](#file-categories)
- [Examples](#examples)
- [Automated Documentation Generation](#automated-documentation-generation)

## Overview

Javadoc is an essential tool for documenting Java code. It improves code maintainability, helps new team members understand the codebase, and generates professional API documentation. This project uses Gradle to automatically generate HTML documentation from Javadoc comments.

### Key Benefits

- **API Documentation**: Auto-generates professional HTML documentation
- **IDE Integration**: Javadoc comments appear in IDE tooltips and autocomplete
- **Code Clarity**: Forces developers to think about the purpose and usage of their code
- **Maintenance**: Future developers can quickly understand complex implementations

## Javadoc Standards

### Class Documentation Template

Every public class should have a class-level Javadoc comment that includes:

```java
/**
 * Brief one-line description of the class.
 *
 * <p>More detailed explanation of what this class does, how it's used,
 * and any important constraints or requirements. This can span multiple
 * paragraphs and include examples if helpful.
 *
 * <p>Usage example:
 * <pre>
 *   MyClass instance = new MyClass();
 *   instance.doSomething();
 * </pre>
 *
 * @see RelatedClass
 * @see RelatedInterface
 */
public class MyClass {
  // implementation
}
```

### Field Documentation

Public fields (especially configuration data) should be documented:

```java
/**
 * Brief description of what this field represents.
 *
 * <p>Include details about:
 * <ul>
 *   <li>The unit of measurement (if applicable)</li>
 *   <li>Valid value ranges</li>
 *   <li>Default values</li>
 *   <li>Any special behavior or side effects</li>
 * </ul>
 */
public double configValue = 0.0;
```

### Method Documentation

Every public method should include:

```java
/**
 * Brief one-line description of what this method does.
 *
 * <p>More detailed description of the method's behavior, including:
 * <ul>
 *   <li>What side effects it may have</li>
 *   <li>What happens in edge cases</li>
 *   <li>How it interacts with other methods/systems</li>
 * </ul>
 *
 * <p>Example usage:
 * <pre>
 *   result = myMethod(value1, value2);
 * </pre>
 *
 * @param param1 description of the first parameter, including type info
 * @param param2 description of the second parameter
 * @return description of the return value and its meaning
 * @throws IOException if an I/O error occurs
 * @throws IllegalArgumentException if param1 is negative
 * @see #relatedMethod()
 */
public ReturnType myMethod(Type param1, Type param2) throws IOException {
  // implementation
}
```

### Constructor Documentation

Constructors should document initialization:

```java
/**
 * Creates a new instance with the specified configuration.
 *
 * <p>This constructor initializes all required components and validates
 * the input parameters.
 *
 * @param name the unique name identifier for this instance
 * @param config the configuration object (must not be null)
 * @throws NullPointerException if config is null
 * @throws IllegalArgumentException if name is empty
 */
public MyClass(String name, Configuration config) {
  // implementation
}
```

## File Categories

### 1. Configuration Classes (`src/main/java/org/frc5010/common/config/json/`)

Configuration classes load JSON data and configure robot subsystems. These should document:
- What JSON structure they expect
- How the configuration affects robot behavior
- Any constraints on configuration values

**Current status**: Improved in progress
- ✅ `CameraConfigurationJson` - Enhanced with comprehensive field and method documentation

**Files needing Javadoc**:
- `DriveteamControllerConfiguration.java`
- `RobotJson.java`
- `VisionPropertiesJson.java`
- `DrivetrainPropertiesJson.java`
- `*ConfigurationJson.java` device configuration classes

**Example** (already implemented):
```java
/**
 * Configuration data class for camera systems in an FRC robot.
 *
 * <p>This class represents the JSON configuration for a single camera, including its type
 * (Limelight, PhotonVision, AprilTag), physical pose relative to the robot center, and
 * calibration parameters.
 */
public class CameraConfigurationJson {
```

### 2. Parser Classes (`src/main/java/org/frc5010/common/config/`)

Parser classes read and process configuration files. They should document:
- What files they read
- The structure of expected data
- Error conditions and exceptions

**Current status**: Improved in progress
- ✅ `RobotParser` - Enhanced with detailed documentation about the configuration loading process

**Files needing Javadoc**:
- `SubsystemParser.java`
- `RobotsParser.java`
- `UnitsParser.java`

### 3. Subsystem Classes (`src/main/java/org/frc5010/common/subsystems/`)

Subsystem classes implement robot functionality. Document:
- The purpose and behavior of the subsystem
- Key methods and their effects
- Important timing or state considerations

**Files needing Javadoc**:
- `CameraSystem.java`
- `AprilTagPoseSystem.java`
- `LEDStrip.java`
- `PowerDistribution5010.java`

### 4. Vision/Sensor Classes (`src/main/java/org/frc5010/common/sensors/`)

Vision and sensor classes interact with hardware. Document:
- What hardware they interface with
- Return value meanings and units
- Calibration and setup requirements

**Files needing Javadoc**:
- All camera implementations
- Sensor implementations (gyro, limit switches, etc.)

### 5. Motor/Drive Classes (`src/main/java/org/frc5010/common/motors/`)

Motor control classes need clear documentation about:
- Control modes and their parameters
- Units and scale factors
- Hardware limitations

**Files needing Javadoc**:
- `GenericMotorController.java` and implementations
- `AngularControlMotor.java`
- `VelocityControlMotor.java`
- `PositionControlMotor.java`

### 6. Utility Classes (`src/main/java/org/frc5010/common/`)

Utility classes provide helper functions. Document:
- What transformations they perform
- Input/output formats
- Use cases

**Files needing Javadoc**:
- `AllianceFlip.java`
- `RobotIdentity.java`
- `SystemIdentification.java`

## Examples

### Example 1: Configuration Class

```java
/**
 * JSON configuration for Limelight camera systems.
 *
 * <p>This class deserializes Limelight-specific configuration from JSON
 * and provides methods to instantiate and configure Limelight cameras
 * on the robot.
 *
 * <p>Typical JSON structure:
 * <pre>
 * {
 *   "name": "limelight",
 *   "type": "limelight",
 *   "use": "apriltag",
 *   "x": 0.0,
 *   "y": 0.0,
 *   "z": 0.5
 * }
 * </pre>
 */
public class LimeLightConfigurationJson implements CameraConfiguration {
```

### Example 2: Complex Method

```java
/**
 * Computes the robot's pose using multiple vision targets with outlier rejection.
 *
 * <p>This method processes all tracked AprilTag detections and uses a
 * weighted average to estimate the robot's position and orientation.
 * Low-confidence detections are automatically filtered out using the
 * ambiguity threshold.
 *
 * <p>The algorithm:
 * <ol>
 *   <li>Filters detections by confidence and ambiguity ratio</li>
 *   <li>Computes individual pose estimates from each tag</li>
 *   <li>Weights estimates by detection confidence</li>
 *   <li>Returns the weighted average pose</li>
 * </ol>
 *
 * @param detections the list of AprilTag detections from the camera
 * @param cameraToRobot the transform from camera coordinates to robot center
 * @return the estimated robot pose in field coordinates, or empty if no valid estimates
 * @throws NullPointerException if detections or cameraToRobot is null
 * @see AprilTagDetection
 */
public Optional<Pose3d> estimatePose(List<AprilTagDetection> detections,
                                      Transform3d cameraToRobot) {
```

### Example 3: Enum or Constants Class

```java
/**
 * Standard AprilTag field layout and utilities for pose estimation.
 *
 * <p>This class provides access to the official FIRST AprilTag field layout
 * for the current game season. It also includes utility methods for
 * common AprilTag-related calculations.
 *
 * <p>Usage:
 * <pre>
 *   AprilTagFieldLayout layout = AprilTags.aprilTagFieldLayout;
 *   Optional<Pose3d> tagPose = layout.getTagPose(tagID);
 * </pre>
 */
public class AprilTags {
  /** The AprilTag field layout for the 2026 game season */
  public static AprilTagFieldLayout aprilTagFieldLayout;
```

## Automated Documentation Generation

### Building Javadocs Locally

To generate Javadoc HTML documentation on your machine:

```bash
./gradlew javadoc
```

The generated HTML will be in `build/docs/javadoc/`.

### Viewing Javadocs

1. **In your IDE**: Hover over any class, method, or field to see Javadoc tooltips
2. **In a browser**: Open `build/docs/javadoc/index.html` after running the Gradle task
3. **GitHub Pages**: The `javadoc.yml` workflow automatically deploys docs on push to `main`

### GitHub Pages Deployment

The project includes a GitHub Actions workflow (`.github/workflows/javadoc.yml`) that:
1. Checks out the code
2. Builds the project
3. Generates Javadocs
4. Publishes them to GitHub Pages

Your documentation will be automatically deployed when you push to the `main` branch.

## Implementation Priority

To ensure the most critical parts of the codebase are well-documented, prioritize adding Javadoc in this order:

1. **High Priority** - Core public APIs
   - `CameraConfigurationJson` ✅
   - `RobotParser` ✅
   - `DeviceConfiguration` ✅
   - All public interfaces

2. **Medium Priority** - Configuration classes
   - `RobotJson`
   - `DriveteamControllerConfiguration`
   - All JSON configuration classes

3. **Medium Priority** - Major subsystems
   - `GenericRobot`
   - `GenericDrivetrain`
   - `CameraSystem` and implementations

4. **Lower Priority** - Utilities and helpers
   - Utility functions
   - Helper classes
   - Test utilities

## Maintenance Notes

- When adding new public classes or methods, always include Javadoc before committing
- Use `@deprecated` tags when removing old APIs
- Keep Javadoc synchronized with code changes
- Review Javadoc warnings during builds

## Tools and Resources

- [Official Javadoc Guide](https://docs.oracle.com/javase/8/docs/technotes/tools/windows/javadoc.html)
- [Javadoc Tag Reference](https://docs.oracle.com/javase/8/docs/technotes/tools/windows/javadoc.html#tags)
- [Best Practices for Javadoc](https://developer.oracle.com/java/javadoc-best-practices)

## Questions?

For questions about Javadoc standards in this project, refer to this document or check existing examples in the improved files.
