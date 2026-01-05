# Schema Validation Report
## akit_swerve_drivetrain.json vs AKitSwerveDrivetrainJson.java

This report validates that the JSON schema correctly enforces the structure required by the `AKitSwerveDrivetrainJson.java` class and its dependency `DrivetrainConstantsJson.java`.

---

## Top-Level Properties

### AKitSwerveDrivetrainJson Class Fields
| Field | Type | Java Default | Schema Enforcement |
|-------|------|-------------|--------------------|
| `type` | String | "SparkTalon" | ✅ enum: ["TalonFX", "SparkTalon", "Spark"] |
| `constants` | DrivetrainConstantsJson | Required | ✅ Required object |
| `gamePiecesJson` | Optional | Empty | ⚠️ Not in JSON (internal field) |

---

## DrivetrainConstantsJson Properties

### Required Fields
All these fields are required in the schema and Java class:

| Field | Java Type | Schema Type | Validation |
|-------|-----------|-------------|-----------|
| `trackWidth` | UnitValueJson | measurement | ✅ {val: number, uom: string} |
| `wheelBase` | UnitValueJson | measurement | ✅ {val: number, uom: string} |
| `wheelDiameter` | UnitValueJson | measurement | ✅ {val: number, uom: string} |
| `bumperFrameWidth` | UnitValueJson | measurement | ✅ {val: number, uom: string} |
| `bumperFrameLength` | UnitValueJson | measurement | ✅ {val: number, uom: string} |
| `maxDriveSpeed` | UnitValueJson | measurement | ✅ {val: number, uom: string} |
| `gyro` | GyroSettingsConfigurationJson | object | ✅ See gyro details below |
| `driveGearRatio` | String | string | ✅ Format: "1:ratio" |
| `steerGearRatio` | String | string | ✅ Format: "1:ratio" |
| `driveMotorControl` | MotorSystemIdJson | motorControl | ✅ See motor control details below |
| `steerMotorControl` | MotorSystemIdJson | motorControl | ✅ See motor control details below |
| `modules` | Map<String, ModuleConfigJson> | object | ✅ Properties: frontLeft, frontRight, backLeft, backRight |
| `coupleRatio` | double | number | ✅ Numeric value |
| `invertLeftSide` | boolean | boolean | ✅ True/false |
| `invertRightSide` | boolean | boolean | ✅ True/false |
| `steerInertia` | UnitValueJson | measurement | ✅ {val: number, uom: string} |
| `driveInertia` | UnitValueJson | measurement | ✅ {val: number, uom: string} |
| `robotMass` | UnitValueJson | measurement | ✅ {val: number, uom: string} |
| `wheelCOF` | double | number | ✅ Numeric value (coefficient of friction) |
| `slipCurrent` | UnitValueJson | measurement | ✅ {val: number, uom: string} |
| `canbus` | String | string | ✅ CAN bus name |

---

## Gyroscope Configuration

### GyroSettingsConfigurationJson Structure
| Field | Java Type | Schema Validation |
|-------|-----------|------------------|
| `type` | String | ✅ enum: ["pigeon2", "pigeon1", "navx", "adxrs450", "yagsl"] |
| `id` | int | ✅ integer (0-62) |
| `inverted` | boolean | ✅ boolean |
| `canbus` | String | ✅ string |

**Java Implementation Note:** The type field is case-sensitive. Valid values from the code are:
- `"pigeon2"` - Phoenix Pigeon 2 gyro
- `"pigeon1"` - Phoenix Pigeon 1 gyro
- `"navx"` - NavX gyroscope
- `"yagsl"` - YAGSL gyroscope (from YAGSL library)

---

## Motor System ID Configuration

### MotorSystemIdJson Structure
| Field | Java Type | Schema Validation | Notes |
|-------|-----------|------------------|-------|
| `closedLoopRamp` | UnitValueJson | ✅ measurement | Default: 0.25 seconds |
| `openLoopRamp` | UnitValueJson | ✅ measurement | Default: 0.25 seconds |
| `feedBack` | FeedBack object | ✅ object | Contains p, i, d |
| `feedBack.p` | double | ✅ number | Proportional gain |
| `feedBack.i` | double | ✅ number | Integral gain |
| `feedBack.d` | double | ✅ number | Derivative gain |
| `maxVelocity` | UnitValueJson | ✅ measurement | Default: 0 |
| `maxAcceleration` | UnitValueJson | ✅ measurement | Default: 0 |
| `feedForward` | FeedForward object | ✅ object | Contains s, g, v, a |
| `feedForward.s` | double | ✅ number | Static constant |
| `feedForward.g` | double | ✅ number | Gravity constant |
| `feedForward.v` | double | ✅ number | Velocity constant |
| `feedForward.a` | double | ✅ number | Acceleration constant |
| `controlMode` | String | ✅ enum | "CLOSED_LOOP" or "OPEN_LOOP" |

**⚠️ Important Note:** The current akit_swerve_drivetrain.json has simplified feedforward (only s, v, a), but the schema now validates the complete MotorSystemIdJson structure which includes the gravity constant 'g'.

---

## Motor Setup Configuration

### MotorSetupJson Structure
| Field | Java Type | Schema Validation | Notes |
|-------|-----------|------------------|-------|
| `name` | String | ✅ Required string | Descriptive name |
| `logLevel` | String | ✅ enum | "LOW", "MEDIUM", "HIGH" |
| `motorType` | String | ✅ string | e.g., "KrakenX60", "Falcon500", "NEO" |
| `controllerType` | String | ✅ enum | "talonfx", "talonsrx", "sparkmax", "victorspx" |
| `canId` | int | ✅ Required int (0-62) | CAN identifier |
| `canBus` | String | ✅ string | Default: "" (rio bus) |
| `idleMode` | String | ✅ enum | "BRAKE" or "COAST" |
| `currentLimit` | UnitValueJson | ✅ measurement | Default: 40 amps |
| `inverted` | boolean | ✅ boolean | Motor inversion |
| `numberOfMotors` | int | ✅ int (min: 1) | Includes followers |
| `followers` | FollowerMotorJson[] | ✅ array | Follower configuration |
| `robotToMotor` | Translation3dJson | ✅ object | Motor position relative to robot |
| `movementPlane` | String | ✅ enum | "XY", "XZ", "YZ" |

**Schema Validation for Followers:**
```json
{
  "canId": integer (0-62),
  "inverted": boolean
}
```

---

## Swerve Module Configuration

### ModuleConfigJson Structure (4 instances: frontLeft, frontRight, backLeft, backRight)
| Field | Java Type | Schema Validation |
|-------|-----------|------------------|
| `driveMotorSetup` | MotorSetupJson | ✅ motorSetup object |
| `steerMotorSetup` | MotorSetupJson | ✅ motorSetup object |
| `encoderId` | int | ✅ integer (0-15) |
| `absoluteOffset` | UnitValueJson | ✅ measurement (rotations or degrees) |
| `encoderInverted` | boolean | ✅ boolean |

---

## Schema Compliance Summary

✅ **Fully Compliant:**
- Type enum values match Java code
- All required fields are enforced
- Nested object structures match Java classes
- CAN ID ranges (0-62) are validated
- Motor movement planes validated
- Idle mode enum enforced
- Control mode enum enforced

⚠️ **Partially Compliant (Loose):**
- `motorType` - Schema allows any string (Java code may have specific types)
- `movementPlane` rotations - Currently allows any string, could be more restrictive
- `uom` (units of measure) - Allows any string, could define valid units explicitly

🔄 **Motor Control Differences:**
- Original JSON uses simplified structure (s, v, a only)
- Java class `MotorSystemIdJson` has full structure (s, g, v, a)
- Schema enforces complete structure - **May require JSON file updates**

---

## Recommendations

1. **Update motor control definitions** in JSON files to include all fields from `MotorSystemIdJson`:
   - Add `closedLoopRamp`, `openLoopRamp`, `maxVelocity`, `maxAcceleration`, `controlMode`
   - Add `g` (gravity) to feedForward constants

2. **Consider stricter motor type validation** - Define enum of supported motor types

3. **Define valid UOM (units of measure) values** - Create enum for common units like:
   - inches, feet, meters, mm
   - rad, deg, rotations
   - m/sec, m/sec^2, rad/sec
   - kg, lbs, kg*m^2

4. **Test the schema against current JSON file** to identify any incompatibilities

---

## Related Java Classes
- `AKitSwerveDrivetrainJson.java` - Main configuration class
- `DrivetrainConstantsJson.java` - Constants holder
- `MotorSystemIdJson.java` - Motor control parameters
- `MotorSetupJson.java` - Individual motor setup
- `GyroSettingsConfigurationJson.java` - Gyroscope configuration
- `UnitValueJson.java` - Unit value representation
