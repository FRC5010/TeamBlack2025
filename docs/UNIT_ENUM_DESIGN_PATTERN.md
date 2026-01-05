# Unit Enum Design Pattern

This document explains the design pattern used for all unit enums in the FRC5010Example project.

## Pattern Overview

```java
public enum [UnitType]Unit {
  ENUM_VALUE_1("alias1", "alias2", "alias3"),
  ENUM_VALUE_2("alias1", "alias2"),
  // ... more values
  ;

  private final String[] aliases;

  [UnitType]Unit(String... aliases) {
    this.aliases = aliases;
  }

  public static [UnitType]Unit fromString(String unitString) {
    // Error handling
    // Normalization
    // Alias matching
    // Clear error message
  }

  public String[] getAliases() {
    return aliases;
  }
}
```

## Complete Example: DistanceUnit

```java
package org.frc5010.common.config.units;

/**
 * Enum representing distance unit types.
 * Each enum value can be parsed from multiple string representations.
 */
public enum DistanceUnit {
  METERS("m", "meter", "meters"),
  INCHES("in", "inch", "inches"),
  FEET("ft", "foot", "feet"),
  MILLIMETERS("mm", "millimeter", "millimeters"),
  CENTIMETERS("cm", "centimeter", "centimeters"),
  YARDS("yd", "yard", "yards");

  private final String[] aliases;

  DistanceUnit(String... aliases) {
    this.aliases = aliases;
  }

  /**
   * Attempts to parse a string into a DistanceUnit.
   *
   * @param unitString The string representation of the unit
   * @return The matching DistanceUnit
   * @throws IllegalArgumentException if no matching unit is found
   */
  public static DistanceUnit fromString(String unitString) {
    if (unitString == null) {
      throw new IllegalArgumentException("Unit string cannot be null");
    }
    String normalized = unitString.trim().toLowerCase();
    for (DistanceUnit unit : values()) {
      for (String alias : unit.aliases) {
        if (alias.equals(normalized)) {
          return unit;
        }
      }
    }
    throw new IllegalArgumentException("Unknown distance unit: " + unitString);
  }

  public String[] getAliases() {
    return aliases;
  }
}
```

## Design Decisions Explained

### 1. Constructor with Varargs

```java
DistanceUnit(String... aliases) {
  this.aliases = aliases;
}
```

**Why?**
- Allows flexible alias definition (1 or many)
- Clean syntax for declaration
- Type-safe

**Usage:**
```java
METERS("m", "meter", "meters")    // 3 aliases
FEET("ft", "foot", "feet")        // 3 aliases
INCHES("in", "inch", "inches")    // 3 aliases
```

### 2. Static Factory Method

```java
public static DistanceUnit fromString(String unitString) {
  // ...
}
```

**Why?**
- Standard pattern for parsing strings to enums
- Provides clear intent
- Allows custom parsing logic
- Named method is clearer than casting

**Usage:**
```java
DistanceUnit unit = DistanceUnit.fromString("meters");
// vs
DistanceUnit unit = (DistanceUnit) Enum.valueOf(DistanceUnit.class, "METERS");
```

### 3. Null Checking

```java
if (unitString == null) {
  throw new IllegalArgumentException("Unit string cannot be null");
}
```

**Why?**
- Fail fast with clear error message
- Prevent cryptic NullPointerException
- Consistent error handling

**Result:**
```
Exception: Unit string cannot be null
// vs
java.lang.NullPointerException
  at DistanceUnit.fromString(DistanceUnit.java:45)
```

### 4. String Normalization

```java
String normalized = unitString.trim().toLowerCase();
```

**Why?**
- Handles whitespace (e.g., " meters ")
- Case-insensitive input (e.g., "Meters", "METERS")
- Improves user experience

**Results:**
```
DistanceUnit.fromString(" METERS ") ✓ Works
DistanceUnit.fromString("meters")   ✓ Works
DistanceUnit.fromString("Meters")   ✓ Works
```

### 5. Comprehensive Error Messages

```java
throw new IllegalArgumentException("Unknown distance unit: " + unitString);
```

**Why?**
- Shows user what they provided
- Clear action needed (choose valid unit)
- Helps with debugging

**Result:**
```
Exception: Unknown distance unit: meteres
// User can see they misspelled "meters"
```

### 6. Getter for Introspection

```java
public String[] getAliases() {
  return aliases;
}
```

**Why?**
- Allows code to discover valid aliases
- Useful for documentation
- Enables dynamic validation

**Usage:**
```java
for (String alias : DistanceUnit.METERS.getAliases()) {
  System.out.println(alias);  // m, meter, meters
}
```

## Enum Integration with UnitsParser

Each enum is integrated into UnitsParser with a dedicated parsing method:

```java
public static Distance parseDistance(double magnitude, String unit) {
  DistanceUnit distanceUnit = DistanceUnit.fromString(unit);
  switch (distanceUnit) {
    case METERS:
      return Meters.of(magnitude);
    case INCHES:
      return Inches.of(magnitude);
    case FEET:
      return Feet.of(magnitude);
    case MILLIMETERS:
      return Millimeters.of(magnitude);
    case CENTIMETERS:
      return Centimeters.of(magnitude);
    case YARDS:
      return Feet.of(magnitude * 3);
    default:
      System.err.println("Unexpected distance unit: " + distanceUnit);
      return Meters.of(magnitude);
  }
}
```

**Benefits:**
- Enum parsing separated from conversion
- Clear separation of concerns
- Each enum responsible for validation
- Each method responsible for conversion

## JSON Schema Integration

Each enum has a corresponding JSON schema that validates enum values:

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "Distance Unit Value",
  "type": "object",
  "required": ["val", "uom"],
  "properties": {
    "val": {
      "type": "number",
      "description": "The numeric distance value"
    },
    "uom": {
      "type": "string",
      "enum": [
        "m", "meter", "meters",
        "in", "inch", "inches",
        "ft", "foot", "feet",
        "mm", "millimeter", "millimeters",
        "cm", "centimeter", "centimeters",
        "yd", "yard", "yards"
      ],
      "description": "Distance unit (meters, inches, feet, mm, cm, or yards)"
    }
  }
}
```

**Relationship:**
```
DistanceUnit enum values
    ↓
    + aliases for each value
    ↓
JSON schema enum constraint
    ↓
VSCode validation feedback
```

## Extending the Pattern

### Adding a New Unit Type

1. **Create the enum class:**
```java
public enum FrequencyUnit {
  HERTZ("hz", "hertz"),
  KILOHERTZ("khz", "kilohertz"),
  MEGAHERTZ("mhz", "megahertz");

  private final String[] aliases;

  FrequencyUnit(String... aliases) {
    this.aliases = aliases;
  }

  public static FrequencyUnit fromString(String unitString) {
    if (unitString == null) {
      throw new IllegalArgumentException("Unit string cannot be null");
    }
    String normalized = unitString.trim().toLowerCase();
    for (FrequencyUnit unit : values()) {
      for (String alias : unit.aliases) {
        if (alias.equals(normalized)) {
          return unit;
        }
      }
    }
    throw new IllegalArgumentException("Unknown frequency unit: " + unitString);
  }

  public String[] getAliases() {
    return aliases;
  }
}
```

2. **Add parsing method to UnitsParser:**
```java
public static Frequency parseFrequency(double magnitude, String unit) {
  FrequencyUnit freqUnit = FrequencyUnit.fromString(unit);
  switch (freqUnit) {
    case HERTZ:
      return Hertz.of(magnitude);
    case KILOHERTZ:
      return Hertz.of(magnitude * 1000);
    case MEGAHERTZ:
      return Hertz.of(magnitude * 1000000);
    default:
      System.err.println("Unexpected frequency unit: " + freqUnit);
      return Hertz.of(magnitude);
  }
}
```

3. **Create JSON schema:**
```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "Frequency Unit Value",
  "type": "object",
  "required": ["val", "uom"],
  "properties": {
    "val": {
      "type": "number"
    },
    "uom": {
      "type": "string",
      "enum": ["hz", "hertz", "khz", "kilohertz", "mhz", "megahertz"]
    }
  }
}
```

## Pattern Characteristics

| Aspect | Implementation |
|--------|---|
| **Safety** | Type-safe enum values, compile-time checked |
| **Flexibility** | Multiple aliases per unit, case-insensitive |
| **Error Handling** | Clear exceptions with actionable messages |
| **Extensibility** | Easy to add new units or aliases |
| **Performance** | O(n*m) parsing, negligible for typical use |
| **Thread Safety** | Enums are singletons, inherently thread-safe |
| **Testability** | Easy to unit test fromString() method |
| **Documentation** | Self-documenting enum values |
| **Integration** | Works seamlessly with JSON schemas |

## Comparison with Alternatives

### Before (String Constants)
```java
public static final String M = "m";
public static final String CM = "cm";
// ... 30+ more constants

public static Distance parseDistance(double magnitude, String unit) {
  switch (unit.trim().toLowerCase()) {
    case M:
    case "meter":
    case "meters":
      return Meters.of(magnitude);
    // ... many more cases
  }
}
```

**Cons:**
- No type safety
- Scattered string cases
- Error validation at runtime
- Hard to find all units

### After (Enums)
```java
public enum DistanceUnit {
  METERS("m", "meter", "meters"),
  // ... more units
}

public static Distance parseDistance(double magnitude, String unit) {
  DistanceUnit distanceUnit = DistanceUnit.fromString(unit);
  switch (distanceUnit) {
    case METERS:
      return Meters.of(magnitude);
    // ... more cases
  }
}
```

**Pros:**
- Type-safe
- Centralized definitions
- Error validation earlier
- Clear error messages
- Easy to extend

## Best Practices

1. **Always null check** - Provide clear error for null input
2. **Always normalize** - Handle whitespace and case variations
3. **Always validate** - Use enums to ensure valid values
4. **Always document** - Explain supported aliases
5. **Always test** - Unit test fromString() thoroughly
6. **Always extend** - Use the pattern for new unit types
7. **Always integrate** - Create corresponding JSON schemas

## Testing the Pattern

```java
@Test
public void testValidUnits() {
  assertEquals(DistanceUnit.METERS, DistanceUnit.fromString("meters"));
  assertEquals(DistanceUnit.FEET, DistanceUnit.fromString("ft"));
  assertEquals(DistanceUnit.INCHES, DistanceUnit.fromString("in"));
}

@Test
public void testCaseInsensitive() {
  assertEquals(DistanceUnit.METERS, DistanceUnit.fromString("METERS"));
  assertEquals(DistanceUnit.FEET, DistanceUnit.fromString("FT"));
}

@Test
public void testWhitespace() {
  assertEquals(DistanceUnit.METERS, DistanceUnit.fromString(" meters "));
}

@Test
public void testInvalidUnit() {
  assertThrows(IllegalArgumentException.class,
    () -> DistanceUnit.fromString("invalid"));
}

@Test
public void testNullUnit() {
  assertThrows(IllegalArgumentException.class,
    () -> DistanceUnit.fromString(null));
}

@Test
public void testAliases() {
  String[] aliases = DistanceUnit.METERS.getAliases();
  assertThat(aliases).contains("m", "meter", "meters");
}
```

## Conclusion

This enum pattern provides:
- **Type Safety** - Compile-time checking
- **Validation** - Early error detection
- **Clarity** - Self-documenting code
- **Maintainability** - Centralized definitions
- **Extensibility** - Easy to add new units
- **Integration** - Works with JSON schemas

It's the recommended approach for any system that needs to parse and validate string-based configurations.
