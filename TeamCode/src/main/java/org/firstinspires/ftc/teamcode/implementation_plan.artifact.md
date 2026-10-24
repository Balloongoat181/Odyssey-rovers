# FTC Mecanum Robot Control Fixes

This plan outlines the changes needed to fix `test_drive.java` for a mecanum robot with an intake and a shooter.

## User Review Required

> [!NOTE]
> The current code uses methods like `gamepad1.aWasPressed()` which are not part of the standard FTC SDK. I will implement a "current vs. previous" state check to handle button presses correctly.

## Proposed Changes

### Drivetrain & Control Logic

#### [MODIFY] [test_drive.java](file:///C:/Users/joshu/Documents/ftc/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test_drive.java)

- **Gamepad Edge Detection**: Add `Gamepad` objects to track state changes.
- **Mecanum Math**: Correct the comments and ensure strafing is explicitly handled.
- **Shooter Logic**: Fix the error where `shooterOn` was setting `intake` power, and implement actual `shooterPower` control.
- **Power Clamping**: Ensure all motor powers are within the valid range (-1.0 to 1.0).

## Verification Plan

### Automated Tests
- I will use `gradle_build` to ensure the code compiles without errors.

### Manual Verification
- The user should deploy the code and test the following:
  - Left stick X/Y and Right stick X control mecanum movement (forward, strafe, turn).
  - Button 'A' toggles intake direction (flip).
  - Button 'B' toggles intake on/off.
  - Button 'X' toggles shooter on/off.
  - Bumpers and D-pad Up/Down adjust intake and shooter powers respectively.
