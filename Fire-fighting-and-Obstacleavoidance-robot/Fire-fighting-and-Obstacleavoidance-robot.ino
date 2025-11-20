#include <Servo.h>

// ---------------------------
// 🔌 PIN DEFINITIONS
// ---------------------------

// Flame sensors (digital output mode via analog pins — internally pulled-up or used as digital inputs)
#define FLAME_SENSOR_LEFT   A0
#define FLAME_SENSOR_CENTER A1
#define FLAME_SENSOR_RIGHT  A2

// IR obstacle detection sensors (digital inputs; LOW = no obstacle, HIGH = obstacle detected)
#define IR_SENSOR_LEFT    2
#define IR_SENSOR_CENTER  4
#define IR_SENSOR_RIGHT   8

// Motor driver control pins (H-bridge logic: L1/L2 = left motor, R1/R2 = right motor)
#define MOTOR_L1 5   // Left motor forward
#define MOTOR_L2 6   // Left motor backward
#define MOTOR_R1 9   // Right motor forward
#define MOTOR_R2 10  // Right motor backward

// Water pump activation (relay-controlled, active HIGH)
#define RELAY_PIN 7

// Servo for directing water stream
#define SERVO_PIN 3

// ---------------------------
// 🧠 GLOBAL OBJECTS & VARIABLES
// ---------------------------

// Servo instance for water nozzle control
Servo waterServo;

// Finite State Machine (FSM) states for high-level robot behavior
enum State {
  SEARCHING_FOR_FIRE,     // Idle: waiting for fire detection
  NAVIGATING_TO_FIRE,     // Approaching fire while avoiding obstacles
  TURNING_LEFT,           // Rotating left toward fire or to avoid obstacle
  TURNING_RIGHT,          // Rotating right toward fire or to avoid obstacle
  REVERSING,              // Backing up when trapped
  EXTINGUISHING           // Active firefighting mode (pump + servo sweep)
};

// Current state of the robot's behavior FSM
State currentState = SEARCHING_FOR_FIRE;

// Non-blocking timing control (avoids `delay()` for responsiveness)
unsigned long stateStartTime = 0;          // Time when current state began
unsigned long lastSensorRead = 0;         // Last time sensors were sampled
unsigned long lastDebugPrint = 0;         // Last time debug info was printed
unsigned long servoMoveMillis = 0;        // Servo sweep timing control
unsigned long pumpActivateMillis = 0;     // Pump duration timer

// Timing constants (all in milliseconds)
const unsigned long SENSOR_READ_INTERVAL = 50;     // Sensor polling rate (~20 Hz)
const unsigned long DEBUG_PRINT_INTERVAL = 300;   // Serial debug output rate (~3.3 Hz)
const unsigned long TURN_DURATION = 600;          // Avg time for 90° turn at current speed
const unsigned long REVERSE_DURATION = 1500;      // Back-up maneuver duration
const unsigned long PUMP_DURATION = 3000;         // Total time pump remains active
const unsigned long SERVO_MOVE_INTERVAL = 100;    // Servo step interval (smooth sweep)

// Cached sensor readings (updated every SENSOR_READ_INTERVAL)
bool leftClear, centerClear, rightClear;   // IR: true = path clear
bool flameLeft, flameCenter, flameRight;   // Flame: true = fire detected (active LOW)

// Weighted obstacle-avoidance confidence counters (debouncing + trend tracking)
int leftClearCount = 0;      // Consecutive clear readings on left
int rightClearCount = 0;     // Consecutive clear readings on right
const int CONFIDENCE_THRESHOLD = 3;  // Min additional confidence to bias turning decision

// Servo & pump control state
int servoPos = 90;                   // Initial servo angle (centered)
bool servoSweepingRight = true;      // Direction of sweep (true = increasing angle)
bool pumpActive = false;             // Flag to prevent re-triggering pump

// ---------------------------
// ⚙️ SETUP: Hardware Initialization
// ---------------------------
void setup() {
    Serial.begin(9600);

    // Motor driver pins: configure as OUTPUT for H-bridge control
    pinMode(MOTOR_L1, OUTPUT);
    pinMode(MOTOR_L2, OUTPUT);
    pinMode(MOTOR_R1, OUTPUT);
    pinMode(MOTOR_R2, OUTPUT);

    // Water pump relay: active HIGH, start in OFF state
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);

    // Attach and initialize servo to center position (90°)
    waterServo.attach(SERVO_PIN);
    waterServo.write(servoPos);  // Center the nozzle

    // Flame sensors: configured as digital inputs
    // Note: These sensors output LOW when flame is detected (active-low logic)
    pinMode(FLAME_SENSOR_LEFT, INPUT);
    pinMode(FLAME_SENSOR_CENTER, INPUT);
    pinMode(FLAME_SENSOR_RIGHT, INPUT);

    // IR obstacle sensors: digital inputs (LOW = no obstacle)
    pinMode(IR_SENSOR_LEFT, INPUT);
    pinMode(IR_SENSOR_CENTER, INPUT);
    pinMode(IR_SENSOR_RIGHT, INPUT);

    // Startup messages — helpful for field debugging and demos
    Serial.println("🚀 Fire Fighting Robot with Predictive Navigation Initialized...");
    Serial.println("⏸️  Robot in STANDBY mode - waiting for fire detection...");
    Serial.println("🔥 Place fire source near any flame sensor to activate");
    
    // Initialize state timer
    stateStartTime = millis();
}

// ---------------------------
// 🔄 MAIN LOOP: Non-blocking FSM Execution
// ---------------------------
void loop() {
    unsigned long currentTime = millis();
    
    // ✅ SENSOR SAMPLING (non-blocking)
    if (currentTime - lastSensorRead >= SENSOR_READ_INTERVAL) {
        lastSensorRead = currentTime;
        readAllSensors();
        updateConfidenceCounters();  // Update obstacle-avoidance weighting
    }
    
    // 📊 DEBUG OUTPUT (non-blocking)
    if (currentTime - lastDebugPrint >= DEBUG_PRINT_INTERVAL) {
        lastDebugPrint = currentTime;
        printStatus();  // Compact real-time status on Serial
    }
    
    // 🌊 SERVO SWEEP CONTROL (only during EXTINGUISHING)
    if (currentState == EXTINGUISHING && pumpActive) {
        handleServoSweep(currentTime);  // Oscillates servo for wider spray
    }
    
    // 🧠 STATE MACHINE EXECUTION
    handleStateMachine(currentTime);
}

// ---------------------------
// 📡 SENSOR READING FUNCTIONS
// ---------------------------

// Read all digital sensors and store boolean states
// Flame sensors: LOW = fire detected → flameX = true
// IR sensors: LOW = path clear → clearX = true
void readAllSensors() {
    // IR sensors (active HIGH for obstacle)
    leftClear   = (digitalRead(IR_SENSOR_LEFT)   == LOW);
    centerClear = (digitalRead(IR_SENSOR_CENTER) == LOW);
    rightClear  = (digitalRead(IR_SENSOR_RIGHT)  == LOW);
    
    // Flame sensors (active LOW for detection)
    flameLeft   = (digitalRead(FLAME_SENSOR_LEFT)   == LOW);
    flameCenter = (digitalRead(FLAME_SENSOR_CENTER) == LOW);
    flameRight  = (digitalRead(FLAME_SENSOR_RIGHT)  == LOW);
}

// Increment counters when path remains clear; reset when blocked
// Enables "confidence-based" turning to reduce oscillation in noisy environments
void updateConfidenceCounters() {
    leftClearCount  = leftClear  ? min(leftClearCount + 1, 255) : 0;
    rightClearCount = rightClear ? min(rightClearCount + 1, 255) : 0;
    // (Using `min()` prevents overflow, though unlikely at this scale)
}

// ---------------------------
// 🧠 STATE MACHINE LOGIC
// ---------------------------

// Dispatches current state handler based on `currentState`
void handleStateMachine(unsigned long currentTime) {
    switch (currentState) {
        case SEARCHING_FOR_FIRE:
            handleSearching();
            break;
            
        case NAVIGATING_TO_FIRE:
            handleNavigationToFire();
            break;
            
        // Transient motion states: auto-transition after fixed duration or condition
        case TURNING_LEFT:
        case TURNING_RIGHT:
            if (currentTime - stateStartTime >= TURN_DURATION) {
                // After turn, re-evaluate: is fire still visible?
                if (flameLeft || flameCenter || flameRight) {
                    changeState(NAVIGATING_TO_FIRE);
                } else {
                    changeState(SEARCHING_FOR_FIRE);
                }
            }
            break;
            
        case REVERSING:
            if (currentTime - stateStartTime >= REVERSE_DURATION) {
                // After backing up, reassess situation
                if (flameLeft || flameCenter || flameRight) {
                    changeState(NAVIGATING_TO_FIRE);
                } else {
                    changeState(SEARCHING_FOR_FIRE);
                }
            }
            break;
            
        case EXTINGUISHING:
            // Extinguishing ends when pump deactivates (timed)
            if (!pumpActive) {
                changeState(SEARCHING_FOR_FIRE);
            }
            break;
    }
}

// 🔍 State: SEARCHING_FOR_FIRE
// Robot remains stationary, periodically scanning for flame.
// Prioritizes center > left > right for initial response.
void handleSearching() {
    // Check for fire in priority order
    if (flameCenter) {
        changeState(NAVIGATING_TO_FIRE);
        return;
    } else if (flameLeft) {
        changeState(TURNING_LEFT);
        return;
    } else if (flameRight) {
        changeState(TURNING_RIGHT);
        return;
    }
    
    // No fire → remain stopped
    stopMotors();
}

// 🎯 State: NAVIGATING_TO_FIRE
// Actively steer toward fire while avoiding obstacles using predictive logic.
// Combines fire localization, path clearance, and weighted obstacle avoidance.
void handleNavigationToFire() {
    // ✅ P1: Fire centered & path clear → ENGAGE EXTINGUISHER
    if (flameCenter && centerClear) {
        changeState(EXTINGUISHING);
        return;
    }
    
    // ✅ P2: Fire on side only → rotate to center it
    if (flameLeft && !flameCenter) {
        changeState(TURNING_LEFT);
        return;
    }
    if (flameRight && !flameCenter) {
        changeState(TURNING_RIGHT);
        return;
    }
    
    // ✅ P3: Fire centered but blocked → detour intelligently
    if (flameCenter && !centerClear) {
        if (leftClear && !rightClear) {
            changeState(TURNING_LEFT);
            Serial.println("🔥 Fire ahead but blocked - going around left");
        } else if (rightClear && !leftClear) {
            changeState(TURNING_RIGHT);
            Serial.println("🔥 Fire ahead but blocked - going around right");
        } else if (leftClear && rightClear) {
            // Both sides open → bias based on confidence counters
            if (leftClearCount > rightClearCount + CONFIDENCE_THRESHOLD) {
                changeState(TURNING_LEFT);
            } else {
                changeState(TURNING_RIGHT);
            }
        } else {
            // No escape forward → back up!
            changeState(REVERSING);
        }
        return;
    }
    
    // ✅ P4: Fire on side, center path clear → move forward (align while advancing)
    if ((flameLeft || flameRight) && centerClear) {
        moveForward();
        return;
    }
    
    // ✅ P5: No immediate fire alignment → fall back to predictive obstacle avoidance
    handlePredictiveObstacleAvoidance();
}

// 🧭 Advanced obstacle avoidance with confidence weighting & fire-aware routing
// Designed to minimize collisions and oscillations in cluttered environments.
void handlePredictiveObstacleAvoidance() {
    // 🚨 P1: Fully blocked → REVERSE!
    if (!leftClear && !centerClear && !rightClear) {
        changeState(REVERSING);
        return;
    }
    
    // 🚦 P2: Center blocked, side(s) open → pre-emptive turn (before collision)
    if (!centerClear && (leftClear || rightClear)) {
        if (leftClear && !rightClear) {
            changeState(TURNING_LEFT);
        } else if (rightClear && !leftClear) {
            changeState(TURNING_RIGHT);
        } else if (leftClear && rightClear) {
            // Use confidence counters for stable decision-making
            if (leftClearCount > rightClearCount + CONFIDENCE_THRESHOLD) {
                changeState(TURNING_LEFT);
            } else if (rightClearCount > leftClearCount + CONFIDENCE_THRESHOLD) {
                changeState(TURNING_RIGHT);
            } else {
                // Tie-breaker: turn toward side where fire was last seen
                if (flameLeft) {
                    changeState(TURNING_LEFT);
                } else if (flameRight) {
                    changeState(TURNING_RIGHT);
                } else {
                    changeState(TURNING_RIGHT); // Fallback: right turn (arbitrary but deterministic)
                }
            }
        }
        return;
    }
    
    // ⚖️ P3: Center clear, asymmetric sides → continue forward
    // (Future enhancement: PWM-based differential steering for gentle correction)
    if (centerClear && !leftClear && rightClear) {
        moveForward();  // Favor right drift (if needed, e.g., with PWM control)
        return;
    }
    if (centerClear && !rightClear && leftClear) {
        moveForward();  // Favor left drift
        return;
    }
    
    // 🚪 P4: Narrow corridor (only center open) → proceed cautiously
    if (centerClear && !leftClear && !rightClear) {
        moveForward();
        return;
    }
    
    // ✅ P5: Favor forward progress when safe
    if (centerClear) {
        moveForward();
        return;
    }
    // (Remaining edge cases are covered by higher-priority checks above)
}

// 🌊 Servo sweep control during EXTINGUISHING: oscillates between 60°–120°
// Non-blocking: steps every SERVO_MOVE_INTERVAL ms for smooth motion
void handleServoSweep(unsigned long currentTime) {
    if (currentTime - servoMoveMillis >= SERVO_MOVE_INTERVAL) {
        servoMoveMillis = currentTime;
        
        if (servoSweepingRight) {
            servoPos += 5;
            if (servoPos >= 120) {
                servoSweepingRight = false;
            }
        } else {
            servoPos -= 5;
            if (servoPos <= 60) {
                servoSweepingRight = true;
            }
        }
        waterServo.write(servoPos);
    }

    // Auto-deactivate pump after PUMP_DURATION
    if (currentTime - pumpActivateMillis >= PUMP_DURATION) {
        deactivateWaterPump();
    }
}

// 🧾 Transition to a new state with entry actions and logging
// Centralized state changes ensure consistent timing and side effects.
void changeState(State newState) {
    currentState = newState;
    stateStartTime = millis();
    
    // Execute entry actions (motor control, pump/servo, logging)
    switch (newState) {
        case SEARCHING_FOR_FIRE:
            stopMotors();
            Serial.println(">>> STATE: 🔍 SEARCHING FOR FIRE (STANDBY)");
            break;
            
        case NAVIGATING_TO_FIRE:
            moveForward();
            Serial.println(">>> STATE: 🔥 NAVIGATING TO FIRE");
            break;
            
        case TURNING_LEFT:
            turnLeft();
            Serial.println(">>> STATE: ↩️ TURNING LEFT");
            break;
            
        case TURNING_RIGHT:
            turnRight();
            Serial.println(">>> STATE: ↪️ TURNING RIGHT");
            break;
            
        case REVERSING:
            moveBackward();
            Serial.println(">>> STATE: ⬇️ REVERSING (TRAPPED!)");
            break;
            
        case EXTINGUISHING:
            stopMotors();
            activateWaterPump();
            Serial.println(">>> STATE: 🚰 EXTINGUISHING FIRE!");
            break;
    }
}

// ---------------------------
// 🛠 MOTOR CONTROL PRIMITIVES
// ---------------------------

// Move both motors forward (differential drive)
void moveForward() {
    digitalWrite(MOTOR_L1, HIGH); digitalWrite(MOTOR_L2, LOW);
    digitalWrite(MOTOR_R1, HIGH); digitalWrite(MOTOR_R2, LOW);
}

// Move both motors backward
void moveBackward() {
    digitalWrite(MOTOR_L1, LOW);  digitalWrite(MOTOR_L2, HIGH);
    digitalWrite(MOTOR_R1, LOW);  digitalWrite(MOTOR_R2, HIGH);
}

// Pivot-turn left: left motor back, right motor forward
void turnLeft() {
    digitalWrite(MOTOR_L1, LOW);  digitalWrite(MOTOR_L2, HIGH);
    digitalWrite(MOTOR_R1, HIGH); digitalWrite(MOTOR_R2, LOW);
}

// Pivot-turn right: left motor forward, right motor back
void turnRight() {
    digitalWrite(MOTOR_L1, HIGH); digitalWrite(MOTOR_L2, LOW);
    digitalWrite(MOTOR_R1, LOW);  digitalWrite(MOTOR_R2, HIGH);
}

// Stop all motors (H-bridge braking)
void stopMotors() {
    digitalWrite(MOTOR_L1, LOW); digitalWrite(MOTOR_L2, LOW);
    digitalWrite(MOTOR_R1, LOW); digitalWrite(MOTOR_R2, LOW);
}

// ---------------------------
// 💧 WATER SYSTEM CONTROL
// ---------------------------

// Activate pump and initialize servo sweep
void activateWaterPump() {
    Serial.println("🚰 Activating Water Pump...");
    digitalWrite(RELAY_PIN, HIGH);  // Energize relay
    pumpActive = true;
    pumpActivateMillis = millis();  // Start timer

    // Initialize servo sweep from leftmost position
    servoPos = 60;
    servoSweepingRight = true;
    waterServo.write(servoPos);
}

// Safely deactivate pump and reset nozzle
void deactivateWaterPump() {
    if (pumpActive) {
        Serial.println("✅ Deactivating Water Pump...");
        digitalWrite(RELAY_PIN, LOW);
        pumpActive = false;
        waterServo.write(90);  // Return to neutral
    }
}

// ---------------------------
// 📈 DEBUGGING & DIAGNOSTICS
// ---------------------------

// Compact real-time status printout (optimized for Serial Monitor readability)
void printStatus() {
    // IR clearance status + confidence counters
    Serial.print("🔍 IR: L:");
    Serial.print(leftClear ? "✓" : "✗"); Serial.print("("); Serial.print(leftClearCount); Serial.print(")");
    Serial.print(" C:"); Serial.print(centerClear ? "✓" : "✗");
    Serial.print(" R:"); Serial.print(rightClear ? "✓" : "✗"); Serial.print("("); Serial.print(rightClearCount); Serial.print(") | ");

    // Flame detection status
    Serial.print("🔥 Flame: L:"); Serial.print(flameLeft ? "🔥" : "·");
    Serial.print(" C:"); Serial.print(flameCenter ? "🔥" : "·");
    Serial.print(" R:"); Serial.print(flameRight ? "🔥" : "·");
    Serial.print(" | ");

    // Current state (emoji-coded for quick visual parsing)
    switch (currentState) {
        case SEARCHING_FOR_FIRE: Serial.print("⏸️ STANDBY"); break;
        case NAVIGATING_TO_FIRE: Serial.print("🎯 NAV→FIRE"); break;
        case TURNING_LEFT:       Serial.print("↩️ LEFT"); break;
        case TURNING_RIGHT:      Serial.print("↪️ RIGHT"); break;
        case REVERSING:          Serial.print("⬇️ REVERSE"); break;
        case EXTINGUISHING:      Serial.print("💧 SPRAY"); break;
    }

    // Time spent in current state (for tuning TURN/REVERSE durations)
    Serial.print(" | "); Serial.print(millis() - stateStartTime); Serial.println("ms");
}