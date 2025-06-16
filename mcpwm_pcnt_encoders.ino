//#include <PulseCounter.h>
//#include <RotaryEncoderPCNT.h>
#include <encoderEncoderPCNT.h>
#include <Bounce2.h>
#include <MCPWM_PWM_ENCODER.h>

extern "C"
{
    #include "driver/mcpwm_prelude.h"
}

// ===== L298 Control Pins =====
static const int IN1_pin = 5;
static const int IN2_pin = 4;
static const int IN3_pin = 2;
static const int IN4_pin = 1;

// ===== quadrature encoder & Button =====
static const int        quadrature_encoder_pin_A    = 10;
static const int        quadrature_encoder_pin_B    = 9;
static const int        motor_encoder_pin_A         = 15;
static const int        motor_encoder_pin_B         = 16;
static const int        quadrature_button_pin       = 8;
static const int        rgb_pin                     = 21;

static const int        pwm_pin_A                   = 6;
static const int        pwm_pin_B                   = 7;
static const uint32_t   pwm_hertz                   = 21000;
static const uint8_t    pwm_number_of_bits          = 10;
static const int32_t    pwm_max_counts              = (1u << pwm_number_of_bits);

// pin_map[motor][0] → INx_pin (drive pin A)  
// pin_map[motor][1] → INx_pin (drive pin B)
static const int pin_map[2][2] =
{
    { IN1_pin, IN2_pin },
    { IN3_pin, IN4_pin }
};

// dir_map[direction][0] → state for pin_map[][0]  
// dir_map[direction][1] → state for pin_map[][1]
static const int dir_map[3][2] =
{
    { LOW,  LOW  },  // STOP
    { LOW,  HIGH },  // FORWARD
    { HIGH, LOW  }   // BACKWARD
};

enum Motor     { MOTOR_A = 0, MOTOR_B = 1 };
enum Direction { STOP    = 0, FORWARD = 1, BACKWARD = 2 };

// ──────────────── tunable dead-band & PI params ────────────────
static constexpr int32_t    MOTOR_STARTING  = 5;
static constexpr int32_t    INNER_DEAD_BAND = 10;    // ±10 ticks → truly zero
static constexpr int32_t    KICK_DEAD_BAND  = 500;   // 11…599 → jump to 600
static constexpr float      Kp              = 0.01f; // proportional gain
static constexpr float      Ki              = 0.005f;// integral gain

static const uint32_t       PI_SAMPLE_MS    = 100;   // ms between PI updates
static const float          PI_SAMPLE_S     = PI_SAMPLE_MS * 0.001f;  // 100 ms → 0.100 s
static const float          PI_SAMPLE_HZ    = 1.0f / PI_SAMPLE_S; // = 10.0 (Hz)

// ──────────────── state for PI loop ────────────────
static float    pi_integral = 0;
static uint32_t lastPI      = 0;
static uint32_t now;
static uint32_t dt;

bool        directionFlag;
int         oldPosition;
int         position;
uint32_t    absPosition;

encoderEncoderPCNT encoder(quadrature_encoder_pin_A, quadrature_encoder_pin_B, motor_encoder_pin_A, motor_encoder_pin_B);
MCPWM_PWM_ENCODER pwm(pwm_pin_A, pwm_pin_B, pwm_hertz, pwm_number_of_bits);
Bounce2::Button button = Bounce2::Button();

// —— L298 helpers ——
void setMotorDirection(uint8_t motor, uint8_t direction);
void setSpoolDirection();
void setUnspoolDirection();
bool getDirectionFlag();
void calibrateSpeed();

// —— Setup ——
void setup()
{
    // 1) MCPWM setup:
    pwm.initPWM(0);

    // 3) Quadrature encoder & button setup:
    button.attach(quadrature_button_pin, INPUT_PULLDOWN);
    button.interval(5);
    button.setPressedState(HIGH);

    // 5) L298 setup:
    pinMode(IN1_pin, OUTPUT);
    pinMode(IN2_pin, OUTPUT);
    pinMode(IN3_pin, OUTPUT);
    pinMode(IN4_pin, OUTPUT);

    // 6) Establish correct direction
    setSpoolDirection();
    pwm.setDutyCycle(pwm.cmprA, 0);
    pwm.setDutyCycle(pwm.cmprB, 0);

    // 7) Misc:
    //rgbLedWrite(rgb_pin, 100, 0, 100);
    oldPosition = encoder.position();
    lastPI = millis();
    //Serial.begin(115200);
    //delay(5000);
    //calibrateSpeed();
}

void loop()
{
    button.update();

    if (button.pressed())
    {
        encoder.zero();
        position = 0;
        oldPosition = 0;
        pi_integral    = 0.0f;
        lastPI         = millis();
        encoder.getCountA(true);
        encoder.getCountB(true);
        pwm.setDutyCycle(pwm.cmprA, 0);
        pwm.setDutyCycle(pwm.cmprB, 0);
        printf("== ZERO ==\n");
    }

    //  encoder block:
    position = encoder.position();
    if (position != oldPosition)
    {
        // set motor direction if it's not appropriately set already:
        if(position < -MOTOR_STARTING && position > -INNER_DEAD_BAND)
        {
            if(getDirectionFlag()) { setUnspoolDirection(); printf("set to Unspool direction \n"); }
        }
        else if(position > MOTOR_STARTING && position < INNER_DEAD_BAND)
        {
            if(!getDirectionFlag()) { setSpoolDirection(); printf("set to Spool direction \n"); }
        }

        // manage the dead-zones to enable scrolling up & down at will:
        if(position < oldPosition) // encoder rotating counter-clock wise
        {
            if(position < KICK_DEAD_BAND && position > 0)
                if(position > INNER_DEAD_BAND){ encoder.setPosition(INNER_DEAD_BAND); }
            if(position < -INNER_DEAD_BAND)
                if(position > -KICK_DEAD_BAND){ encoder.setPosition(-KICK_DEAD_BAND); }
        }
        if(position > oldPosition) // encoder rotating clock wise
        {
            if(position > INNER_DEAD_BAND)
                if(position < KICK_DEAD_BAND){ encoder.setPosition(KICK_DEAD_BAND); }
            if(position > -KICK_DEAD_BAND && position < 0)
                if(position < -INNER_DEAD_BAND){ encoder.setPosition(-INNER_DEAD_BAND); }
        }
        if(position > pwm_max_counts)   // clamp position value below the value equals 100% duty cycle
            encoder.setPosition(pwm_max_counts);
        if(position < -pwm_max_counts)
            encoder.setPosition(-pwm_max_counts);

        // make sure abs(position) is not more than the pwm resolution accepts:
        //absPosition = abs(position) < pwm.getResolution() ? abs(position) : pwm.getResolution();
        absPosition = abs(position);
        // Drive motor A:
        pwm.setDutyCycle(pwm.cmprA, absPosition);
        pwm.setDutyCycle(pwm.cmprB, absPosition);
        //printf("encoder value = %d\n", position);
        //printf("encoder value = %d pwm.cmprA = %d\n", position, absPosition);
        oldPosition = position;
    }


    
    // 5) every PI_SAMPLE_MS, run PI for Motor B
    now = millis();
    dt  = now - lastPI;
    if (dt >= PI_SAMPLE_MS)
    {

        int cntA = encoder.getCountA(true);
        int cntB = encoder.getCountB(true);
        printf("(current encoder position = %d): cntA = %d cntB =%d\n", position, cntA, cntB);
        /*
        float vA = encoder.getCountA(true)  * (1000.0f / dt);  // ticks/sec
        float vB = encoder.getCountB(true)  * (1000.0f / dt);

        float err = vA - vB;
        pi_integral += err * PI_SAMPLE_S; //pi_integral += err * (dt * 0.001f);
        float adj = Kp * err + Ki * pi_integral;

        int32_t dutyB = (int32_t)absPosition + (int32_t)round(adj);
        dutyB = constrain(dutyB, 0, (int32_t)pwm.getResolution());
        pwm.setDutyCycle(pwm.cmprB, dutyB);

        lastPI = now;
        printf("position=%4d  dutycycleA=%4u  vA=%.0f  vB=%.0f  err=%+.0f  adj=%+.0f\n", position, absPosition, vA, vB, err, adj);
        */
    }
    
}

void setMotorDirection(uint8_t motor, uint8_t direction)
{
    // guard against out-of-range
    if (motor > MOTOR_B || direction > BACKWARD)
    {
        return;
    }

    int p0 = pin_map[motor][0];
    int p1 = pin_map[motor][1];

    digitalWrite(p0, dir_map[direction][0]);
    digitalWrite(p1, dir_map[direction][1]);
}

void setSpoolDirection()
{
    directionFlag = true;
    setMotorDirection(MOTOR_A, FORWARD);
    setMotorDirection(MOTOR_B, BACKWARD);
}

void setUnspoolDirection()
{
    directionFlag = false;
    setMotorDirection(MOTOR_A, BACKWARD);
    setMotorDirection(MOTOR_B, FORWARD);
}

bool getDirectionFlag()
{
    return directionFlag;
}

void calibrateSpeed()
{
    constexpr uint32_t CAL_MS   = 5000;// this is simply to give me time to switch to the serial monitor
    const uint32_t  fullDuty   = pwm.getResolution();

    // 1) clear any leftover counts
    encoder.getCountA(true);
    encoder.getCountB(true);

    // 2) spin A at full, wait exactly CAL_MS
    pwm.setDutyCycle(pwm.cmprA, fullDuty);
    delay(CAL_MS);
    int32_t cntA = encoder.getCountA(true);
    pwm.setDutyCycle(pwm.cmprA, 0);
    float vA_full = cntA * (1000.0f / float(CAL_MS));       // ticks/sec

    // 3) spin B at full, wait exactly CAL_MS
    pwm.setDutyCycle(pwm.cmprB, fullDuty);
    delay(CAL_MS);
    int32_t cntB = encoder.getCountB(true);
    pwm.setDutyCycle(pwm.cmprB, 0);
    float vB_full = cntB * (1000.0f / float(CAL_MS));       // ticks/sec

    // 4) compute your common max and scales
    float vmax_common = min(vA_full, vB_full);
    float scaleA      = (vA_full > 0) ? vmax_common / vA_full : 1.0f;
    float scaleB      = (vB_full > 0) ? vmax_common / vB_full : 1.0f;

    printf("vA_full=%.0f ticks/s, vB_full=%.0f ticks/s\nscaleA=%.3f scaleB=%.3f\n",
           vA_full, vB_full, scaleA, scaleB);
}
