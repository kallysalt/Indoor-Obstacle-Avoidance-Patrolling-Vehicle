const Timer_A_UpModeConfig upConfig_0 =
{   TIMER_A_CLOCKSOURCE_SMCLK,
    TIMER_A_CLOCKSOURCE_DIVIDER_64,
    552,
    TIMER_A_TAIE_INTERRUPT_DISABLE,
    TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,
    TIMER_A_DO_CLEAR
};

const Timer_A_ContinuousModeConfig continuousModeConfig =
{
 TIMER_A_CLOCKSOURCE_SMCLK, // SMCLK Clock Source
 TIMER_A_CLOCKSOURCE_DIVIDER_1, // SMCLK/1 = 3MHz
 TIMER_A_TAIE_INTERRUPT_DISABLE, // Disable Timer ISR
 TIMER_A_SKIP_CLEAR // Skup Clear Counter
};

/* Timer_A Capture Mode Configuration Parameter */
const Timer_A_CaptureModeConfig captureModeConfig =
{
 TIMER_A_CAPTURECOMPARE_REGISTER_1, // CC Register 2
 TIMER_A_CAPTUREMODE_RISING_AND_FALLING_EDGE, // Rising Edge and falling
 TIMER_A_CAPTURE_INPUTSELECT_CCIxA, // CCIxA Input Select
 TIMER_A_CAPTURE_SYNCHRONOUS, // Synchronized Capture
 TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE, // Enable interrupt
 TIMER_A_OUTPUTMODE_OUTBITVALUE // Output bit value
};

static void Delay(uint32_t loop);

#define PORT1  GPIO_PORT_P1
#define PORT2  GPIO_PORT_P2
#define S1  GPIO_PIN1
#define S2  GPIO_PIN4
#define PORT5  GPIO_PORT_P5
#define PORT6  GPIO_PORT_P6
#define PIN4  GPIO_PIN4
#define PIN5  GPIO_PIN5
#define PIN6  GPIO_PIN6
#define PIN7  GPIO_PIN7

volatile float dutyCycle = 0;
volatile bool status;
bool s1_pressed;
bool s2_pressed;
int i;
int j;
int meas1 = 0;
int meas2 = 0;
int time1 = 0;
volatile int distance1 = 0;

volatile int y = 0; //measured distance
volatile float u = 0; //controller output
float Kp = 0.1;
float Ki = 0;
float Kd = 0.0001;
volatile int error;
volatile int previous_error = 0;
volatile float error_integral;
volatile float error_derivative;
volatile bool forward=0;
volatile bool backward=0;
volatile bool turn=0;
// ???
int base = 3000*8;
float dt = 0.1;
float upper = 0.5; //pwm ratio
float allowed_error = 2;
int r = 20;

/*
PID code in TA0 interrupt:
y = distance1;
error = r - y;
error_integral += error*dt;
error_derivative = (error - previous_error)/dt;
u = Kp*error + Kd*error_derivative; // u is treated as a pwm ratio
// + Ki*error_integral
if(u > upper) u = upper;
else if(u < -upper) u = -upper;
previous_error = error;
dutyCycle = abs(u)*base;
// Set threshold
if(abs(error) < allowed_error){
    dutyCycle = 0;
    error_integral = 0;
}

PID code in TA1 interrupt:
if(u < 0){
    // backward;
}
else{
    // forward;
}
// setPWMVal

*/





