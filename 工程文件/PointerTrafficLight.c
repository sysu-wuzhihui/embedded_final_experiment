//本程序综合了FSM，A/D转换，边缘触发中断
//在带有紧急信号灯的十字路口灯的基础上，进行了一下两点改进：一，将东与北两个方向的车流量量化，以此来给定某方向车辆通行时间，
//车流量大的，通行时间变长，此处用到了模数转换；二，加入了关闭交通灯系统功能，在任何情况下，按下关闭系统的开关，将导致所有灯熄灭，
//松开关闭系统的开关，系统继续正常运行，此处用到了边缘触发的中断

#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "inc/tm4c123gh6pm.h"

#define LIGHT                   (*((volatile uint32_t *)0x400051FC))
#define GPIO_PORTB_OUT          (*((volatile uint32_t *)0x400051FC)) // bits 6-0
#define GPIO_PORTE_IN           (*((volatile uint32_t *)0x4002401C)) // bits 2-0
#define SENSOR                  (*((volatile uint32_t *)0x4002401C))

struct State {
  uint32_t Out;            // 7-bit output
  uint32_t Time;           // 10 ms
  const struct State *Next[8];};// depends on 3-bit input
typedef const struct State STyp;
	
#define goN   &FSM[0]
#define waitN &FSM[1]
#define goE   &FSM[2]
#define waitE &FSM[3]
#define Emer  &FSM[5]		//FSM[5],FSM[6],FSM[7] are all Emer
	
STyp FSM[8]={
 {0x21,300,{goN,waitN,goN,waitN, Emer, Emer, Emer,Emer}},
 {0x22, 50,{goE,goE,goE,goE, Emer, Emer, Emer,Emer}},
 {0x0C,300,{goE,goE,waitE,waitE, Emer, Emer, Emer,Emer}},
 {0x14, 50,{goN,goN,goN,goN, Emer, Emer, Emer,Emer}},	
 {0x40, 300, {goN,goN,goN,goN, Emer, Emer, Emer,Emer}},
 {0x40, 300, {goN,goN,goN,goN, Emer, Emer, Emer,Emer}},
 {0x40, 300, {goN,goN,goN,goN, Emer, Emer, Emer,Emer}},
 {0x40, 300, {goN,goN,goN,goN, Emer, Emer, Emer,Emer}},
};

volatile unsigned long ADCvalue;
int time;   //state time
int stop = 0;   //1 means stop the system

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

//边缘触发初始化，上下边缘均触发中断
void EdgeCounter_Init(void){                          
  SYSCTL_RCGCGPIO_R |= 0x00000020; // (a) activate clock for port F
	while((SYSCTL_PRGPIO_R&0x0020) == 0){};   //wait till activate port F 
  //FallingEdges = 0;             // (b) initialize counter
  GPIO_PORTF_DIR_R &= ~0x10;    // (c) make PF4 in (built-in button)
  GPIO_PORTF_AFSEL_R &= ~0x10;  //     disable alt funct on PF4
  GPIO_PORTF_DEN_R |= 0x10;     //     enable digital I/O on PF4   
  GPIO_PORTF_PCTL_R &= ~0x000F0000; // configure PF4 as GPIO
  GPIO_PORTF_AMSEL_R = 0;       //     disable analog functionality on PF
  GPIO_PORTF_PUR_R |= 0x10;     //     enable weak pull-up on PF4
  GPIO_PORTF_IS_R &= ~0x10;     // (d) PF4 is edge-sensitive
  GPIO_PORTF_IBE_R |= 0x10;    //     PF4 is  both edges
  GPIO_PORTF_IEV_R &= ~0x10;    //     PF4 falling edge event
  GPIO_PORTF_ICR_R = 0x10;      // (e) clear flag4
  GPIO_PORTF_IM_R |= 0x10;      // (f) arm interrupt on PF4 *** No IME bit as mentioned in Book ***
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // (g) priority 5
  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
  EnableInterrupts();           // (i) Clears the I bit
}

//中断处理函数,开启与关闭系统切换
void GPIOPortF_Handler(void){
  GPIO_PORTF_ICR_R = 0x10;      // acknowledge flag4
	stop ^= 1;   //stop or start the system
	
	if (stop)
		LIGHT = 0x00;
}

// This initialization function sets up the ADC according to the
// following parameters.  Any parameters not explicitly listed
// below are not modified:
// Max sample rate: <=125,000 samples/second
// Sequencer 0 priority: 1st (highest)
// Sequencer 1 priority: 2nd
// Sequencer 2 priority: 3rd
// Sequencer 3 priority: 4th (lowest)
// SS3 triggering event: software trigger
// SS3 1st sample source: Ain9 (PE4)
// SS3 interrupts: flag set on completion but no interrupt requested
void ADC0_InitSWTriggerSeq3_Ch1(void){ volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000010;   // 1) activate clock for Port E
  delay = SYSCTL_RCGC2_R;         //    allow time for clock to stabilize
  GPIO_PORTE_DIR_R &= ~0x10;      // 2) make PE4 input
  GPIO_PORTE_AFSEL_R |= 0x10;     // 3) enable alternate function on PE4
  GPIO_PORTE_DEN_R &= ~0x10;      // 4) disable digital I/O on PE4
  GPIO_PORTE_AMSEL_R |= 0x10;     // 5) enable analog function on PE4
  SYSCTL_RCGC0_R |= 0x00010000;   // 6) activate ADC0 
	//SYSCTL_RCGCADC_R = 0x00000000;  
  delay = SYSCTL_RCGC2_R;         
  SYSCTL_RCGC0_R &= ~0x00000300;  // 7) configure for 125K 
  ADC0_SSPRI_R = 0x0123;          // 8) Sequencer 3 is highest priority
  ADC0_ACTSS_R &= ~0x0008;        // 9) disable sample sequencer 3
  ADC0_EMUX_R &= ~0xF000;         // 10) seq3 is software trigger
  ADC0_SSMUX3_R = (ADC0_SSMUX3_R&0xFFFFFFF0)+9; // 11) channel Ain9 (PE4)
  ADC0_SSCTL3_R = 0x0006;         // 12) no TS0 D0, yes IE0 END0
  ADC0_ACTSS_R |= 0x0008;         // 13) enable sample sequencer 3
}

//------------ADC0_InSeq3------------
// Busy-wait Analog to digital conversion
// Input: none
// Output: 12-bit result of ADC conversion
unsigned long ADC0_InSeq3(void){  
	unsigned long result;
  ADC0_PSSI_R = 0x0008;            // 1) initiate SS3
  while((ADC0_RIS_R&0x08)==0){};   // 2) wait for conversion done
  result = ADC0_SSFIFO3_R&0xFFF;   // 3) read result
  ADC0_ISC_R = 0x0008;             // 4) acknowledge completion
  return result;
}
   
 
int main(void){
	
  STyp *Pt;  // state pointer
  uint32_t Input;
  volatile uint32_t delay;
  PLL_Init();                  // configure for 50 MHz clock
  SysTick_Init();              // initialize SysTick timer
	ADC0_InitSWTriggerSeq3_Ch1();   // ADC0 initialization PE4/AIN9
	EdgeCounter_Init();           // initialize GPIO Port F interrupt，to stop or start the system
	
  // activate port B and port E
  SYSCTL_RCGCGPIO_R |= 0x12;
  delay = SYSCTL_RCGCGPIO_R;
  GPIO_PORTB_DIR_R |= 0x7F;    // make PB6-0 out
  GPIO_PORTB_AFSEL_R &= ~0x7F; // disable alt func on PB6-0
  GPIO_PORTB_DEN_R |= 0x7F;    // enable digital I/O on PB6-0
                               // configure PB6-0 as GPIO
  GPIO_PORTB_PCTL_R &= ~0x0FFFFFFF;
  GPIO_PORTB_AMSEL_R &= ~0x7F; // disable analog functionality on PB6-0
  GPIO_PORTE_DIR_R &= ~0x07;   // make PE2-0 in
  GPIO_PORTE_AFSEL_R &= ~0x07; // disable alt func on PE2-0
  GPIO_PORTE_DEN_R |= 0x07;    // enable digital I/O on PE2-0
                               // configure PE2-0 as GPIO
  GPIO_PORTE_PCTL_R = (GPIO_PORTE_PCTL_R&0xFFFFF000)+0x00000000;
  GPIO_PORTE_AMSEL_R &= ~0x07; // disable analog functionality on PE2-0
	
  Pt = goN;                    // initial state: Green north; Red east
	
  while(1){
		//stop时，灯一定全灭（中断处理程序也会熄灭灯），判断状态停留时间，ADC转换，输入传感器继续工作，一旦开启系统（松开PF4），直接在当前的传感器数值下运行
		if (stop)
			LIGHT = 0x00;
		else
			LIGHT = Pt->Out;           // set lights to current state's Out value
		
		ADCvalue = ADC0_InSeq3();  //get ADCvalue from traffic
		time = Pt==goN || Pt==goE? (int)(Pt->Time * (float)ADCvalue / 1000) : Pt->Time;   //the state time with ADCvalue on goN or goE
    SysTick_Wait10ms(time);// wait 10 ms * current state's time
    Input = SENSOR;            // get new input from car detectors
    Pt = Pt->Next[Input];      // transition to next state
  }
}

