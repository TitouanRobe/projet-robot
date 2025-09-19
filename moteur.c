#include <msp430.h>


#define L_ODOM BIT0
#define L_MOTOR_DIR BIT1
#define L_MOTOR_PWM BIT2
#define aze  (100.0 / 174.0)
#define DISTG_VOULU (150.0)
#define TICSG_VOULU (DISTG_VOULU/aze)

#define RATIO (1)
#define BTN_S2 BIT3
#define VITESSEG (54)
#define VITESSED (50)

#define R_MOTOR_PWM BIT4
#define R_MOTOR_DIR BIT5
#define R_ODOM BIT3
#define DISTD_VOULU (150.0)
#define TICSD_VOULU (DISTD_VOULU/aze)

volatile int i;

#pragma vector = PORT1_VECTOR
__interrupt void port1_isr(void) {
  if ((P1IFG & BIT3) == BIT3) {
    // int pas = 50;
    // if (TA1CCR2 > TA1CCR0) {
    //   TA1CCR1 = 0;
    //   TA1CCR2 = 0;
    // } else {
    //   TA1CCR1 += pas;
    //   TA1CCR2 += pas;
    // }
    P1IFG &= ~(BTN_S2);
  }
}


volatile int l_cpt = 0;
volatile int r_cpt = 0;
#pragma vector = PORT2_VECTOR
__interrupt void port2_isr(void) {
  if ((P2IFG & L_ODOM) == L_ODOM){
    l_cpt++;
    P2IES ^= L_ODOM;
    P2IFG &= ~(L_ODOM);
    if (r_cpt>TICSG_VOULU){
      TA1CCR1 = 0;
      TA1CCR2 = 0;
    }
  }
  if ((P2IFG & R_ODOM) == R_ODOM){
    r_cpt++;
    P2IES ^= R_ODOM;
    P2IFG &= ~(R_ODOM);
    if (r_cpt>TICSG_VOULU){
      TA1CCR1 = 0;
      TA1CCR2 = 0;
    }
  }
}


void left_init(void) {
  P2OUT &= ~(L_MOTOR_DIR);

  P1DIR &= ~BTN_S2;

  P2DIR |= (L_MOTOR_PWM | L_MOTOR_DIR);

  P2SEL |= (L_MOTOR_PWM);
  P2SEL &= ~(L_MOTOR_DIR);
  P2SEL2 &= ~(L_MOTOR_PWM | L_MOTOR_DIR);
}


void left_timer(void) {
  TA1CTL |= TASSEL_2 | MC_1 |ID_0;  
  TA1CCTL1 |= OUTMOD_7;
  TA1CCR0 = 100;
  TA1CCR1 = VITESSEG;  
}


void left_odom(void){
  P2SEL &= ~(L_ODOM);
  P2SEL2 &= ~(L_ODOM);

  P2DIR &= ~(L_ODOM);

  P2IE |= L_ODOM;
  P2IES |= L_ODOM; //front descendant
  P2IFG &= ~(L_ODOM); // RAZ flag d’interruption
}

void left_wheel(void) {
  left_init();
  left_timer();
  left_odom();
}


void right_init(void) {
  P2OUT |= (R_MOTOR_DIR);

  P1DIR &= ~BTN_S2;

  P2DIR |= (R_MOTOR_PWM | R_MOTOR_DIR);

  P2SEL |= (R_MOTOR_PWM);
  P2SEL &= ~(R_MOTOR_DIR);
  P2SEL2 &= ~(R_MOTOR_PWM | R_MOTOR_DIR);
}


void right_timer(void) {
  TA1CCTL2 |= OUTMOD_7;
  TA1CCR2 = VITESSED;
}

void right_odom(void){
  P2SEL &= ~(R_ODOM);
  P2SEL2 &= ~(R_ODOM);

  P2DIR &= ~(R_ODOM);

  P2IE |= R_ODOM;
  P2IES |= R_ODOM; //front descendant
  P2IFG &= ~(R_ODOM); // RAZ flag d’interruption
}


void right_wheel(void) {
  right_init();
  right_timer();
  right_odom();
}


void bouton(void){
  P1SEL &= ~BTN_S2;
  P1SEL2 &= ~BTN_S2;  
  P1REN |= BTN_S2;
  P1OUT |= BTN_S2;

  P1IE |= BTN_S2;
  P1IES |= BTN_S2;

  P1IFG &= ~(BTN_S2);
}

int main(void) {
  WDTCTL = WDTPW | WDTHOLD;
  BCSCTL1 = CALBC1_1MHZ;
  DCOCTL = CALDCO_1MHZ;

  bouton();

  left_wheel();
  right_wheel();

  __enable_interrupt();

  while (1){

  }
}


