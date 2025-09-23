#include <msp430.h>
#include "ADC.h"
#include "Afficheur.h"
#include <stdio.h>
#include <stdlib.h>

#include <msp430.h>


#define L_ODOM BIT0
#define L_MOTOR_DIR BIT1
#define L_MOTOR_PWM BIT2
#define aze  (100.0 / 174.0)
#define DISTG_VOULU (100.0) // 1.5m = 260.0
#define TICSG_VOULU (DISTG_VOULU/aze)

#define RATIO (1)
#define VITESSEG (43)
#define VITESSED (40)

#define R_MOTOR_PWM BIT4
#define R_MOTOR_DIR BIT5
#define R_ODOM BIT3
#define DISTD_VOULU (100.0) // 1.5m = 260.0
#define TICSD_VOULU (DISTD_VOULU/aze)

volatile int i;
volatile int l_cpt = 0;
volatile int r_cpt = 0;
volatile int ticsg_voulu = TICSG_VOULU;
volatile int n_enchainement = 5;

volatile int etat = 0;

void faire_huit(void)
{
    // boucle gauche
    tourner_gauche_chore();
    __delay_cycles(300000);

    // boucle droite
    tourner_droite_chore();
    __delay_cycles(300000);

    // répète pour dessiner un huit
    tourner_gauche_chore();
    __delay_cycles(300000);

    tourner_droite_chore();
    __delay_cycles(300000);

    TA1CCR1 = 0;
    TA1CCR2 = 0;

    P2OUT ^=  R_MOTOR_DIR;   // roue droite en avant
    P2OUT ^= L_MOTOR_DIR;  // roue gauche en arrière
}

void faire_huit_inv(void)
{
    // boucle gauche
    tourner_droite_chore();
    __delay_cycles(300000);

    // boucle droite
    tourner_gauche_chore();
    __delay_cycles(300000);

    // répète pour dessiner un huit
    tourner_droite_chore();
    __delay_cycles(300000);

    tourner_gauche_chore();
    __delay_cycles(300000);

    TA1CCR1 = 0;
    TA1CCR2 = 0;

    P2OUT ^=  R_MOTOR_DIR;   // roue droite en avant
    P2OUT ^= L_MOTOR_DIR;  // roue gauche en arrière
}
void faire_toupie(void)
{
    TA1CCR1 = VITESSEG;
    __delay_cycles(1600000); // durée de la rotation
}

void tourner_gauche_chore(void)
{
    TA1CCR1 = 0;         // roue gauche arrêtée
    TA1CCR2 = VITESSED;  // roue droite avance
    __delay_cycles(500000);
}


void tourner_droite_chore(void)
{
    TA1CCR1 = VITESSEG;  // roue gauche avance
    TA1CCR2 = 0;         // roue droite arrêtée
    __delay_cycles(500000); // durée du virage
}

void avancer(void)
{
  TA1CCR1 = VITESSEG;
  TA1CCR2 = VITESSED;
  __delay_cycles(500000); // durée du virage
}

void arreter(void)
{
  TA1CCR1 = 0;
  TA1CCR2 = 0;
  __delay_cycles(1000000); // durée du virage
}

void reculer(void)
{
    P2OUT ^=  R_MOTOR_DIR;   // roue droite en avant
    P2OUT ^= L_MOTOR_DIR;  // roue gauche en arrière
    TA1CCR1 = VITESSEG;
    TA1CCR2 = VITESSED;

  __delay_cycles(1000000); // durée du virage
}




#pragma vector = PORT2_VECTOR
 __interrupt void port2_isr(void) {
  // 1.5

  if ((P2IFG & L_ODOM) == L_ODOM)
  {
    l_cpt++;
    P2IES ^= L_ODOM;
    P2IFG &= ~(L_ODOM);
    if (l_cpt>ticsg_voulu){
      TA1CCR1 = 0; // roue gauche
      TA1CCR2 = 0; // roue droite
    }
  }

  if ((P2IFG & R_ODOM) == R_ODOM)
  {
    r_cpt++;
    P2IES ^= R_ODOM;
    P2IFG &= ~(R_ODOM);
    if (r_cpt>TICSG_VOULU){
      TA1CCR1 = 0; // roue gauche
      TA1CCR2 = 0; // roue droite
    }
  }

  if(TA1CCR1 == 0 && TA1CCR2 == 0)
  {
     avancer();

    reculer();

    tourner_gauche_chore();

     avancer();

    __delay_cycles(50000);

     arreter();

    faire_huit();

    tourner_droite_chore();
     faire_huit();

    faire_toupie();
    faire_toupie();

    reculer();

    __delay_cycles(50000);

    faire_huit();
    faire_huit_inv();
    faire_huit();
    faire_huit_inv();

     avancer();

    reculer();

     tourner_gauche_chore();

     avancer();

    __delay_cycles(50000);

    arreter();

    faire_huit_inv();
    faire_huit_inv();

    faire_huit();
    faire_huit();

    tourner_droite_chore();

    faire_toupie();
    faire_toupie();

    reculer();

    etat = 1;

     TA1CCR1 = 0;
    TA1CCR2 = 0;

  }
}

void left_init(void) {
  P2OUT &= ~(L_MOTOR_DIR);
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


void led_setup()
{

 // led rouge
  P1SEL &= ~ BIT6;
  P1SEL2 &= ~BIT6;
  P1DIR |= BIT6;

  P1OUT &=~ BIT6;
}


int main(void)
{
  volatile unsigned int i;
  volatile unsigned int distance;
  volatile unsigned int luminosite;
  WDTCTL = WDTPW | WDTHOLD;
  BCSCTL1 = CALBC1_1MHZ;    // frequence d’horloge 1MHz
  DCOCTL = CALDCO_1MHZ;     // "


  led_setup();
  left_wheel();
  right_wheel();


  ADC_init();
  Aff_Init();

  __enable_interrupt();

  while (etat !=1)
  {
      ADC_Demarrer_conversion(2);

      luminosite = ADC_Lire_resultat();

      if(luminosite > 692)
      {
          P1OUT &=~ BIT6;
      }
      else
      {
          P1OUT |= BIT6;
      }

      __delay_cycles(200);
    }
}