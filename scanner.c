#include <msp430.h>
#include "ADC.h"
#include "Afficheur.h"


#pragma vector = PORT1_VECTOR
__interrupt void port1_isr(void) {
  if ((P1IFG & BIT3) == BIT3) {
    int pas = 50;

    if (TA1CCR1>TA1CCR0){
      TA1CCR1 = 0;
      TA1CCR2 = 0;
    }else{
      TA1CCR1 += pas;
       TA1CCR2 += pas;
    }

  }
  P1IFG &= ~(BIT3);
}

void timer(void){

  P2DIR |= BIT2;            // On a aiguillé la sortie du timer sur le port 1.6 qui est une LED, d'où le clignotement de la LED
  P2DIR |= BIT4;

  P2SEL |= BIT2;            // selection fonction TA0.1
  P2SEL2 &= ~BIT2;          // selection fonction TA0.1

  P2SEL |= BIT4;            // selection fonction TA0.1
  P2SEL2 &= ~BIT4;          // selection fonction TA0.1

  P2SEL &= ~ BIT1;            // selection fonction TA0.1
  P2SEL2 &= ~BIT1;          // selection fonction TA0.1

  P2SEL &= ~ BIT5;            // selection fonction TA0.1
  P2SEL2 &= ~BIT5;          // selection fonction TA0.1

  P2DIR  |= BIT1;
  P2OUT|= BIT1;

  P2DIR  |= BIT5;
  P2OUT &=~ BIT5;

  P1SEL &= ~BIT3 | BIT2;            // selection fonction TA0.1
  P1SEL2 &= ~BIT3 | BIT2;

  P1DIR &= ~BIT2;

  P2SEL &= ~BIT7;            // selection fonction TA0.1
  P2SEL2 &= ~BIT7;

  TA1CCR0 = 1000;           // determine la periode du signal
  TA1CCR1 = 0;           // determine le rapport cyclique du signal
  TA1CCR2 = 0;           // determine le rapport cyclique du signal


  TA1CCTL1 |= OUTMOD_7;     // activation mode de sortie n°7
  TA1CCTL2 |= OUTMOD_7;     // activation mode de sortie n°7
  TA1CTL |= TASSEL_2 | MC_1| ID_0; // source SMCLK pour TimerA , mode comptage Up. Le changement de mode de comptage redemarre le compteur.1
  TA1CTL &= ~MC_0; // Arrêt du compteur
  TA1CTL |= MC_1| ID_2; // Redémarrage du compteur
}

int main(void)
{
  volatile unsigned int i;
  WDTCTL = WDTPW | WDTHOLD;
  BCSCTL1 = CALBC1_1MHZ;    // frequence d’horloge 1MHz
  DCOCTL = CALDCO_1MHZ;     // "

  P1DIR &= ~BIT3;
  P2DIR &= ~BIT7;

  timer();

  P1REN |= BIT3;
  P1OUT |= BIT3;

  P2REN |= BIT7;
  P2OUT |= BIT7;

  P1IE |= BIT3;
  P1IES |= BIT3;

  P2IE |= BIT7;
  P2IES |= BIT7;

  P1IFG &= ~(BIT3);
  P2IFG &= ~(BIT7);

  __enable_interrupt();

  ADC_init();
  Aff_Init();

  while (1)
  {
    ADC_Demarrer_conversion(2);

    int distance = convert_Hex_Dec(ADC_Lire_resultat());

    Aff_valeur(distance);

    for ( i=10000; i>0; i--);

  }
}