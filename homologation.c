// -------------------------- DEFINITION DES BIBLIOTHEQUES, MACROS, VARIABLES --------------------------

#include <msp430.h>
#include "ADC.h"
#include "Afficheur.h"
#include <stdio.h>
 
#define L_ODOM BIT0
#define L_MOTOR_DIR BIT1
#define L_MOTOR_PWM BIT2
#define aze  (100.0 / 174.0)
#define DISTG_VOULU (130.0)
#define TICSG_VOULU (DISTG_VOULU/aze)
 
#define RATIO (1)
#define BTN_S2 BIT3
#define VITESSEG (85)
#define VITESSED (84)
 
#define R_MOTOR_PWM BIT4
#define R_MOTOR_DIR BIT5
#define R_ODOM BIT3
#define DISTD_VOULU (130.0)
#define TICSD_VOULU (DISTD_VOULU/aze)


 
volatile int i;
 
volatile int l_cpt = 0;
volatile int r_cpt = 0;
volatile int ticsg_voulu = TICSG_VOULU;
volatile unsigned int distance;
volatile unsigned int luminosite;

// -------------------------- TABLE DE TRANSITIONS --------------------------

#define NB_ETATS (3)
#define NB_EVTS (3)

typedef enum {EVT_OBSTACLE, EVT_PAS_OBSTACLE, EVT_DESTINATION} Evenement;
 
typedef enum {ETAT_AVANCER, ETAT_ARRET, ETAT_ARRET_COMPLET} Etat;
 
typedef void (*Action)(void);
 
typedef struct
{    
  Etat nouvel_etat;
  Action action;
} Transition;

// -------------------------- INTERRUPTION CALCUL DE LA DISTANCE --------------------------

#pragma vector = PORT2_VECTOR
__interrupt void port2_isr(void) {

  // déclenchement front montant/descendant roue gauche
  if ((P2IFG & L_ODOM) == L_ODOM){
    // compteur de tics roue gauche
    l_cpt++;
    // changement du front
    P2IES ^= L_ODOM;
    P2IFG &= ~(L_ODOM);
  }

  // déclenchement front montant/descendant roue droite
  if ((P2IFG & R_ODOM) == R_ODOM){
    // compteur de tics roue droite
    r_cpt++;
    // changement du front
    P2IES ^= R_ODOM;
    P2IFG &= ~(R_ODOM);
  }
}

// -------------------------- FONCTIONS --------------------------


void capteur_setup()
{
  // BIT corespondant au capteur
  P1DIR &= ~BIT1;               
  P1SEL |= BIT1;                
  P1SEL2 &= ~BIT1;
 
 // buzzer 
  // P1SEL &= ~ BIT6;              
  // P1SEL2 &= ~BIT6;
  // P1DIR |= BIT6;
 
  // P1OUT &=~ BIT6;            
} 

void lecture_event(int distance, Evenement *event)
{  
    // si capteur detecte obstace à 7 cm
    if (distance > 300)
    {
        *event = EVT_OBSTACLE;      
    }
    else
    {
      // si distance parcouru pour les 2 roues
      if(l_cpt>ticsg_voulu && r_cpt>ticsg_voulu){
        
        // complétion de l'objectif on passe à l'etat event destination
        *event = EVT_DESTINATION;

      }else{ // sinon on est dans le cas où pas d'obstacle
          *event = EVT_PAS_OBSTACLE;
      }
      
    }
}

// action pour etat avancer
void action_avancer()
{
   TA1CCR1 = VITESSEG;        
   TA1CCR2 = VITESSED;  
}

// action pour etat arreté
void action_arreter()
{
   TA1CCR1 = 0;          
   TA1CCR2 = 0;    
    __delay_cycles(20);
}

// action pour etat arrêt complet
void action_arret_complet()
{
      TA1CCR1 = 0;
      TA1CCR2 = 0;
}


// gestion du capteur de lumière
void update_light(int luminosite){
    if(luminosite > 692)
      {
          P1OUT &=~ BIT6;  
      }
      else
      {
          P1OUT |= BIT6;
      }
}

// -------------------------- RELATIF AUX MOTEURS --------------------------

// initialisation de la roue gauche
void left_wheel(void) {
  P2OUT &= ~(L_MOTOR_DIR);
 
  P1DIR &= ~BTN_S2;
 
  P2DIR |= (L_MOTOR_PWM | L_MOTOR_DIR);
 
  P2SEL |= (L_MOTOR_PWM);
  P2SEL &= ~(L_MOTOR_DIR);
  P2SEL2 &= ~(L_MOTOR_PWM | L_MOTOR_DIR);

  // timer
  TA1CTL |= TASSEL_2 | MC_1 |ID_0;  
  TA1CCTL1 |= OUTMOD_7;
  TA1CCR0 = 100;

  // Odomètre
  P2SEL &= ~(L_ODOM);
  P2SEL2 &= ~(L_ODOM);
 
  P2DIR &= ~(L_ODOM);
 
  P2IE |= L_ODOM;
  P2IES |= L_ODOM; //front descendant
  P2IFG &= ~(L_ODOM); // RAZ flag d’interruption
}

 
 // initialisation de la roue droite
void right_wheel(void) {
  P2OUT |= (R_MOTOR_DIR);
 
  P1DIR &= ~BTN_S2;
 
  P2DIR |= (R_MOTOR_PWM | R_MOTOR_DIR);
 
  P2SEL |= (R_MOTOR_PWM);
  P2SEL &= ~(R_MOTOR_DIR);
  P2SEL2 &= ~(R_MOTOR_PWM | R_MOTOR_DIR);

  // timer 
  TA1CCTL2 |= OUTMOD_7;

  // Odomètre
  P2SEL &= ~(R_ODOM);
  P2SEL2 &= ~(R_ODOM);
 
  P2DIR &= ~(R_ODOM);
 
  P2IE |= R_ODOM;
  P2IES |= R_ODOM; //front descendant
  P2IFG &= ~(R_ODOM); // RAZ flag d’interruption

}
  
int main(void) {
  WDTCTL = WDTPW | WDTHOLD;
  BCSCTL1 = CALBC1_1MHZ;
  DCOCTL = CALDCO_1MHZ;
 
  left_wheel();
  right_wheel();

  capteur_setup();

  ADC_init();
  Aff_Init();
 
  __enable_interrupt();

  Etat etat = ETAT_ARRET;
  Evenement evt = EVT_PAS_OBSTACLE;
 
  Transition table_transition[NB_ETATS][NB_EVTS] = {
 
        [ETAT_AVANCER] = {
        [EVT_OBSTACLE] = {ETAT_ARRET, action_arreter},
        [EVT_PAS_OBSTACLE] = {ETAT_AVANCER, action_avancer},
        [EVT_DESTINATION] = {ETAT_ARRET_COMPLET, action_arret_complet}
        },
 
        [ETAT_ARRET] = {
        [EVT_OBSTACLE] = {ETAT_ARRET, NULL},
        [EVT_PAS_OBSTACLE]= {ETAT_AVANCER, action_avancer},
        [EVT_DESTINATION] = {ETAT_ARRET_COMPLET, action_arret_complet}
        },
 
        [ETAT_ARRET_COMPLET] = {
        [EVT_OBSTACLE] = {ETAT_ARRET_COMPLET, NULL},
        [EVT_PAS_OBSTACLE] = {ETAT_ARRET_COMPLET, NULL},
        [EVT_DESTINATION] = {ETAT_ARRET_COMPLET, NULL}
        }
       
  };

  Transition trs;
 
  while (1){
    do
    {
      // Démarre conversion sur voie 1 correspondant au scanner
      ADC_Demarrer_conversion(1);
      // Lecture de la distance              
      distance = ADC_Lire_resultat();      
      Aff_valeur(convert_Hex_Dec(distance));   
 
      lecture_event(distance, &evt); // lecture de l'évenement (obstacle, pas obstacle ou fin)
 
      trs = table_transition[etat][evt];
 
      if (trs.action != NULL)
      {
          trs.action();
      }    
 
      etat = trs.nouvel_etat;

      // lecture sur la voie du capteur de lumière
      ADC_Demarrer_conversion(2);      
      luminosite = ADC_Lire_resultat();      

      // mise à jour LED en fonction du capteur de lumière
      update_light(luminosite);


      __delay_cycles(200);
 
    } while(etat != ETAT_ARRET_COMPLET); // while tant qu'on est pas arrivé à la destination
  }
}