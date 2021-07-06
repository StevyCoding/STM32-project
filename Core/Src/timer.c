#include "timer.h"




void PWM(TIM_TypeDef * timer, uint8_t canal, uint32_t HCLKFrequency, uint32_t PWMFrequency ,float duty_cycle) {
    /* Activer l’horloge du timer*/



if(timer ==TIM2){
           RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
}else if(timer ==TIM22){
           RCC->APB2ENR |= RCC_APB2ENR_TIM22EN;
       }else if(timer ==TIM6){
           RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
}else if(timer ==TIM21){
           RCC->APB2ENR |= RCC_APB2ENR_TIM21EN;
} timer->CR1|=TIM_CR1_DIR_Msk;
       /*Activation du PWM */
       if( canal ==1){
                  timer->CCMR1 &= ~TIM_CCMR1_OC1M_0;
                  timer->CCMR1 |= TIM_CCMR1_OC1M_1| TIM_CCMR1_OC1M_2;
                  timer->CCER |= TIM_CCER_CC1E;/*activation du PWM du canal*/
              }
              else if(canal==2){
                  timer->CCMR1 &= ~TIM_CCMR1_OC2M_0;
                  timer->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
                  timer->CCER |= TIM_CCER_CC2E;
              }int ARR = HCLKFrequency/PWMFrequency - 1;
              if (ARR>65535) {
                  /* la valeur du prescaler*/
                  timer->PSC=999;
                  timer->ARR = HCLKFrequency/(PWMFrequency*(timer->PSC+1))-1;



              }else {
                  timer->ARR=ARR;



              }
              /*configuration la valeur du registre de comparaison*/
              if(canal==1){
                  timer->CCR1 = (timer->ARR+1)*duty_cycle;
              }
              else if(canal==2){
                  timer->CCR2 = (timer->ARR+1)*duty_cycle;
              } /*activatin du timer*/
              timer->CR1|=TIM_CR1_CEN_Msk;



}






