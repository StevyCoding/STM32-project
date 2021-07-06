#include "alternate.h"

/*
 * alternate.c
 *
 *  Created on: Mar 2, 2021
 *      Author: USER
 */
void AlternateFunction(GPIO_TypeDef * port, uint8_t pin, uint8_t af){
    uint8_t nb_port;
        nb_port=(uint32_t)((uint32_t *)port - IOPPERIPH_BASE)/ (uint32_t)0x400;
        /*Activer l’horloge */
        RCC->IOPENR|=1<<nb_port;
    /*configuration de la pin en mode alternate*/
port->MODER &=~(0b11<<(2*pin));
port->MODER |=(0b10<<(2*pin));



/*Activer la fonction alternative « af »*/
        if (pin<8)
        {
            port->AFR[0] &=~(0b1111<<(4*pin));
        port->AFR[0] |=(af<<(4*pin));
        }
        else if (pin<16)
        {
            port->AFR[1]&=~(0b1111<<4*(pin-8));
            port->AFR[1]|=(af<<4*(pin-8));
        }



}






