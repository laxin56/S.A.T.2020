/*
 * PD.c
 *
 *  Created on: Nov 4, 2020
 *      Author: nowik
 */

//Plik zawiera implementację regulatora PD

/*

#define KP 1
#define KD 1

float out = 0;

int16_t blad;
int16_t pochodna;
int16_t wczesniejszy_blad;

wartosc zadana - dana orientacja w kącie
wartość rzeczywista - odczyt danego kata

powiazac kat obrotu z predkoscia silnikow i dobrac nastawy


wartosc zadana - predkosc
wartosc rzeczywista - dana predkosc


while(1)
{

	blad = wartość_zadana - wartość rzeczywista;
	pochodna = (blad - wczesniejszy_blad)*dt;
	out = KP*blad + KD*pochodna;
	wczesniejszy_blad = blad;

	HAL_Delay(dt);

}

*/
