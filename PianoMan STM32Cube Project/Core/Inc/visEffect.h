/*

  WS2812B CPU and memory efficient library

  Date: 28.9.2016

  Author: Martin Hubacek
  	  	  http://www.martinhubacek.cz
  	  	  @hubmartin

  Licence: MIT License

*/

#ifndef VISEFFECT_H_
#define VISEFFECT_H_

#include <stdint.h>
#include <stdbool.h>

void clearLED();
void visInit();
void visHandle();
void setRed(uint8_t);
void setKeys(bool);

#endif /* VISEFFECT_H_ */
