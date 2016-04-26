#include "clib.h"

int round(float number){
	return number > 0 ? number + 0.5f : number - 0.5f;
}

int abs(float number){
	if (number > 0) return number;
	else return -(number);
}