#include "debugging_functions.h"


void print_bits_8(int8_t bits)
{
	int one = 1;
	int i;
	printf("\nbits = %d, binary: ", bits);
	for(i = 7; i >= 0; i--)
	{
		if((one << i) & bits)
		{
			printf("1");
		}
		else
		{
			printf("0");
		}
	}
	printf("\n");

}

void print_bits_16(int16_t bits)
{
	int one = 1;
	int i;
	printf("\nbits = %d, binary: ", bits);
	for(i = 15; i >= 0; i--)
	{
		if(i == 7)
		{
			printf(" ");
		}
		if((one << i) & bits)
		{
			printf("1");
		}
		else
		{
			printf("0");
		}
	}
	printf("\n");

}

void print_bits_24(int bits)
{
	int one = 1;
	int i;
	printf("\nbits = %d, binary: ", bits);
	for(i = 23; i >= 0; i--)
	{
		if(i == 15 || i == 7)
		{
			printf(" ");
		}
		if((one << i) & bits)
		{
			printf("1");
		}
		else
		{
			printf("0");
		}
	}
	printf("\n");

}

