// TestDLL.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "odedll.h"

int * tt()
{
	int a[3] = {6,5,4};
	int* m = a ;
	m[0]=1;
	m[1] =2;
	m[2] =3;
	return m;
}

int main()
{
	int a = 0;
	a += 5;
	//a += MyMathFuncs::Miao();
	int * youyi =tt() ;
	a = youyi[0];
	a = youyi[1];
	a = youyi[2];
	return 0;
}

