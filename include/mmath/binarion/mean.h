
#ifndef BINARION_MEAN_H
#define BINARION_MEAN_H

#include "mmath/binarion.h"

template<class... T_b>
Binarion Binarion::mean(Binarion first, T_b... args) const
{
	int i = 1;
	Binarion binarion(*this);

	binarion = binarion.slerp(first, 1.0/(++i));

	for(Binarion bina : std::initializer_list<Binarion>{args...}){
		binarion = binarion.slerp(bina, 1.0/(++i));
	}
	
	return binarion;
}

template<class... T_th>
double Binarion::mean(double first, T_th... args)
{
	int i = 1;
	Binarion bina(first);

	for(double theta : std::initializer_list<double>{args...}){
		bina = bina.slerp(Binarion::fromYaw(theta), 1.0/(++i));
	}

	return bina.getYaw();
}

template<class... T_th>
double Binarion::mean(std::string str, double first, T_th... args)
{
	int i = 1;
	Binarion bina(first, str);

	for(double theta : std::initializer_list<double>{args...}){
		bina = bina.slerp(Binarion::fromYaw(theta, str), 1.0/(++i));
	}

	return bina.getYaw(str);
}

#endif

