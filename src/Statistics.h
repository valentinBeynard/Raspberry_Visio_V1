#pragma once

#include <stdlib.h>

class Statistics
{
public:
	Statistics();
	~Statistics();

	void	reset();
	void	addSample(const float s);
	float	getAverage();

	float	m_minimum;
	float	m_maximum;
	float	m_sum;
	size_t	m_samples;
};

