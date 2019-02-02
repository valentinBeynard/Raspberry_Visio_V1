#include "Statistics.h"
#include "Tools.h"


Statistics::Statistics()
{
	reset();
}

Statistics::~Statistics()
{
}

void Statistics::reset()
{
	m_maximum = 0.0f;
	m_minimum = FLT_MAX;
	m_sum = 0.0f;
	m_samples = 0;
}

void Statistics::addSample(const float s)
{
	m_minimum = MIN(s, m_minimum);
	m_maximum = MAX(s, m_maximum);
	
	m_sum += m_samples;
	m_samples++;
}

float Statistics::getAverage()
{
	if (m_samples)
		return (m_sum / (float)m_samples);
	else
		return 0.0f;
}
