/*
 * rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
 * Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
 * Copyright (C) 2012 by Hoernchen <la@tfc-server.de>
 * Copyright (C) 2012 by Kyle Keen <keenerd@gmail.com>
 * Copyright (C) 2013 by Elias Oenal <EliasOenal@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <errno.h>
#include <signal.h>
#include <cstring>
#include <cstdio>
#include <cstdlib>

#ifndef _WIN32
#include <unistd.h>
#else
#include <windows.h>
#include <fcntl.h>
#include <io.h>
#include "getopt/getopt.h"
#define usleep(x) Sleep(x/1000)
#if defined(_MSC_VER) && (_MSC_VER < 1800)
#define round(x) (x > 0.0 ? floor(x + 0.5): ceil(x - 0.5))
#endif
#define _USE_MATH_DEFINES
#endif

#include <cmath>
#include <pthread.h>
#include <libusb.h>

#include "rtl-sdr.h"

extern "C"
{
#include "convenience/convenience.h"
}

#define DEFAULT_SAMPLE_RATE		24000
#define DEFAULT_BUF_LENGTH		(1 * 16384)
#define MAXIMUM_OVERSAMPLE		16
#define MAXIMUM_BUF_LENGTH		(MAXIMUM_OVERSAMPLE * DEFAULT_BUF_LENGTH)
#define AUTO_GAIN			-100
#define BUFFER_DUMP			4096

#define FREQUENCIES_LIMIT		1000

static volatile int do_exit = 0;

#include <vector>
#include <complex>

using namespace std;

vector<float> _lutPtr(256);

enum class DecimationFilterType
{
	Fast,
	Baseband,
	Audio
};

namespace DecimationKernels
{
	constexpr float Cic3Max = 0.5f - 0.4985f;
	constexpr float Hb11TapMax = 0.5f - 0.475f;
	constexpr float Hb15TapMax = 0.5f - 0.451f;
	constexpr float Hb19TapMax = 0.5f - 0.428f;
	constexpr float Hb23TapMax = 0.5f - 0.409f;
	constexpr float Hb27TapMax = 0.5f - 0.392f;
	constexpr float Hb31TapMax = 0.5f - 0.378f;
	constexpr float Hb35TapMax = 0.5f - 0.366f;
	constexpr float Hb39TapMax = 0.5f - 0.356f;
	constexpr float Hb43TapMax = 0.5f - 0.347f;
	constexpr float Hb47TapMax = 0.5f - 0.340f;
	constexpr float Hb51TapMax = 0.5f - 0.333f;

	constexpr float Kernel11[] =
	{
		0.0060431029837374152f,
		0.0f,
		-0.049372515458761493f,
		0.0f,
		0.29332944952052842f,
		0.5f,
		0.29332944952052842f,
		0.0f,
		-0.049372515458761493f,
		0.0f,
		0.0060431029837374152f
	};

	constexpr float Kernel15[] =
	{
		-0.001442203300285281f,
		0.0f,
		0.013017512802724852f,
		0.0f,
		-0.061653278604903369f,
		0.0f,
		0.30007792316024057f,
		0.5f,
		0.30007792316024057f,
		0.0f,
		-0.061653278604903369f,
		0.0f,
		0.013017512802724852f,
		0.0f,
		-0.001442203300285281f
	};

	constexpr float Kernel19[] =
	{
		0.00042366527106480427f,
		0.0f,
		-0.0040717333369021894f,
		0.0f,
		0.019895653881950692f,
		0.0f,
		-0.070740034412329067f,
		0.0f,
		0.30449249772844139f,
		0.5f,
		0.30449249772844139f,
		0.0f,
		-0.070740034412329067f,
		0.0f,
		0.019895653881950692f,
		0.0f,
		-0.0040717333369021894f,
		0.0f,
		0.00042366527106480427f
	};

	constexpr float Kernel23[] =
	{
		-0.00014987651418332164f,
		0.0f,
		0.0014748633283609852f,
		0.0f,
		-0.0074416944990005314f,
		0.0f,
		0.026163522731980929f,
		0.0f,
		-0.077593699116544707f,
		0.0f,
		0.30754683719791986f,
		0.5f,
		0.30754683719791986f,
		0.0f,
		-0.077593699116544707f,
		0.0f,
		0.026163522731980929f,
		0.0f,
		-0.0074416944990005314f,
		0.0f,
		0.0014748633283609852f,
		0.0f,
		-0.00014987651418332164f
	};

	constexpr float Kernel27[] =
	{
		0.000063730426952664685f,
		0.0f,
		-0.00061985193978569082f,
		0.0f,
		0.0031512504783365756f,
		0.0f,
		-0.011173151342856621f,
		0.0f,
		0.03171888754393197f,
		0.0f,
		-0.082917863582770729f,
		0.0f,
		0.3097770473566307f,
		0.5f,
		0.3097770473566307f,
		0.0f,
		-0.082917863582770729f,
		0.0f,
		0.03171888754393197f,
		0.0f,
		-0.011173151342856621f,
		0.0f,
		0.0031512504783365756f,
		0.0f,
		-0.00061985193978569082f,
		0.0f,
		0.000063730426952664685f
	};

	constexpr float Kernel31[] =
	{
		-0.000030957335326552226f,
		0.0f,
		0.00029271992847303054f,
		0.0f,
		-0.0014770381124258423f,
		0.0f,
		0.0052539088990950535f,
		0.0f,
		-0.014856378748476874f,
		0.0f,
		0.036406651919555999f,
		0.0f,
		-0.08699862567952929f,
		0.0f,
		0.31140967076042625f,
		0.5f,
		0.31140967076042625f,
		0.0f,
		-0.08699862567952929f,
		0.0f,
		0.036406651919555999f,
		0.0f,
		-0.014856378748476874f,
		0.0f,
		0.0052539088990950535f,
		0.0f,
		-0.0014770381124258423f,
		0.0f,
		0.00029271992847303054f,
		0.0f,
		-0.000030957335326552226f
	};

	constexpr float Kernel35[] =
	{
		0.000017017718072971716f,
		0.0f,
		-0.00015425042851962818f,
		0.0f,
		0.00076219685751140838f,
		0.0f,
		-0.002691614694785393f,
		0.0f,
		0.0075927497927344764f,
		0.0f,
		-0.018325727896057686f,
		0.0f,
		0.040351004914363969f,
		0.0f,
		-0.090198224668969554f,
		0.0f,
		0.31264689763504327f,
		0.5f,
		0.31264689763504327f,
		0.0f,
		-0.090198224668969554f,
		0.0f,
		0.040351004914363969f,
		0.0f,
		-0.018325727896057686f,
		0.0f,
		0.0075927497927344764f,
		0.0f,
		-0.002691614694785393f,
		0.0f,
		0.00076219685751140838f,
		0.0f,
		-0.00015425042851962818f,
		0.0f,
		0.000017017718072971716f
	};

	constexpr float Kernel39[] =
	{
		-0.000010175082832074367f,
		0.0f,
		0.000088036416015024345f,
		0.0f,
		-0.00042370835558387595f,
		0.0f,
		0.0014772557414459019f,
		0.0f,
		-0.0041468438954260153f,
		0.0f,
		0.0099579126901608011f,
		0.0f,
		-0.021433527104289002f,
		0.0f,
		0.043598963493432855f,
		0.0f,
		-0.092695953625928404f,
		0.0f,
		0.31358799113382152f,
		0.5f,
		0.31358799113382152f,
		0.0f,
		-0.092695953625928404f,
		0.0f,
		0.043598963493432855f,
		0.0f,
		-0.021433527104289002f,
		0.0f,
		0.0099579126901608011f,
		0.0f,
		-0.0041468438954260153f,
		0.0f,
		0.0014772557414459019f,
		0.0f,
		-0.00042370835558387595f,
		0.0f,
		0.000088036416015024345f,
		0.0f,
		-0.000010175082832074367f
	};

	constexpr float Kernel43[] =
	{
		0.0000067666739082756387f,
		0.0f,
		-0.000055275221547958285f,
		0.0f,
		0.00025654074579418561f,
		0.0f,
		-0.0008748125689163153f,
		0.0f,
		0.0024249876017061502f,
		0.0f,
		-0.0057775190656021748f,
		0.0f,
		0.012299834239523121f,
		0.0f,
		-0.024244050662087069f,
		0.0f,
		0.046354303503099069f,
		0.0f,
		-0.094729903598633314f,
		0.0f,
		0.31433918020123208f,
		0.5f,
		0.31433918020123208f,
		0.0f,
		-0.094729903598633314f,
		0.0f,
		0.046354303503099069f,
		0.0f,
		-0.024244050662087069f,
		0.0f,
		0.012299834239523121f,
		0.0f,
		-0.0057775190656021748f,
		0.0f,
		0.0024249876017061502f,
		0.0f,
		-0.0008748125689163153f,
		0.0f,
		0.00025654074579418561f,
		0.0f,
		-0.000055275221547958285f,
		0.0f,
		0.0000067666739082756387f
	};

	constexpr float Kernel47[] =
	{
		-0.0000045298314172004251f,
		0.0f,
		0.000035333704512843228f,
		0.0f,
		-0.00015934776420643447f,
		0.0f,
		0.0005340788063118928f,
		0.0f,
		-0.0014667949695500761f,
		0.0f,
		0.0034792089350833247f,
		0.0f,
		-0.0073794356720317733f,
		0.0f,
		0.014393786384683398f,
		0.0f,
		-0.026586603160193314f,
		0.0f,
		0.048538673667907428f,
		0.0f,
		-0.09629115286535718f,
		0.0f,
		0.31490673428547367f,
		0.5f,
		0.31490673428547367f,
		0.0f,
		-0.09629115286535718f,
		0.0f,
		0.048538673667907428f,
		0.0f,
		-0.026586603160193314f,
		0.0f,
		0.014393786384683398f,
		0.0f,
		-0.0073794356720317733f,
		0.0f,
		0.0034792089350833247f,
		0.0f,
		-0.0014667949695500761f,
		0.0f,
		0.0005340788063118928f,
		0.0f,
		-0.00015934776420643447f,
		0.0f,
		0.000035333704512843228f,
		0.0f,
		-0.0000045298314172004251f
	};

	constexpr float Kernel51[] =
	{
		0.0000033359253688981639f,
		0.0f,
		-0.000024584155158361803f,
		0.0f,
		0.00010677777483317733f,
		0.0f,
		-0.00034890723143173914f,
		0.0f,
		0.00094239127078189603f,
		0.0f,
		-0.0022118302078923137f,
		0.0f,
		0.0046575030752162277f,
		0.0f,
		-0.0090130973415220566f,
		0.0f,
		0.016383673864361164f,
		0.0f,
		-0.028697281101743237f,
		0.0f,
		0.05043292242400841f,
		0.0f,
		-0.097611898315791965f,
		0.0f,
		0.31538104435015801f,
		0.5f,
		0.31538104435015801f,
		0.0f,
		-0.097611898315791965f,
		0.0f,
		0.05043292242400841f,
		0.0f,
		-0.028697281101743237f,
		0.0f,
		0.016383673864361164f,
		0.0f,
		-0.0090130973415220566f,
		0.0f,
		0.0046575030752162277f,
		0.0f,
		-0.0022118302078923137f,
		0.0f,
		0.00094239127078189603f,
		0.0f,
		-0.00034890723143173914f,
		0.0f,
		0.00010677777483317733f,
		0.0f,
		-0.000024584155158361803f,
		0.0f,
		0.0000033359253688981639f
	};
};

enum class WindowType
{
	None,
	Hamming,
	Blackman,
	BlackmanHarris4,
	BlackmanHarris7,
	HannPoisson,
	Youssef
};

namespace FilterBuilder
{
	void Normalize(vector<float>& h)
	{
		// Normalize the filter kernel for unity gain at DC
		float sum = 0.0f;
		for (size_t i = 0; i < h.size(); i++)
		{
			sum += h[i];
		}
		for (size_t i = 0; i < h.size(); i++)
		{
			h[i] /= sum;
		}
	}

	void ApplyWindow(vector<float>& coefficients, const vector<float>& window)
	{
		for (size_t i = 0; i < coefficients.size(); i++)
		{
			coefficients[i] *= window[i];
		}
	}

	vector<float> MakeWindow(WindowType windowType, int length)
	{
		vector<float> w(length);
		for (int i = 0; i < length; i++)
		{
			float n;
			float a0;
			float a1;
			float a2;
			float a3;
			float a4;
			float a5;
			float a6;
			float alpha;

			w[i] = 1.0f;

			switch (windowType)
			{
				case WindowType::Hamming:
					a0 = 0.54f;
					a1 = 0.46f;
					a2 = 0.0f;
					a3 = 0.0f;
					w[i] *= a0
						  - a1 * (float) cos(2.0 * M_PI * i / length)
						  + a2 * (float) cos(4.0 * M_PI * i / length)
						  - a3 * (float) cos(6.0 * M_PI * i / length);
					break;

				case WindowType::Blackman:
					a0 = 0.42f;
					a1 = 0.5f;
					a2 = 0.08f;
					a3 = 0.0f;
					w[i] *= a0
						  - a1 * (float) cos(2.0 * M_PI * i / length)
						  + a2 * (float) cos(4.0 * M_PI * i / length)
						  - a3 * (float) cos(6.0 * M_PI * i / length);
					break;

				case WindowType::BlackmanHarris4:
					a0 = 0.35875f;
					a1 = 0.48829f;
					a2 = 0.14128f;
					a3 = 0.01168f;
					w[i] *= a0
						  - a1 * (float) cos(2.0 * M_PI * i / length)
						  + a2 * (float) cos(4.0 * M_PI * i / length)
						  - a3 * (float) cos(6.0 * M_PI * i / length);
					break;

				case WindowType::BlackmanHarris7:
					a0 = 0.27105140069342f;
					a1 = 0.43329793923448f;
					a2 = 0.21812299954311f;
					a3 = 0.06592544638803f;
					a4 = 0.01081174209837f;
					a5 = 0.00077658482522f;
					a6 = 0.00001388721735f;
					w[i] *= a0
						  - a1 * (float) cos(2.0 * M_PI * i / length)
						  + a2 * (float) cos(4.0 * M_PI * i / length)
						  - a3 * (float) cos(6.0 * M_PI * i / length)
						  + a4 * (float) cos(8.0 * M_PI * i / length)
						  - a5 * (float) cos(10.0 * M_PI * i / length)
						  + a6 * (float) cos(12.0 * M_PI * i / length);
					break;

				case WindowType::HannPoisson:
					n = i - length / 2.0f;
					alpha = 0.005f;
					w[i] *= 0.5f * (float) ((1.0 + cos(2.0 * M_PI * n / length)) * exp(-2.0 * alpha * abs(n) / length));
					break;

				case WindowType::Youssef:
					a0 = 0.35875f;
					a1 = 0.48829f;
					a2 = 0.14128f;
					a3 = 0.01168f;
					n = i - length / 2.0f;
					alpha = 0.005f;
					w[i] *= a0
						  - a1 * (float) cos(2.0 * M_PI * i / length)
						  + a2 * (float) cos(4.0 * M_PI * i / length)
						  - a3 * (float) cos(6.0 * M_PI * i / length);
					w[i] *= (float) exp(-2.0 * alpha * abs(n) / length);
					break;
				default:
					break;
			}
		}
		return w;
	}

	vector<float> MakeSinc(double sampleRate, double frequency, int length)
	{
		if (length % 2 == 0)
		{
			fprintf(stderr, "Length should be odd");
			exit(1);
		}

		double freqInRad = 2.0 * M_PI * frequency / sampleRate;

		vector<float> h(length);

		for (int i = 0; i < length; i++)
		{
			int n = i - length / 2;
			if (n == 0)
				h[i] = (float) freqInRad;
			else
				h[i] = (float) (sin(freqInRad * n) / n);
		}

		return h;
	}

	vector<float> MakeLowPassKernel(double sampleRate, int filterOrder, int cutoffFrequency, WindowType windowType)
	{
		filterOrder |= 1;

		vector<float> h = MakeSinc(sampleRate, cutoffFrequency, filterOrder);
		vector<float> w = MakeWindow(windowType, filterOrder);

		ApplyWindow(h, w);

		Normalize(h);

		return h;
	}

	vector<float> MakeBandPassKernel(double sampleRate, int filterOrder, int cutoff1, int cutoff2, WindowType windowType)
	{
		double bw = (cutoff2 - cutoff1) / 2;
		double fshift = cutoff2 - bw;
		double shiftRadians = 2 * M_PI * fshift / sampleRate;

		vector<float> h = MakeLowPassKernel(sampleRate, filterOrder, bw, windowType);

		for (int i = 0; i < h.size(); i++)
		{
			int n = i - filterOrder / 2;
			h[i] *= (float) (2 * cos(shiftRadians * n));
		}
		return h;
	}
};

class FirFilter
{
private:
	static constexpr double Epsilon = 1e-6;
	static constexpr int CircularBufferSize = 2;

	vector<float> _coeffBuffer;
	vector<float> _queueBuffer;

	size_t _queueSize;
	int _offset;
	bool _isSymmetric;
	bool _isHalfBand;
	int _decimationFactor;

public:
	FirFilter(const vector<float>& coefficients, int decimationFactor):
		_coeffBuffer(),
		_queueBuffer(),
		_decimationFactor(decimationFactor)// TODO err checking
	{
		SetCoefficients(coefficients);
	}

	size_t getLength()
	{
		return _queueSize;
	}

	void SetCoefficients(const vector<float>& coefficients)
	{
		if (coefficients.empty())
		{
			return;
		}

		if (_coeffBuffer.empty() || coefficients.size() != _queueSize)
		{
			_queueSize = coefficients.size();
			_offset = _queueSize * (CircularBufferSize - 1);

			_coeffBuffer.resize(_queueSize);
			_queueBuffer.resize(_queueSize * CircularBufferSize);
		}

		for (size_t i = 0; i < _queueSize; i++)
		{
			_coeffBuffer[i] = coefficients[i];
		}

		if (_queueSize % 2)
		{
			_isSymmetric = true;
			_isHalfBand = true;

			size_t halfLen = _queueSize / 2;

			for (size_t i = 0; i < halfLen; i++)
			{
				size_t j = _queueSize - 1 - i;
				if (abs(_coeffBuffer[i] - _coeffBuffer[j]) > Epsilon)
				{
					_isSymmetric = false;
					_isHalfBand = false;
					break;
				}
				if (i % 2)
				{
					_isHalfBand = _coeffBuffer[i] == 0.0f && _coeffBuffer[j] == 0.0f;
				}
			}

			if (_isHalfBand)
			{
				_decimationFactor = 2;
			}
		}
	}

private:
	void ProcessSymmetricKernel(float* buffer, int length)
	{
		for (int n = 0, m = 0; n < length; n += _decimationFactor, m++)
		{
			float* queue = _queueBuffer.data() + _offset;

			for (int k = 0, l = n + _decimationFactor - 1; k < _decimationFactor; k++, l--)
			{
				queue[k] = buffer[l];
			}

			float acc = 0.0f;

			size_t halfLen = _queueSize / 2;
			size_t len = halfLen;

			float* ptr1 = _coeffBuffer.data();
			float* ptr2 = queue;
			float* ptr3 = queue + _queueSize - 1;

			if (len >= 4)
			{
				do
				{
					acc += ptr1[0] * (ptr2[0] + ptr3[0])
						 + ptr1[1] * (ptr2[1] + ptr3[-1])
						 + ptr1[2] * (ptr2[2] + ptr3[-2])
						 + ptr1[3] * (ptr2[3] + ptr3[-3]);

					ptr1 += 4;
					ptr2 += 4;
					ptr3 -= 4;
				} while ((len -= 4) >= 4);
			}
			while (len-- > 0)
			{
				acc += *ptr1++ * (*ptr2++ + *ptr3--);
			}
			acc += queue[halfLen] * _coeffBuffer[halfLen];

			if ((_offset -= _decimationFactor) < 0)
			{
				int oldOffset = _offset + _decimationFactor;
				_offset += _queueSize * (CircularBufferSize - 1);
				memcpy(_queueBuffer.data() + _offset + _decimationFactor, _queueBuffer.data() + oldOffset, (_queueSize - _decimationFactor) * sizeof(float));
			}

			buffer[m] = acc;
		}
	}

	void ProcessSymmetricKernelInterleaved(float* buffer, int length)
	{
		length <<= 1;
		for (int n = 0, m = 0; n < length; n += _decimationFactor * 2, m += 2)
		{
			float* queue = _queueBuffer.data() + _offset;

			for (int k = 0, l = n + 2 * (_decimationFactor - 1); k < _decimationFactor; k++, l -= 2)
			{
				queue[k] = buffer[l];
			}

			float acc = 0.0f;

			size_t halfLen = _queueSize / 2;
			size_t len = halfLen;

			float* ptr1 = _coeffBuffer.data();
			float* ptr2 = queue;
			float* ptr3 = queue + _queueSize - 1;

			if (len >= 4)
			{
				do
				{
					acc += ptr1[0] * (ptr2[0] + ptr3[0])
						 + ptr1[1] * (ptr2[1] + ptr3[-1])
						 + ptr1[2] * (ptr2[2] + ptr3[-2])
						 + ptr1[3] * (ptr2[3] + ptr3[-3]);

					ptr1 += 4;
					ptr2 += 4;
					ptr3 -= 4;
				} while ((len -= 4) >= 4);
			}
			while (len-- > 0)
			{
				acc += *ptr1++ * (*ptr2++ + *ptr3--);
			}
			acc += queue[halfLen] * _coeffBuffer[halfLen];

			if ((_offset -= _decimationFactor) < 0)
			{
				int oldOffset = _offset + _decimationFactor;
				_offset += _queueSize * (CircularBufferSize - 1);
				memcpy(_queueBuffer.data() + _offset + _decimationFactor, _queueBuffer.data() + oldOffset, (_queueSize - _decimationFactor) * sizeof(float));
			}

			buffer[m] = acc;
		}
	}

	void ProcessHalfBandKernel(float* buffer, int length)
	{
		for (int n = 0, m = 0; n < length; n += 2, m++)
		{
			float* queue = _queueBuffer.data() + _offset;

			queue[0] = buffer[n + 1];
			queue[1] = buffer[n];

			float acc = 0.0f;

			size_t halfLen = _queueSize / 2;
			size_t len = halfLen;

			float* ptr1 = _coeffBuffer.data();
			float* ptr2 = queue;
			float* ptr3 = queue + _queueSize - 1;

			if (len >= 8)
			{
				do
				{
					acc += ptr1[0] * (ptr2[0] + ptr3[0])
						 + ptr1[2] * (ptr2[2] + ptr3[-2])
						 + ptr1[4] * (ptr2[4] + ptr3[-4])
						 + ptr1[6] * (ptr2[6] + ptr3[-6]);

					ptr1 += 8;
					ptr2 += 8;
					ptr3 -= 8;
				} while ((len -= 8) >= 8);
			}
			if (len >= 4)
			{
				acc += ptr1[0] * (ptr2[0] + ptr3[0])
						+ ptr1[2] * (ptr2[2] + ptr3[-2]);
				ptr1 += 4;
				ptr2 += 4;
				ptr3 -= 4;
				len -= 4;
			}
			while (len-- > 0)
			{
				acc += *ptr1++ * (*ptr2++ + *ptr3--);
			}
			acc += queue[halfLen] * _coeffBuffer[halfLen];

			if ((_offset -= _decimationFactor) < 0)
			{
				int oldOffset = _offset + _decimationFactor;
				_offset += _queueSize * (CircularBufferSize - 1);
				memcpy(_queueBuffer.data() + _offset + _decimationFactor, _queueBuffer.data() + oldOffset, (_queueSize - _decimationFactor) * sizeof(float));
			}

			buffer[m] = acc;
		}
	}

	void ProcessHalfBandInterleaved(float* buffer, int length)
	{
		length <<= 1;
		for (int n = 0, m = 0; n < length; n += 4, m += 2)
		{
			float* queue = _queueBuffer.data() + _offset;

			queue[0] = buffer[n + 2];
			queue[1] = buffer[n];

			float acc = 0.0f;

			size_t halfLen = _queueSize / 2;
			size_t len = halfLen;

			float* ptr1 = _coeffBuffer.data();
			float* ptr2 = queue;
			float* ptr3 = queue + _queueSize - 1;

			if (len >= 8)
			{
				do
				{
					acc += ptr1[0] * (ptr2[0] + ptr3[0])
						 + ptr1[2] * (ptr2[2] + ptr3[-2])
						 + ptr1[4] * (ptr2[4] + ptr3[-4])
						 + ptr1[6] * (ptr2[6] + ptr3[-6]);

					ptr1 += 8;
					ptr2 += 8;
					ptr3 -= 8;
				} while ((len -= 8) >= 8);
			}
			if (len >= 4)
			{
				acc += ptr1[0] * (ptr2[0] + ptr3[0])
					 + ptr1[2] * (ptr2[2] + ptr3[-2]);
				ptr1 += 4;
				ptr2 += 4;
				ptr3 -= 4;
				len -= 4;
			}
			while (len-- > 0)
			{
				acc += *ptr1++ * (*ptr2++ + *ptr3--);
			}
			acc += queue[halfLen] * _coeffBuffer[halfLen];

			if ((_offset -= _decimationFactor) < 0)
			{
				int oldOffset = _offset + _decimationFactor;
				_offset += _queueSize * (CircularBufferSize - 1);
				memcpy(_queueBuffer.data() + _offset + _decimationFactor, _queueBuffer.data() + oldOffset, (_queueSize - _decimationFactor) * sizeof(float));
			}

			buffer[m] = acc;
		}
	}

	void ProcessStandard(float* buffer, int length)
	{
		for (int n = 0, m = 0; n < length; n+= _decimationFactor, m++)
		{
			float* queue = _queueBuffer.data() + _offset;

			for (int k = 0, l = n + _decimationFactor - 1; k < _decimationFactor; k++, l--)
			{
				queue[k] = buffer[l];
			}

			float acc = 0.0f;

			size_t len = _queueSize;
			float* ptr1 = queue;
			float* ptr2 = _coeffBuffer.data();
			if (len >= 4)
			{
				do
				{
					acc += ptr1[0] * ptr2[0]
						 + ptr1[1] * ptr2[1]
						 + ptr1[2] * ptr2[2]
						 + ptr1[3] * ptr2[3];
					ptr1 += 4;
					ptr2 += 4;
				} while ((len -= 4) >= 4);
			}
			while (len-- > 0)
			{
				acc += *ptr1++ * *ptr2++;
			}

			if ((_offset -= _decimationFactor) < 0)
			{
				int oldOffset = _offset + _decimationFactor;
				_offset += _queueSize * (CircularBufferSize - 1);
				memcpy(_queueBuffer.data() + _offset + _decimationFactor, _queueBuffer.data() + oldOffset, (_queueSize - _decimationFactor) * sizeof(float));
			}

			buffer[m] = acc;
		}
	}

	void ProcessStandardInterleaved(float* buffer, int length)
	{
		length <<= 1;
		for (int n = 0, m = 0; n < length; n += _decimationFactor * 2, m += 2)
		{
			float* queue = _queueBuffer.data() + _offset;

			for (int k = 0, l = n + 2 * (_decimationFactor - 1); k < _decimationFactor; k++, l -= 2)
			{
				queue[k] = buffer[l];
			}

			float acc = 0.0f;

			size_t len = _queueSize;
			float* ptr1 = queue;
			float* ptr2 = _coeffBuffer.data();
			if (len >= 4)
			{
				do
				{
					acc += ptr1[0] * ptr2[0]
						 + ptr1[1] * ptr2[1]
						 + ptr1[2] * ptr2[2]
						 + ptr1[3] * ptr2[3];
					ptr1 += 4;
					ptr2 += 4;
				} while ((len -= 4) >= 4);
			}
			while (len-- > 0)
			{
				acc += *ptr1++ * *ptr2++;
			}

			if ((_offset -= _decimationFactor) < 0)
			{
				int oldOffset = _offset + _decimationFactor;
				_offset += _queueSize * (CircularBufferSize - 1);
				memcpy(_queueBuffer.data() + _offset + _decimationFactor, _queueBuffer.data() + oldOffset, (_queueSize - _decimationFactor) * sizeof(float));
			}

			buffer[m] = acc;
		}
	}

public:
	void Process(float* buffer, int length)
	{
		if (_isHalfBand)
			ProcessHalfBandKernel(buffer, length);
		else if (_isSymmetric)
			ProcessSymmetricKernel(buffer, length);
		else
			ProcessStandard(buffer, length);
	}

	void ProcessInterleaved(float* buffer, int length)
	{
		if (_isHalfBand)
			ProcessHalfBandInterleaved(buffer, length);
		else if (_isSymmetric)
			ProcessSymmetricKernelInterleaved(buffer, length);
		else
			ProcessStandardInterleaved(buffer, length);
	}
};

class IQFirFilter
{
	FirFilter _rFilter;
	FirFilter _iFilter;

public:
	IQFirFilter(const vector<float>& coefficients, int decimationFactor = 1):
		_rFilter(coefficients, decimationFactor),
		_iFilter(coefficients, decimationFactor)
	{

	}

	void Process(vector<complex<float>>& iq)
	{
		float* ptr = (float*) iq.data();
		int length = iq.size();
		_rFilter.ProcessInterleaved(ptr, length);
		_iFilter.ProcessInterleaved(ptr + 1, length);
	}

	void SetCoefficients(const vector<float>& coefficients)
	{
		_rFilter.SetCoefficients(coefficients);
		_iFilter.SetCoefficients(coefficients);
	}
};

struct CicDecimator
{
	float _xOdd;
	float _xEven;

	void Process(float* buffer, int length)
	{
		for (int i = 0, j = 0; i < length; i += 2, j++)
		{
			float even = buffer[i];
			float odd = buffer[i + 1];
			buffer[j] = (float) (0.125 * (odd + _xEven + 3.0 * (_xOdd + even)));
			_xOdd = odd;
			_xEven = even;
		}
	}

	void ProcessInterleaved(float* buffer, int length)
	{
		length *= 2;
		for (int i = 0, j = 0; i < length; i += 4, j += 2)
		{
			float even = buffer[i];
			float odd = buffer[i + 2];
			buffer[j] = 0.125f * (odd + _xEven + 3.0f * (_xOdd + even));
			_xOdd = odd;
			_xEven = even;
		}
	}
};

class FloatDecimator
{
	int _stageCount;
	vector<CicDecimator> cic_decimators;
	vector<FirFilter> _firFilters;
	static constexpr double _minimumCICSampleRate = 1500000;

public:
	FloatDecimator(int stageCount, double samplerate = 0, DecimationFilterType filterType = DecimationFilterType::Audio)
	{
		_stageCount = stageCount;

		int _cicCount = 0;
		int firCount = 0;

		switch (filterType)
		{
			case DecimationFilterType::Fast:
				_cicCount = stageCount;
				break;

			case DecimationFilterType::Audio:
				firCount = stageCount;
				break;

			case DecimationFilterType::Baseband:
				while (_cicCount < stageCount && samplerate >= _minimumCICSampleRate)
				{
					_cicCount++;
					samplerate /= 2;
				}
				firCount = stageCount - _cicCount;
				break;
		}

		// todo resize and memcpy
		vector<float> tmp_coefs;
		for(float f: DecimationKernels::Kernel51)
			tmp_coefs.push_back(f);

		// call constructors manually?
		cic_decimators.resize(_cicCount);
		_firFilters.resize(firCount, FirFilter(tmp_coefs, 2));
	}

	int get_stagecount()
	{
		return _stageCount;
	}

	void Process(float* buffer, int length)
	{
		DecimateStage1(buffer, length);
		length >>= cic_decimators.size();
		DecimateStage2(buffer, length);
	}

	void ProcessInterleaved(float* buffer, int length)
	{
		DecimateStage1Interleaved(buffer, length);
		length >>= cic_decimators.size();
		DecimateStage2Interleaved(buffer, length);
	}

private:
	void DecimateStage1(float* buffer, int sampleCount)
	{
		for (size_t n = 0; n < cic_decimators.size(); n++)
		{
			int contextId = 0;
			float* chunk = buffer;
			int chunkLength = sampleCount;

			cic_decimators[contextId * cic_decimators.size() + n].Process(chunk, chunkLength);

			sampleCount /= 2;
		}
	}

	void DecimateStage2(float* buffer, int length)
	{
		for (size_t n = 0; n < _firFilters.size(); n++)
		{
			_firFilters[n].Process(buffer, length);
			length /= 2;
		}
	}

	void DecimateStage1Interleaved(float* buffer, int sampleCount)
	{
		for (size_t n = 0; n < cic_decimators.size(); n++)
		{
			int contextId = 0;
			float* chunk = buffer;
			int chunkLength = sampleCount;

			cic_decimators[contextId * cic_decimators.size() + n].ProcessInterleaved(chunk, chunkLength);

			sampleCount /= 2;
		}
	}

	void DecimateStage2Interleaved(float* buffer, int length)
	{
		for (size_t n = 0; n < _firFilters.size(); n++)
		{
			_firFilters[n].ProcessInterleaved(buffer, length);
			length /= 2;
		}
	}
};

class IQDecimator
{
	FloatDecimator _rDecimator;
	FloatDecimator _iDecimator;

public:

	IQDecimator(int stageCount, double samplerate, DecimationFilterType filterType):
		_rDecimator(stageCount, samplerate, filterType),
		_iDecimator(stageCount, samplerate, filterType)
	{

	}

	void Process(vector<complex<float>>& buffer)
	{
		float* rPtr = (float*) buffer.data();
		float* iPtr = rPtr + 1;

		_rDecimator.ProcessInterleaved(rPtr, buffer.size());
		_iDecimator.ProcessInterleaved(iPtr, buffer.size());
	}

	int get_stagecount()
	{
		return _rDecimator.get_stagecount();
	}
};

complex<float> FromAngle(double angle)
{
	return complex<float>(cos(angle), sin(angle));
}

struct Oscillator
{
private:
	complex<float> _rotation;
	complex<float> _vector;
	double _sampleRate;
	double _frequency;

public:

	double get_samplerate()
	{
		return _sampleRate;
	}

	void set_samplerate(double value)
	{
		if (_sampleRate != value)
		{
			_sampleRate = value;
			Configure();
		}
	}

	double get_frequency()
	{
		return _frequency;
	}

	void set_frequency(double value)
	{
		if (_frequency != value)
		{
			_frequency = value;
			Configure();
		}
	}

	void Configure()
	{
		if (_vector.real() == 0.0f && _vector.imag() == 0.0f)
		{
			_vector.real(1.0f);
		}
		if (_sampleRate != 0.0)
		{
			double anglePerSample = 2.0 * M_PI * _frequency / _sampleRate;
			_rotation = FromAngle(anglePerSample);
		}
	}

	complex<float> get_phase()
	{
		return _vector;
	}

	void set_phase(complex<float> value)
	{
		_vector = value;
	}

	float get_real()
	{
		return _vector.real();
	}

	void set_real(float value)
	{
		_vector.real(value);
	}

	float get_imag()
	{
		return _vector.imag();
	}

	void set_imag(float value)
	{
		_vector.imag(value);
	}

	void Tick()
	{
		_vector *= _rotation;
		float norm = 1.0f / abs(_vector);
		_vector *= norm;
	}

	void Mix(vector<float>& buffer)
	{
		for (size_t i = 0; i < buffer.size(); i++)
		{
			Tick();
			buffer[i] *= _vector.real();
		}
	}

	void Mix(vector<complex<float>>& buffer)
	{
		for (size_t i = 0; i < buffer.size(); i++)
		{
			Tick();
			buffer[i] *= _vector;
		}
	}

	/*static implicit operator Complex(Oscillator osc)
	{
		return osc.Phase;
	}*/
};

class DownConverter
{
	Oscillator osc;

	double _sampleRate;
	double _frequency;

	void Configure()
	{
		if (_sampleRate == 0.0)
			return;

		osc.set_samplerate(_sampleRate);
		osc.set_frequency(_frequency);
	}

public:
	double get_samplerate()
	{
		return _sampleRate;
	}

	void set_samplerate(double value)
	{
		if (_sampleRate != value)
		{
			_sampleRate = value;
			Configure();
		}
	}

	double get_frequency()
	{
		return _frequency;
	}

	void set_frequency(double value)
	{
		if (_frequency != value)
		{
			_frequency = value;
			Configure();
		}
	}

	void Process(vector<complex<float>>& buffer)
	{
		osc.Mix(buffer);
	}
};

constexpr int MaxDecimationFactor = 1024;
constexpr int minOutputSampleRate = 32000;

int GetDecimationStageCount(int input_sample_rate)
{
	if (input_sample_rate <= minOutputSampleRate)
		return 0;

	int result = MaxDecimationFactor;
	while (input_sample_rate < minOutputSampleRate * result && result > 0)
		result /= 2;

	fprintf(stderr, "GetDecimationStageCount: %d\n", result);

	return log2(result);
}

struct dongle_state
{
	int      exit_flag;
	pthread_t thread;
	rtlsdr_dev_t *dev;
	int      dev_index;
	uint32_t freq;
	uint32_t rate;
	int      gain;
	int      ppm_error;
	int      direct_sampling;
	int      mute;
	struct demod_state *demod_target;
};

struct demod_state
{
	int      exit_flag;
	pthread_t thread;

	vector<complex<float>> iq;

	complex<float> iq_state; // fm only

	vector<float> audio;

	int      bandwidth;
	int      downsample;    /* min 1, max 256 */
	int      squelch_level, conseq_squelch, terminate_on_squelch;
	void     (*mode_demod)(struct demod_state*);
	pthread_rwlock_t rw;
	pthread_cond_t ready;
	pthread_mutex_t ready_m;
	struct output_state *output_target;

	vector<float> hissBuffer;
	FirFilter* hissFilter;
	float _noiseLevel;
	float noiseThreshold;
	float noiseAveragingRatio;
	bool shouldISkipThis;

	IQFirFilter* iq_filter;
	DownConverter down_converter;

	int decim_stage_count;
	IQDecimator* iqdecim;
};

struct output_state
{
	int       exit_flag;
	pthread_t thread;

	FILE*     file;

	float     result[MAXIMUM_BUF_LENGTH];
	size_t    result_len;

	pthread_rwlock_t rw;
	pthread_cond_t ready;
	pthread_mutex_t ready_m;
};

struct controller_state
{
	int      exit_flag;
	pthread_t thread;
	uint32_t freqs[FREQUENCIES_LIMIT];
	int      freq_len;
	int      freq_now;
	int      edge;
	pthread_cond_t hop;
	pthread_mutex_t hop_m;
};

// multiple of these, eventually
struct dongle_state dongle;
struct demod_state demod;
struct output_state output;
struct controller_state controller;

void usage(void)
{
	fprintf(stderr,
		"rtl_fm, a simple narrow band FM demodulator for RTL2832 based DVB-T receivers\n\n"
		"Use:\trtl_fm -f freq [-options] [filename]\n"
		"\t-f frequency_to_tune_to [Hz]\n"
		"\t    use multiple -f for scanning (requires squelch)\n"
		"\t    ranges supported, -f 118M:137M:25k\n"
		"\t[-M modulation (default: fm)]\n"
		"\t    fm, wbfm, raw, am, usb, lsb\n"
		"\t    wbfm == -M fm -s 170k -o 4 -A fast -r 32k -l 0 -E deemp\n"
		"\t    raw mode outputs 2x16 bit IQ pairs\n"
		"\t[-b bandwidth (default: 12.5k)]\n"
		"\t[-d device_index (default: 0)]\n"
		"\t[-T enable bias-T on GPIO PIN 0 (works for rtl-sdr.com v3 dongles)]\n"
		"\t[-g tuner_gain (default: automatic)]\n"
		"\t[-l squelch_level (default: 0/off)]\n"
		"\t[-p ppm_error (default: 0)]\n"
		"\t[-E enable_option (default: none)]\n"
		"\t    use multiple -E to enable multiple options\n"
		"\t    edge:   enable lower edge tuning\n"
		"\t    dc:     enable dc blocking filter\n"
		"\t    deemp:  enable de-emphasis filter\n"
		"\t    direct: enable direct sampling\n"
		"\tfilename ('-' means stdout)\n"
		"\t    omitting the filename also uses stdout\n\n"
		"\n"
		"Produces signed 16 bit ints, use Sox or aplay to hear them.\n"
		"\trtl_fm ... | play -t raw -r 24k -es -b 16 -c 1 -V1 -\n"
		"\t           | aplay -r 24k -f S16_LE -t raw -c 1\n"
		"\t  -M wbfm  | play -r 32k ... \n"
		"\t  -s 22050 | multimon -t raw /dev/stdin\n\n");
	exit(1);
}

#ifdef _WIN32
BOOL WINAPI sighandler(int signum)
{
	if (signum = CTRL_C_EVENT)
	{
		fprintf(stderr, "Signal caught, exiting!\n");
		do_exit = 1;
		rtlsdr_cancel_async(dongle.dev);
		return TRUE;
	}

	return FALSE;
}
#else
static void sighandler(int)
{
	fprintf(stderr, "Signal caught, exiting!\n");
	do_exit = 1;
	rtlsdr_cancel_async(dongle.dev);
}
#endif

/* more cond dumbness */
#define safe_cond_signal(n, m) pthread_mutex_lock(m); pthread_cond_signal(n); pthread_mutex_unlock(m)
#define safe_cond_wait(n, m) pthread_mutex_lock(m); pthread_cond_wait(n, m); pthread_mutex_unlock(m)

#if defined(_MSC_VER) && (_MSC_VER < 1800)
double log2(double n)
{
	return log(n) / log(2.0);
}
#endif

constexpr int MinHissFrequency = 4000;
constexpr int MaxHissFrequency = 6000;
constexpr int HissFilterOrder = 20;
constexpr float HissFactor = 0.00002f;

void ProcessSquelch(struct demod_state* d)
{
	d->hissBuffer = d->audio;
	d->hissFilter->Process(d->hissBuffer.data(), d->hissBuffer.size());

	fprintf(stderr, "%zu; %.10f; %.10f; %.10f\n", d->audio.size(), d->noiseAveragingRatio, d->noiseThreshold, d->_noiseLevel);

	size_t skipped = 0;

	for (size_t i = 0; i < d->hissBuffer.size(); i++)
	{
		float n = (1.0f - d->noiseAveragingRatio) * d->_noiseLevel + d->noiseAveragingRatio * abs(d->hissBuffer[i]);
		if (!isnan(n))
			d->_noiseLevel = n;

		if (d->_noiseLevel > d->noiseThreshold)
		{
			skipped++;
			d->audio[i] = 0.0f;
		}
	}

	if(skipped == d->audio.size())
		d->shouldISkipThis = true;
}

void fm_demod(struct demod_state *d)
{
	for (const complex<float>& sample: d->iq)
	{
		complex<float> a = sample * conj(d->iq_state);
		float num = abs(a);
		if (num > 0.0f)
			a /= num;
		float num2 = arg(a);
		d->audio.push_back(isnan(num2) ? 0.0f : num2 * 1E-05f);
		d->iq_state = sample;
	}
}

void full_demod(struct demod_state *d)
{
	d->down_converter.Process(d->iq);

	if (d->decim_stage_count > 0)
	{
		d->iqdecim->Process(d->iq);
		d->iq.resize(d->iq.size() / pow(2.0, d->decim_stage_count));
	}

	d->iq_filter->Process(d->iq);

	for (complex<float>& c: d->iq)
	{
		c.real(c.real() * 0.01f);
		c.imag(c.imag() * 0.01f);
	}

	d->audio.clear();
	fm_demod(d);

	// nfm only
	if(d->squelch_level)
		ProcessSquelch(d);

	// nfm only
	for (float& a: d->audio)
		a *= 10000.0f;
}

static void rtlsdr_callback(unsigned char* buf, uint32_t len, void *ctx)
{
	struct dongle_state *s = (struct dongle_state *) ctx;
	struct demod_state *d = s->demod_target;

	if (do_exit)
		return;
	if (!ctx)
		return;

	if(s->mute)
	{
		memset(buf, 127, s->mute);
		s->mute = 0;
	}

	pthread_rwlock_wrlock(&d->rw);
	d->iq.clear();
	d->iq.reserve(len / 2);
	for(uint32_t i = 0; i < len; i += 2)
		d->iq.emplace_back(_lutPtr[*(buf + i + 1)], _lutPtr[*(buf + i)]);
	pthread_rwlock_unlock(&d->rw);

	safe_cond_signal(&d->ready, &d->ready_m);
}

static void* dongle_thread_fn(void *arg)
{
	struct dongle_state *s = (struct dongle_state *) arg;
	int capture_rate = rtlsdr_get_sample_rate(s->dev);
	rtlsdr_read_async(s->dev, rtlsdr_callback, s, 0, 2 * (capture_rate / 10));
	return 0;
}

static void* demod_thread_fn(void *arg)
{
	struct demod_state *d = (struct demod_state *) arg;
	struct output_state *o = d->output_target;

	while (!do_exit)
	{
		safe_cond_wait(&d->ready, &d->ready_m);

		pthread_rwlock_wrlock(&d->rw);
		full_demod(d);
		pthread_rwlock_unlock(&d->rw);

		if (d->exit_flag)
			do_exit = 1;

		if (d->squelch_level && d->shouldISkipThis)
		{
			d->shouldISkipThis = false;
			safe_cond_signal(&controller.hop, &controller.hop_m);
			continue;
		}

		pthread_rwlock_wrlock(&o->rw);
		memcpy(o->result, d->audio.data(), d->audio.size() * sizeof(float));
		o->result_len = d->audio.size();
		pthread_rwlock_unlock(&o->rw);

		safe_cond_signal(&o->ready, &o->ready_m);
	}

	return 0;
}

static void* output_thread_fn(void *arg)
{
	struct output_state *s = (struct output_state *) arg;

	while (!do_exit)
	{
		// use timedwait and pad out under runs
		safe_cond_wait(&s->ready, &s->ready_m);

		pthread_rwlock_rdlock(&s->rw);
		fwrite(s->result, sizeof(float), s->result_len, s->file);
		pthread_rwlock_unlock(&s->rw);
	}

	return 0;
}

static void optimal_settings(int freq)
{
	dongle.rate = 1024000; // TODO allow set samplerate
	dongle.freq = freq + dongle.rate / 4;
}

static void* controller_thread_fn(void *arg)
{
	// thoughts for multiple dongles
	// might be no good using a controller thread if retune/rate blocks
	struct controller_state *s = (struct controller_state *) arg;

	/* set up primary channel */
	optimal_settings(s->freqs[0]);
	if (dongle.direct_sampling)
		verbose_direct_sampling(dongle.dev, 1);

	/* Set the frequency */
	verbose_set_frequency(dongle.dev, dongle.freq);
	verbose_set_sample_rate(dongle.dev, dongle.rate);

	while (!do_exit)
	{
		safe_cond_wait(&s->hop, &s->hop_m);
		if (s->freq_len <= 1)
			continue;
		/* hacky hopping */
		s->freq_now = (s->freq_now + 1) % s->freq_len;
		optimal_settings(s->freqs[s->freq_now]);
		rtlsdr_set_center_freq(dongle.dev, dongle.freq);
		dongle.mute = BUFFER_DUMP;
	}

	return nullptr;
}

void frequency_range(struct controller_state *s, char *arg)
{
	char *start, *stop, *step;
	int i;
	start = arg;
	stop = strchr(start, ':') + 1;
	stop[-1] = '\0';
	step = strchr(stop, ':') + 1;
	step[-1] = '\0';
	for(i=(int)atofs(start); i<=(int)atofs(stop); i+=(int)atofs(step))
	{
		s->freqs[s->freq_len] = (uint32_t)i;
		s->freq_len++;
		if (s->freq_len >= FREQUENCIES_LIMIT)
			break;
	}
	stop[-1] = ':';
	step[-1] = ':';
}

void dongle_init(struct dongle_state *s)
{
	s->rate = DEFAULT_SAMPLE_RATE;
	s->gain = AUTO_GAIN; // tenths of a dB
	s->mute = 0;
	s->direct_sampling = 0;
	s->demod_target = &demod;
}

void demod_init(struct demod_state *s)
{
	s->squelch_level = 0;
	s->conseq_squelch = 10;
	s->terminate_on_squelch = 0;// TODO init fir func
	//s->mode_demod = &fm_demod;
	pthread_rwlock_init(&s->rw, NULL);
	pthread_cond_init(&s->ready, NULL);
	pthread_mutex_init(&s->ready_m, NULL);
	s->output_target = &output;

	s->_noiseLevel = 0.0f;
	s->shouldISkipThis = false;
}

void demod_cleanup(struct demod_state *s)
{
	pthread_rwlock_destroy(&s->rw);
	pthread_cond_destroy(&s->ready);
	pthread_mutex_destroy(&s->ready_m);
}

void output_init(struct output_state *s)
{
	pthread_rwlock_init(&s->rw, NULL);
	pthread_cond_init(&s->ready, NULL);
	pthread_mutex_init(&s->ready_m, NULL);
}

void output_cleanup(struct output_state *s)
{
	pthread_rwlock_destroy(&s->rw);
	pthread_cond_destroy(&s->ready);
	pthread_mutex_destroy(&s->ready_m);
}

void controller_init(struct controller_state *s)
{
	s->freqs[0] = 100000000;
	s->freq_len = 0;
	s->edge = 0;
	pthread_cond_init(&s->hop, NULL);
	pthread_mutex_init(&s->hop_m, NULL);
}

void controller_cleanup(struct controller_state *s)
{
	pthread_cond_destroy(&s->hop);
	pthread_mutex_destroy(&s->hop_m);
}

void sanity_checks(void)
{
	if (controller.freq_len == 0) {
		fprintf(stderr, "Please specify a frequency.\n");
		exit(1);
	}

	if (controller.freq_len >= FREQUENCIES_LIMIT) {
		fprintf(stderr, "Too many channels, maximum %i.\n", FREQUENCIES_LIMIT);
		exit(1);
	}

	if (controller.freq_len > 1 && demod.squelch_level == 0) {
		fprintf(stderr, "Please specify a squelch level.  Required for scanning multiple frequencies.\n");
		exit(1);
	}

}// TODO init fir func

int main(int argc, char** argv)
{
	// fill in look-up-table
	float scale = 1.0f / 127.0f;
	for (int i = 0; i < 256; i++)
		_lutPtr[i] = (i - 128) * scale;

	int dev_given = 0;
	int custom_ppm = 0;
	int enable_biastee = 0;
	dongle_init(&dongle);
	demod_init(&demod);
	output_init(&output);
	controller_init(&controller);

	int opt;
	while ((opt = getopt(argc, argv, "d:f:g:l:b:t:p:E:M:hT")) != -1) {
		switch (opt) {
		case 'd':
			dongle.dev_index = verbose_device_search(optarg);
			dev_given = 1;
			break;
		case 'f':
			if (controller.freq_len >= FREQUENCIES_LIMIT)
				break;
			if (strchr(optarg, ':'))
				frequency_range(&controller, optarg);
			else
				controller.freqs[controller.freq_len++] = (uint32_t) atofs(optarg);
			break;
		case 'g':
			dongle.gain = (int)(atof(optarg) * 10);
			break;
		case 'l':
			demod.squelch_level = (int)atof(optarg);
			break;
		case 'b':
			demod.bandwidth = (uint32_t)atofs(optarg);
			break;
		case 't':
			demod.conseq_squelch = (int)atof(optarg);
			if (demod.conseq_squelch < 0) {
				demod.conseq_squelch = -demod.conseq_squelch;
				demod.terminate_on_squelch = 1;
			}
			break;
		case 'p':
			dongle.ppm_error = atoi(optarg);
			custom_ppm = 1;
			break;
		case 'E':
			if (strcmp("edge", optarg) == 0)
				controller.edge = 1;
			else if (strcmp("direct", optarg) == 0)
				dongle.direct_sampling = 1;
			break;
		case 'M':
			/*if (strcmp("fm",  optarg) == 0)
				demod.mode_demod = &fm_demod;
			else if (strcmp("raw",  optarg) == 0)
				demod.mode_demod = &raw_demod;
			else if (strcmp("am",  optarg) == 0)
				demod.mode_demod = &am_demod;
			else if (strcmp("usb", optarg) == 0)
				demod.mode_demod = &usb_demod;
			else if (strcmp("lsb", optarg) == 0)
				demod.mode_demod = &lsb_demod;
			else if (strcmp("wbfm",  optarg) == 0)
			{
				controller.wb_mode = 1;
				demod.mode_demod = &fm_demod;
				demod.bandwidth = 150000;
				demod.squelch_level = 0;
			}*/
			break;
		case 'T':
			enable_biastee = 1;
			break;
		case 'h':
		default:
			usage();
			break;
		}
	}

	sanity_checks();

	/* MY GOVNOCODE */

	int capture_rate = 1024000;
	int freq_offset = capture_rate / 4;

	demod.decim_stage_count = GetDecimationStageCount(capture_rate);

	int audio_sample_rate = capture_rate / pow(2.0, demod.decim_stage_count);

	if(demod.squelch_level)
	{
		demod.noiseAveragingRatio = 30.0f / audio_sample_rate;
		vector<float> bpk = FilterBuilder::MakeBandPassKernel(audio_sample_rate, HissFilterOrder, MinHissFrequency, MaxHissFrequency, WindowType::BlackmanHarris4);
		demod.hissFilter = new FirFilter(bpk, 1);
		demod.noiseThreshold = log10(2 - demod.squelch_level / 100.0) * HissFactor;
	}

	demod.iqdecim = new IQDecimator(demod.decim_stage_count, capture_rate, DecimationFilterType::Baseband);

	vector<float> coeffs = FilterBuilder::MakeLowPassKernel(audio_sample_rate, 1000, demod.bandwidth / 2, WindowType::BlackmanHarris4);
	demod.iq_filter = new IQFirFilter(coeffs);

	demod.down_converter.set_samplerate(capture_rate);
	demod.down_converter.set_frequency(-freq_offset);

	/* END MY GOVNOCODE */

	if (controller.freq_len > 1)
		demod.terminate_on_squelch = 0;
	if (!dev_given)
		dongle.dev_index = verbose_device_search("0");

	if (dongle.dev_index < 0)
		exit(1);

	int r = rtlsdr_open(&dongle.dev, (uint32_t)dongle.dev_index);
	if (r < 0)
	{
		fprintf(stderr, "Failed to open rtlsdr device #%d.\n", dongle.dev_index);
		exit(1);
	}

#ifndef _WIN32
	struct sigaction sigact;
	sigact.sa_handler = sighandler;
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigaction(SIGINT, &sigact, nullptr);
	sigaction(SIGTERM, &sigact, nullptr);
	sigaction(SIGQUIT, &sigact, nullptr);
	sigaction(SIGPIPE, &sigact, nullptr);
#else
	SetConsoleCtrlHandler((PHANDLER_ROUTINE) sighandler, TRUE);
#endif

	/* Set the tuner gain */
	if (dongle.gain == AUTO_GAIN)
	{
		verbose_auto_gain(dongle.dev);
	}
	else
	{
		dongle.gain = nearest_gain(dongle.dev, dongle.gain);
		verbose_gain_set(dongle.dev, dongle.gain);
	}

	/* bias tee */
	rtlsdr_set_bias_tee(dongle.dev, enable_biastee);
	if (enable_biastee)
		fprintf(stderr, "activated bias-T on GPIO PIN 0\n");

	if(custom_ppm)
		verbose_ppm_set(dongle.dev, dongle.ppm_error);

	const char* filename = "-";
	if(argc > optind)
		filename = argv[optind];

	if(strcmp(filename, "-") == 0)
	{
		/* Write samples to stdout */
		output.file = stdout;
#ifdef _WIN32
		_setmode(_fileno(output.file), _O_BINARY);
#endif
	}
	else
	{
		output.file = fopen(filename, "wb");
		if (!output.file)
		{
			fprintf(stderr, "Failed to open %s\n", filename);
			exit(1);
		}
	}

	//r = rtlsdr_set_testmode(dongle.dev, 1);

	/* Reset endpoint before we start reading from it (mandatory) */
	verbose_reset_buffer(dongle.dev);

	pthread_create(&controller.thread, NULL, controller_thread_fn, (void *)(&controller));
	usleep(100000);
	pthread_create(&output.thread, NULL, output_thread_fn, (void *)(&output));
	pthread_create(&demod.thread, NULL, demod_thread_fn, (void *)(&demod));
	pthread_create(&dongle.thread, NULL, dongle_thread_fn, (void *)(&dongle));

	while (!do_exit)
		usleep(100000);

	if (do_exit)
		fprintf(stderr, "\nUser cancel, exiting...\n");
	else
		fprintf(stderr, "\nLibrary error %d, exiting...\n", r);

	rtlsdr_cancel_async(dongle.dev);
	pthread_join(dongle.thread, NULL);
	safe_cond_signal(&demod.ready, &demod.ready_m);
	pthread_join(demod.thread, NULL);
	safe_cond_signal(&output.ready, &output.ready_m);
	pthread_join(output.thread, NULL);
	safe_cond_signal(&controller.hop, &controller.hop_m);
	pthread_join(controller.thread, NULL);

	/* MY GOVNOCODE TOO */
	delete demod.hissFilter;
	delete demod.iqdecim;
	delete demod.iq_filter;
	/* END MY GODNOCODE */

	//dongle_cleanup(&dongle);
	demod_cleanup(&demod);
	output_cleanup(&output);
	controller_cleanup(&controller);

	if (output.file != stdout) {
		fclose(output.file);}

	rtlsdr_close(dongle.dev);
	return r >= 0 ? r : -r;
}

// vim: tabstop=8:softtabstop=8:shiftwidth=8:noexpandtab
