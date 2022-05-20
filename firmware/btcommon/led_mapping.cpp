#include <boost/make_shared.hpp>
#include <boost/dynamic_bitset.hpp>
#include <set>
#include <algorithm>    // std::min

//#define OUTPUT_LED_MAPPING

#ifdef OUTPUT_LED_MAPPING
#include <stdio.h>
#include <sstream>
#include <string.h>

static void ToBinString(unsigned int val, char *buff, int bits) {
	for (int i = 0; i < bits; ++i) {
		if (val & 1) {
			*(buff + i) = '1';
		}
		else {
			*(buff + i) = '0';
		}
		val >>= 1;
	}
	*(buff + bits) = '\0';
}
#endif

#include "led_mapping.h"

// This method defines operator() "greater" for this class ==> functor. euqal is defined y !(>) && !(<)
bool LEDMappingInfo::BetterPattern::operator() (const SharedEvaluation& x, const SharedEvaluation& y) const {
	// method is const, this means you can use it
	// with a const greater<T> object
	assert(x);
	assert(y);

	auto itX = x->eval.begin();
	auto itY = y->eval.begin();
	for (;;) {  
		if (itX == x->eval.end()) {
			if (itY == y->eval.end()) {
				return x->pattern > y->pattern;	// "bigger pattern number better"
			}
			else {
				// less "0" better
				return true;
			}
		}
		else {
			if (itY == y->eval.end()) {
				return false;	// "more "0" worse
			}
			else {
				int valueX = *(itX);
				int valueY = *(itY);

				if (valueX > valueY) {
					return true;	// great distance between "0" better
				}
				else if (valueX < valueY) {
					return false;	// short distance worse
				}
				else {
					// equal, compare next less better "0" evaulation
					++itX;
					++itY;
				}
			}
		}
	}
}

void LEDMappingInfo::Load(int patternSize) {
	assert(patternSize >= 8 && patternSize <= 32);

	this->patternSize = patternSize;
	ledInfoCollection.clear();
	patternLEDInfoCollection.clear();
	lcidLEDInfoCollection.clear();

	typedef std::set<SharedEvaluation, BetterPattern> BetterLEDPatternSet;
	BetterLEDPatternSet betterSet;

	for (boost::uint64_t i = (((boost::uint64_t)1)  << patternSize) - 2; i >= 32; --i) {

		SharedEvaluation evaluation(new Evaluation());
		evaluation->pattern = (boost::uint32_t)i;

		GetPatternEvalation(evaluation->pattern, patternSize, evaluation->eval);

		assert(evaluation->eval.size() != 0);			// 0 always exists
		int distance = *(evaluation->eval.begin());		// there are adjacent "0" -- distance between "0"s is 0
		if (distance == 0) {
			continue;
		}

		betterSet.insert(evaluation);
	}

	int ledIndex = 1;
	boost::uint8_t lcid = 1;

#ifdef OUTPUT_LED_MAPPING
    std::stringstream ss;
    ss << "bit_pattern" << patternSize << ".csv";
	FILE* csvFile = fopen(ss.str().c_str(), "wb");
#endif

	for (auto itBetterSet = betterSet.begin(); itBetterSet != betterSet.end(); ++itBetterSet) {
		const SharedEvaluation& evaluation = *itBetterSet;
		boost::uint32_t pattern = evaluation->pattern;
		SharedLEDInfo ledInfo(boost::make_shared<LEDInfo>());
		
		ledInfo->ledIndex = ledIndex;
		if (patternSize == 8) {
			ledInfo->lcid = pattern;
        }
        else {
            ledInfo->lcid = lcid;
        }
		ledInfo->bitPattern = pattern;

		ledInfoCollection[ledIndex] = ledInfo;
		patternLEDInfoCollection[pattern] = ledInfo;
		boost::uint32_t rlcid = GetRouterLCIDIndex(ledInfo->lcid);
		lcidLEDInfoCollection[rlcid] = ledInfo;

		++ledIndex;
		++lcid;

#ifdef OUTPUT_LED_MAPPING
		if (csvFile != NULL) {
			char buffer[256];
			ToBinString(pattern, buffer, patternSize);

			fprintf(csvFile, "\n%4d,%5d,%4X,%16s,%3d", ledInfo->ledIndex, pattern, pattern, buffer, (int)ledInfo->lcid);
		}
#else

        if (ledIndex > 256) {
            break;
        }
#endif

	} 

#ifdef OUTPUT_LED_MAPPING
	if (csvFile != NULL) {
		fprintf(csvFile, "\n");
		fclose(csvFile);
	}
#endif
}

void LEDMappingInfo::Save() {
}

SharedLEDInfo LEDMappingInfo::GetLEDInfoByIndex(boost::uint32_t ledIndex) {
	auto it = ledInfoCollection.find(ledIndex);
	if (it == ledInfoCollection.end()) {
		return NULL;
	}
	return it->second;
}

SharedLEDInfo LEDMappingInfo::GetLEDInfoByLCID(boost::uint16_t lcid, boost::uint16_t routerId) {
	auto it = lcidLEDInfoCollection.find(GetRouterLCIDIndex(lcid, routerId));
	if (it == lcidLEDInfoCollection.end()) {
		return NULL;
	}
	return it->second;
}

SharedLEDInfo LEDMappingInfo::GetLEDInfoByPattern(boost::uint32_t pattern) {
	auto it = patternLEDInfoCollection.find(pattern);
	if (it == patternLEDInfoCollection.end()) {
		return NULL;
	}
	return it->second;
}

boost::uint32_t LEDMappingInfo::GetRouterLCIDIndex(boost::uint16_t lcid, boost::uint16_t routerId) {
	return ((boost::uint32_t)routerId * 0x10000) + lcid;
}

void LEDMappingInfo::GetPatternEvalation(boost::uint32_t pattern, int numbits, std::multiset<int>& eval) {
	assert(numbits >= 8);

	boost::dynamic_bitset<> value(numbits, (boost::uint32_t)pattern);

	// check adjacent 0
	for (int pos = 0; pos < numbits; ++pos) {
		if (value.test(pos) == true) {
			// evaluate "0" bit only
			continue;
		}

		int distance = 0;
		for (int leftPos = 1; leftPos < numbits; ++leftPos) {
			int location = (pos + leftPos) % numbits;			// in a circular way
			if (value.test(location) == false) {			// found its left side "0"
				break;
			}
			distance++;
		}
		eval.insert(distance);		// or need to save what bit too?
		if (distance == 0) {
			return;			// adjacent "0" -- not valid LED
		}
	}	
}
