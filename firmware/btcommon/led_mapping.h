#ifndef LED_MAPPING_H
#define LED_MAPPING_H

#include <map>
#include <set>

#include <boost/serialization/export.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/tracking.hpp>
#include <boost/serialization/shared_ptr.hpp>

#include <boost/cstdint.hpp>
#include <boost/shared_ptr.hpp>


struct LEDInfo {
	boost::uint32_t ledIndex;		// unique ID that presents to user
	boost::uint32_t bitPattern;		// pattern of LED emitting. Uses lower bits
	boost::uint8_t lcid;				// LED Communication ID: for beacon -- router communication only

	LEDInfo() : ledIndex(0), bitPattern(0), lcid(0) {} 
    template<class Archive> void serialize(Archive & ar, unsigned int version) {
		ar & ledIndex;
		ar & bitPattern;
		ar & lcid;
	}
};

typedef boost::shared_ptr<LEDInfo> SharedLEDInfo;

typedef std::map<boost::uint32_t, SharedLEDInfo> LEDInfoCollection;		// Key can be ledIndex, lcid + router id or bitPattern


// define a collection of LEDs
struct LEDMappingInfo {
private:
	struct Evaluation {
		boost::uint32_t pattern;
		std::multiset<int> eval;
	};

	typedef boost::shared_ptr<Evaluation> SharedEvaluation;
	
	struct BetterPattern {
		// This method defines operator() for this class
		// you can do: greater<int> op; op(x,y);
		bool operator() (const SharedEvaluation& x, const SharedEvaluation& y) const;
	};

private:
	LEDInfoCollection patternLEDInfoCollection;
	LEDInfoCollection lcidLEDInfoCollection;
	LEDInfoCollection ledInfoCollection;

	// helpers
	boost::uint32_t GetRouterLCIDIndex(boost::uint16_t lcid, boost::uint16_t routerId = 0);

public:
	boost::uint8_t patternSize;		// bit pattern size

	LEDMappingInfo() : patternSize(8) {} 
    template<class Archive> void serialize(Archive & ar, unsigned int version) {
		ar & patternSize;
		ar & ledInfoCollection;
	}

	void Load(int patternSize);
	void Save();

	SharedLEDInfo GetLEDInfoByIndex(boost::uint32_t ledIndex);
	SharedLEDInfo GetLEDInfoByLCID(boost::uint16_t lcid, boost::uint16_t routerId = 0);		// lcid: LED Communication ID. router ID: default is 0
	SharedLEDInfo GetLEDInfoByPattern(boost::uint32_t pattern);

private:

	// evaluation a pattern. It return a set of distances of a "0" bit to left side of another "0" bit
	// if found conective 0, it does not continue and return.
	void GetPatternEvalation(boost::uint32_t pattern, int numbits, std::multiset<int>& eval);

	// > 0: eval1 bettern; 0 qual; < 0: worst
	int BetterSet(const std::multiset<int>& eval1, const std::multiset<int>& eval2); 
};

#endif