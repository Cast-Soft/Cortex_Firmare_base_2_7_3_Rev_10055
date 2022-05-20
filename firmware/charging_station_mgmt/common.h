// common.h
// Copyright (c) CAST Group of Companies Inc., 1994-2011. All rights reserved.

#ifndef BLACKTRAX_COMM_COMMON_H_
#define	BLACKTRAX_COMM_COMMON_H_
#include <boost/cstdint.hpp>
#include <vector>
#include <boost/shared_ptr.hpp>
/// <summary>
/// A macro to disallow the copy constructor and operator= functions. 
/// This should be used in the private: declarations for a class.
/// </summary>
#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
  TypeName(const TypeName&);               \
  void operator=(const TypeName&)

typedef boost::int32_t CommContext;

namespace blacktrax {
namespace comm {

struct Stub {                ///<  for library encapsualtion
    virtual ~Stub() {}
};

}};

#endif // BLACKTRAX_COMM_COMMON_H_

