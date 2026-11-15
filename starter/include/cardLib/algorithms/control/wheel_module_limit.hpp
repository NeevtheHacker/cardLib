// wheel_module_limit.hpp
// Why: the most wheel modules a drive may have, without pulling in speed constants, so
//      sensor and drive headers can size their arrays (SPEC 20.13).
// Owns: MAX_WHEEL_MODULES.
#pragma once

#include <cstddef>

namespace cardLib {

// Up to this many wheel modules per drive.
constexpr std::size_t MAX_WHEEL_MODULES = 8;

}  // namespace cardLib
