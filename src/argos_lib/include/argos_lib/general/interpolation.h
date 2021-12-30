/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <algorithm>
#include <array>

namespace argos_lib {

  /**
   * @brief Point that helps generate an interpolation map
   *
   * @tparam T Internal type.  Typically float or double
   */
  template <class T>
  struct InterpMapPoint {
    T inVal;
    T outVal;

    constexpr InterpMapPoint(T in, T out) : inVal(in), outVal(out) {}

    constexpr bool operator<(const InterpMapPoint<T>& other) { return inVal < other.inVal; }
    constexpr bool operator==(const InterpMapPoint<T>& other) { return inVal == other.inVal; }
  };

  template <class T>
  constexpr bool operator<(const InterpMapPoint<T>& a, const T& b) {
    return a.inVal < b;
  }

  template <class T>
  constexpr bool operator<(const T& a, const InterpMapPoint<T>& b) {
    return a < b.inVal;
  }

  /**
   * @brief Performs linear interpolation of a value based on a set of input->output mapping points
   *
   * @tparam T Internal type of values being interpolated.  Typically float or double
   * @tparam size Number of elements in interpolation map
   */
  template <class T, int size>
  class InterpolationMap {
   public:
    InterpolationMap() = delete;
    /**
     * @brief Constructs new interpolation map
     *
     * @param initArray Interpolation points.  Must be sorted by input value with smallest element first.
     */
    constexpr InterpolationMap(std::array<InterpMapPoint<T>, size> initArray) : m_mapArray(initArray) {
      // assert(("Map must contain at least one value.", !initArray.empty()));
      // assert(("Map values must be sorted.", std::is_sorted(initArray.cbegin(), initArray.cend())));
    }

    /**
     * @brief Generate interpolated output based on input
     *
     * @param inVal Input value to remap
     * @return Interpolated value
     */
    constexpr T Map(const T inVal) const {
      if (inVal >= m_mapArray.back().inVal) {
        return m_mapArray.back().outVal;
      } else if (inVal <= m_mapArray.front().inVal) {
        return m_mapArray.front().outVal;
      } else {
        auto afterPoint{std::lower_bound(m_mapArray.cbegin(), m_mapArray.cend(), inVal)};
        auto beforePoint{std::prev(afterPoint)};
        const auto lerpPct = (inVal - beforePoint->inVal) / (afterPoint->inVal - beforePoint->inVal);
        return beforePoint->outVal + lerpPct * (afterPoint->outVal - beforePoint->outVal);
      }
    }

    /**
     * @brief Shorthand for InterpolationMap::Map()
     *
     * @copy_doc argos_lib::InterpolationMap::Map()
     */
    constexpr T operator()(const T inVal) { return Map(inVal); }

   private:
    std::array<InterpMapPoint<T>, size> m_mapArray;
  };

}  // namespace argos_lib
