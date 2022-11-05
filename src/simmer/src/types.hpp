/*
 * Copyright (c) 2022 Shahir Mowlaei
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial
 * portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT.  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

#include <cstdint>
#include <cmath>
#include <limits>


using CrdType = double;
using LCType  = std::uint8_t;
using IdxType = std::uint64_t;


auto constexpr IdxTypeMax { std::numeric_limits<IdxType>::max() };


/* 'subsolid' := { LineColor < SOLD } */
enum class LineColor : LCType
{
    INFC, // 0
    EXIT, // 1
    SOLD, // 2
    META, // 3

    INVD  // 4
};


struct DuoType
{
    IdxType cIdx {};
    IdxType sIdx {};
};


struct TriType
{
    IdxType sIdx {};
    IdxType cIdx {};
    IdxType oIdx {};
};


[[nodiscard]]
inline bool
operator<(TriType const & lhs, TriType const & rhs)
{
    return (lhs.sIdx < rhs.sIdx);
}


struct QudType
{
    IdxType cIdxP {};
    IdxType sIdxP {};
    IdxType cIdxS {};
    IdxType sIdxS {};
};


/*
 * CrdType comparison operators
 * adpated from: M. Gregoire, Professional C++
 * also see https://en.cppreference.com/w/cpp/types/numeric_limits/epsilon
 */
template <std::floating_point T>
inline bool
fEqual(T x, T y, std::uint32_t ulp = 2) noexcept
{
	return    fabs(x - y) <= std::numeric_limits<T>::epsilon() * std::fabs(x + y) * ulp
		   or fabs(x - y) <  std::numeric_limits<T>::min();
}


template <std::floating_point T>
inline bool
fLess(T x, T y, std::uint32_t ulp = 2) noexcept
{
    return    (y - x) >  std::numeric_limits<T>::epsilon() * std::fabs(x + y) * ulp
		   or (y - x) >= std::numeric_limits<T>::min();
}


template <std::floating_point T>
inline bool
fELess(T x, T y, std::uint32_t ulp = 2) noexcept
{
    return fLess(x, y, ulp) or fEqual(x, y, ulp);
}

