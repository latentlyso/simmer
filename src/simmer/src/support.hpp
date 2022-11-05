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

#include <functional>
#include <numeric>
#include <numbers>
#include <string_view>
#include <string>

#include "fmt/format.h"

#include "geometry/point.hpp"


struct DctType
{
    // dual INFC lines have the same
    // lexicographic orientation if s(ign) is 'true'
    bool s {};

    /* translation */
    smr::Point tP {};
    smr::Point tS {};

    /* rotation a(ngle) */
    CrdType a {};
};


struct TriangleType
{
    smr::Point u {};
    smr::Point v {};
    smr::Point w {};
};


inline CrdType
vctrDot(smr::Point const & p, smr::Point const & q) noexcept
{
    return (p.x * q.x + p.y * q.y);
}


inline CrdType
vctrCross(smr::Point const & p, smr::Point const & q) noexcept
{
    return (p.x * q.y - p.y * q.x);
}


inline smr::Point
unitVctr(smr::Point const & p) noexcept
{
    return { (1 / std::sqrt(std::pow(p.x, 2) + std::pow(p.y, 2))) * p };
}


inline smr::Point
rotate(smr::Point const & p, CrdType a) noexcept
{
    return { std::cos(a) * p.x - std::sin(a) * p.y, std::sin(a) * p.x + std::cos(a) * p.y };
}



/* order of arguments matters */
inline CrdType
vctrAngle(smr::Point const & k, smr::Point const & l) noexcept
{
    /* vector norms */
    CrdType kn { std::sqrt(std::pow(k.x, 2) + std::pow(k.y, 2)) };
    CrdType ln { std::sqrt(std::pow(l.x, 2) + std::pow(l.y, 2)) };

    auto const Pi  { std::numbers::pi };

    auto const a { std::asin(((k.x * l.y) - (k.y * l.x)) / (kn * ln)) };
    auto const s { fELess(0., (k.x * l.x) + (k.y * l.y))              };

    if (s)
        return a;

    if (fELess(0., a))
        return (Pi - a);
    else
        return -(Pi + a);
}


template <typename T>
smr::Point
linePoint(T const & line) noexcept
{
    return { std::midpoint(line.u.x, line.v.x), std::midpoint(line.u.y, line.v.y) };
}


inline CrdType
euclideanDistance(smr::Point const & a, smr::Point const & b) noexcept
{
    return std::pow( std::pow(a.x - b.x, 2.) + std::pow(a.y - b.y, 2.), .5);
}


/* used for parsing line/cell idx */
inline IdxType
xAsIdxType(char const * t, IdxType v = 0) noexcept
{
    if(t and (*t))
        return static_cast<IdxType>(atoll(t));
    return v;
}


/* used for parsing line color */
inline LCType
xAsLCType(char  const * t,
          LCType        v = static_cast<LCType>(LineColor::SOLD)) noexcept
{
    if(t and (* t))
        return static_cast<LCType>(atoi(t));
    return v;
}


inline CrdType
xAsDouble(char const * t, CrdType v = 0.) noexcept
{
    if(t and (* t))
        return atof(t);
    return v;
}


template <typename T>
CrdType
euclideanLLDistance(T const & a, T const & b) noexcept
{
    auto const ax { std::midpoint(a.u.x, a.v.x) };
    auto const ay { std::midpoint(a.u.y, a.v.y) };

    auto const bx { std::midpoint(b.u.x, b.v.x) };
    auto const by { std::midpoint(b.u.y, b.v.y) };

    return std::pow( std::pow(ax - bx, 2.) + std::pow(ay - by, 2.), .5);
}


template <typename P, typename L>
CrdType
euclideanPLDistance(P const & p, L const & l) noexcept
{
    auto const mx { std::midpoint(l.u.x, l.v.x) };
    auto const my { std::midpoint(l.u.y, l.v.y) };

    auto wavg { (  euclideanDistance(p, l.u)      * 1
                 + euclideanDistance(p, l.v)      * 1
                 + euclideanDistance(p, {mx, my}) * 2
                ) / 4 };

    return wavg;
}


template <typename T>
void
orderPoints(T & line) noexcept
{
    auto & u { line.u };
    auto & v { line.v };

    if ( !(u <= v) )
        std::swap(u, v);
}


// adpated from:
// https://en.cppreference.com/w/cpp/utility/format/format
template <typename... Args>
std::string
dyna_print(std::string_view rt_fmt_str, Args && ... args) noexcept
{
    return fmt::vformat(rt_fmt_str, fmt::make_format_args(args...));
}


