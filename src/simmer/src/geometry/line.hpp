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

#include <array>

#include "../support.hpp"


namespace smr {

struct Line
{
    /* lex: { (u < v), (u.x < u.y) } */
    Point u {}; // end-point
    Point v {}; // end-point
};

    
struct Edge
{
    Edge() = delete;
    
    Edge(Point   const & u      ,
         Point   const & v      ,
         IdxType         idx    ,
         IdxType         thisNbr,
         IdxType         thatNbr);

    /* lex: { (u < v), (u.x < u.y) } */
    Point u {}; // end-point
    Point v {}; // end-point

    IdxType idx {};

    // each edge can have at most four nbrs
    // zero flags /null/ (see ctor)
    std::array<IdxType, 4> nbrs;
};

    
[[ nodiscard ]]
bool operator==(Line const & lhs, Line const & rhs) noexcept;
[[ nodiscard ]]
bool operator< (Line const & lhs, Line const & rhs) noexcept;
[[ nodiscard ]]
bool operator<=(Line const & lhs, Line const & rhs) noexcept;
    
[[ nodiscard ]]
bool operator==(Edge const & lhs, Edge const & rhs) noexcept;
[[ nodiscard ]]
bool operator< (Edge const & lhs, Edge const & rhs) noexcept;
[[ nodiscard ]]
bool operator<=(Edge const & lhs, Edge const & rhs) noexcept;

    
struct Param
{
    /* Closest Point of Approach */
    static inline CrdType CPA { .000001 };
};
    
}  /* namespace smr */


inline CrdType
lineNorm(smr::Line const & l) noexcept
{
    return euclideanDistance(l.u, l.v);
}


bool
intersectionFlag(smr::Line const & pr, smr::Line const & qs) noexcept;

bool
intersectionFlagCPA(smr::Line const & l0, smr::Line const & l1) noexcept;

CrdType
intersectionMark(smr::Line const & pr, smr::Line const & qs) noexcept;

CrdType
nonIntSegmentDistance(smr::Line  const & k, smr::Line const & l) noexcept;

CrdType
pointLineDistance(smr::Point const & k, smr::Line const & l) noexcept;

