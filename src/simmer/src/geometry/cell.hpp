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

#include <optional>
#include <vector>

#include "line.hpp"


using BlobType = std::vector<IdxType>;

class Cell
{
public:

             Cell() = delete;
    virtual ~Cell() = default;
    
    Cell(IdxType idx, bool dummy) noexcept;

    void
    addPoly(std::vector<smr::Point> && poly        ,
            std::vector<smr::Line>  && wallsMore   ,
            std::vector<smr::Line>  && susosMore   ,
            std::vector<TriType>    && susosExtMore,
            std::vector<BlobType>   && blobsMore   );
    
    auto         getIdx      () const { return idx     ; }
    auto         isDummy     () const { return dummy   ; }
    auto const & getPolys    () const { return polys   ; }
    auto const & getWalls    () const { return walls   ; }
    auto const & getSusos    () const { return susos   ; }
    auto       & getSusoExts ()       { return susoExts; }
    auto const & getBlobs    () const { return blobs   ; }
    
    std::optional<std::string>
    validate() noexcept;
    
protected:

    IdxType const idx;
    bool    const dummy;
    
    std::vector<std::vector<smr::Point>> polys;

    std::vector<smr::Line> walls;
    std::vector<smr::Line> susos;

    std::vector<TriType>  susoExts;
    std::vector<BlobType> blobs;
};


template <typename T>
auto &
operator+=(std::vector<T> & a, std::vector<T> const & b)
{
    a.reserve(a.size() + b.size());
    a.insert(a.cend(), b.cbegin(), b.cend());
    return a;
}


using PolyType = std::vector<smr::Point>;

bool
polyIntersectionFlag(PolyType const & polyO, PolyType const & polyS) noexcept;


