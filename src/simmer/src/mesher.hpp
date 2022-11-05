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

#include "CDT.h"

#include "geometry/line.hpp"


class Mesher
{
public:

             Mesher() = delete;
    virtual ~Mesher() = default;
    
    explicit
    Mesher(IdxType spt) noexcept : spt { spt } {};

    virtual
    std::vector<smr::Edge>
    mesh(std::vector<std::vector<smr::Point>> const & polys) const;
        
    std::vector<TriangleType>
    zerothOrderTriangles(std::vector<std::vector<smr::Point>> const & polys) const;
    
protected:

    IdxType const spt;

    mutable std::vector<TriangleType> tris {};
    
    mutable bool trisFlg {};

};


CDT::TriangleVec
triangulateT(std::vector<CDT::V2d<CrdType>> & vrt, std::vector<CDT::Edge> & edg);

std::vector<CDT::V2d<CrdType>>
triangulateVC(std::vector<CDT::V2d<CrdType>> & vrt, std::vector<CDT::Edge> & edg);

std::pair<std::vector<CDT::V2d<CrdType>>, CDT::TriangleVec>
triangulateVCT(std::vector<CDT::V2d<CrdType>> & vrt, std::vector<CDT::Edge> & edg);

