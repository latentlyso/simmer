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

#include "geometry/line.hpp"


class Mesher;

class Augmenter
{
public:
    
     Augmenter() = delete;
    ~Augmenter() = default;
    
    Augmenter(Mesher const & mesher) noexcept : mesher { mesher } {}
    
    std::pair<std::vector<smr::Edge>, std::vector<TriangleType>>
    augment(std::vector<std::vector<smr::Point>> const & polys,
            std::vector<smr::Line>               const & walls) const;
    
private:

    Mesher const & mesher;
};


std::vector<smr::Edge>
subtractLines(std::vector<smr::Edge>       && edges,
              std::vector<smr::Line> const &  walls);

void
sortNCorrectNbrs(std::vector<smr::Edge> & edgesC);

std::vector<smr::Edge>
formUniques(std::vector<smr::Edge> & edges);

void
uniqeNRename(std::vector<smr::Edge> &  edges ,
             std::vector<smr::Edge> && edgesU);

