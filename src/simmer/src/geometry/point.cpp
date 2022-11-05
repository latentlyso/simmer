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

#include "point.hpp"


namespace smr
{    
    bool
    operator==(smr::Point const & lhs, smr::Point const & rhs) noexcept
    {
        return fEqual(lhs.x, rhs.x) and fEqual(lhs.y, rhs.y);
    }


    bool
    operator<(smr::Point const & lhs, smr::Point const & rhs) noexcept
    {
        return fLess(lhs.x, rhs.x) or (fEqual(lhs.x, rhs.x) and fLess(lhs.y, rhs.y));
    }


    bool
    operator<=(smr::Point const & lhs, smr::Point const & rhs) noexcept
    {
        return fLess(lhs.x, rhs.x) or (fEqual(lhs.x, rhs.x) and fELess(lhs.y, rhs.y));
    }

    
    Point
    operator+(smr::Point const & lhs, smr::Point const & rhs) noexcept
    {
        return { (lhs.x + rhs.x), (lhs.y + rhs.y) };
    }

    
    Point
    operator-(smr::Point const & lhs, smr::Point const & rhs) noexcept
    {
        return { (lhs.x - rhs.x), (lhs.y - rhs.y) };
    }

    
    Point
    operator*(CrdType const & lhs, smr::Point const & rhs) noexcept
    {
        return { (lhs * rhs.x), (lhs * rhs.y) };
    }

}   /* namespace smr */
