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

#include <future>

#include "cell.hpp"


Cell::Cell(IdxType idx, bool dummy) noexcept

    : idx   { idx   },
      dummy { dummy }
{}


void
Cell::addPoly(std::vector<smr::Point> && poly        ,
              std::vector<smr::Line>  && wallsMore   ,
              std::vector<smr::Line>  && susosMore   ,
              std::vector<TriType>    && susoExtsMore,
              std::vector<BlobType>   && blobsMore   )
{
    polys.emplace_back(std::move(poly));

    walls    += wallsMore;
    susos    += susosMore;
    susoExts += susoExtsMore;
    blobs    += blobsMore;
}


std::optional<std::string>
Cell::validate() noexcept
{
    for (IdxType i {}; i < polys.size(); i++)
    {
        auto ftr
        {
            std::async(
                [this, i]
                {
                    for (IdxType j { i + 1 }; j < polys.size(); j++)
                        if (polyIntersectionFlag(polys[i], polys[j]))
                            return true;

                    return false;
                }
            )
        };

        auto const & poly  { polys[i]    };
        auto const   pSize { poly.size() };

        if (pSize < 3)
            return fmt::format("non-poly in cell {}", idx);

        for (IdxType i {}; i < (pSize - 1); i++)
            if (fELess(lineNorm({ poly[i], poly[i+1] }), smr::Param::CPA))
                return fmt::format("line norm <= smr::Param::CPA in cell {}", idx);
        
        if (fELess(lineNorm({ poly.front(), poly.back() }), smr::Param::CPA))
            return fmt::format("line norm <= smr::Param::CPA in cell {}", idx);
        
        /* test for self-intersection */
        for (IdxType i {}; i < (pSize - 2); i++)                          /* next point not on previous line */
            if (fELess(pointLineDistance(poly[i+2], { poly[i+1], poly[i] }), smr::Param::CPA))
                return fmt::format("a polygon intersects itself, within smr::Param::CPA, in cell {}", idx);
        
        for (IdxType i {}; i < (pSize - 1); i++)                  /* non-adjacent lines (except w/ last one) */
            for (IdxType j { i + 2 }; j < (pSize - 2); j++)
                if (intersectionFlagCPA({ poly[i], poly[i+1] }, { poly[j], poly[j+1] }))
                    return fmt::format("a polygon intersects itself, within smr::Param::CPA, in cell {}", idx);
        
        smr::Line const line { poly.front(), poly.back() };                                     /* last line */
        for (IdxType i { 1 }; i < (pSize - 2); i++)
            if (intersectionFlagCPA({ poly[i], poly[i+1] }, line))
                return fmt::format("a polygon intersects itself, within smr::Param::CPA, in cell {}", idx);

        if (ftr.get())
            return fmt::format("polygons intersect, within smr::Param::CPA, in cell {}", idx);
    }

    return {};
}


inline bool
polyIntersectionFlag(PolyType const & polyO, PolyType const & polyS) noexcept
{
    for (auto itrS { polyS.cbegin() }; itrS != (polyS.cend() - 1); itrS++)
    {
        smr::Line const lineS { * itrS, * (itrS + 1) };

        for (auto itrO { polyO.cbegin() }; itrO != (polyO.cend() - 1); itrO++)
            if (intersectionFlagCPA(lineS, { * itrO, * (itrO + 1) } ))
                return true;

        if (intersectionFlagCPA(lineS, { * (polyO.cend() - 1), * (polyO.cbegin()) } ))
            return true;
    }

    smr::Line const lineS { * (polyS.cend() - 1), * polyS.cbegin() };

    for (auto itr { polyO.cbegin() }; itr != (polyO.cend() - 1); itr++)
        if (intersectionFlagCPA(lineS, { * itr, * (itr + 1) } ))
            return true;

    if (intersectionFlagCPA(lineS, { * (polyO.cend() - 1), * (polyO.cbegin()) } ))
        return true;

    return false;
}

