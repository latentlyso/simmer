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

#include "line.hpp"


namespace smr {

bool
operator==(Line const & lhs, Line const & rhs) noexcept
{
    return (lhs.u == rhs.u) and (lhs.v == rhs.v);
}


bool
operator<(Line const & lhs, Line const & rhs) noexcept
{
    return (lhs.u < rhs.u) or ((lhs.u == rhs.u) and (lhs.v < rhs.v));
}


bool
operator<=(Line const & lhs, Line const & rhs) noexcept
{
    return (lhs.u < rhs.u) or ((lhs.u == rhs.u) and (lhs.v <= rhs.v));
}


Edge::Edge(Point   const & u      ,
           Point   const & v      ,
           IdxType         idx    ,
           IdxType         thisNbr,
           IdxType         thatNbr)
    
    : u   { u       },
      v   { v       },
      idx { 1 + idx },  // idx == 0 is reserved for the /null/ flag in
                        // nbrs; see Augmenter
      nbrs {{ 1 + thisNbr, 1 + thatNbr, 0, 0 }}
{}


bool
operator==(Edge const & lhs, Edge const & rhs) noexcept
{
    return (lhs.u == rhs.u) and (lhs.v == rhs.v);
}


bool
operator<(Edge const & lhs, Edge const & rhs) noexcept
{
    return (lhs.u < rhs.u) or ((lhs.u == rhs.u) and (lhs.v < rhs.v));
}


bool
operator<=(Edge const & lhs, Edge const & rhs) noexcept
{
    return (lhs.u < rhs.u) or ((lhs.u == rhs.u) and (lhs.v <= rhs.v));
}

} /* namespace smr */


/* adpated from https://stackoverflow.com/a/565282 */
bool
intersectionFlag(smr::Line const & pr, smr::Line const & qs) noexcept
{
    auto const & p { pr.u        };
    auto const & r { pr.v - pr.u };

    auto const & q { qs.u        };
    auto const & s { qs.v - qs.u };

    auto const qp  { q - p            };
    auto const rs  { vctrCross(r , s) };
    auto const qpr { vctrCross(qp, r) };

    if (fEqual(rs, 0.))
    {
        if (fEqual(qpr, 0.))
        {
            auto const rr { vctrDot(r, r) };

            auto const t0 {      vctrDot(qp, r) / rr };
            auto const t1 { t0 + vctrDot(s , r) / rr };

            if (fELess(0., t0) and fELess(t0, 1.))
                return true;

            if (fELess(0., t1) and fELess(t1, 1.))
                return true;

            if (fLess(std::min(t0, t1), 0.) and fLess(1., std::max(t0, t1)))
                return true;

            return false;
        }
        else
            return false;
    }

    auto const t { vctrCross(qp, s) / rs };
    auto const u { qpr              / rs };

    if (fELess(0., u) and fELess(u, 1.) and fELess(0., t) and fELess(t, 1.))
        return true;

    return false;
}


bool
intersectionFlagCPA(smr::Line const & l0, smr::Line const & l1) noexcept
{
    if (intersectionFlag(l0, l1))
        return true;

    if (fELess(nonIntSegmentDistance(l0, l1), smr::Param::CPA))
        return true;

    return false;
}


/*
 *   the order of lines matters
 * | assumption: the lines intersect but are not collinear
 * | the distance of intersection point from the first ('u') point of
 *   the first line is returned
 */
CrdType
intersectionMark(smr::Line const & pr, smr::Line const & qs) noexcept
{
    auto const & s { qs.v - qs.u };

    return (vctrCross(qs.u - pr.u, s) / vctrCross(pr.v - pr.u, s));
}


/** Computes the distance between two non-intersecting 'smr::Line' segments
 *
 * | The minimum of the distance of the four endpoints
 *   wrt the /other/ line is returned
 * | does NOT handle degenrate line segments
 */
CrdType
nonIntSegmentDistance(smr::Line const & k, smr::Line const & l) noexcept
{
    auto const kv { k.v - k.u };
    auto const lv { l.v - l.u };
        
    auto const kn2 { std::pow(kv.x, 2) + std::pow(kv.y, 2) };
    auto const ln2 { std::pow(lv.x, 2) + std::pow(lv.y, 2) };

    auto const kpu { vctrDot(lv, k.u - l.u) / ln2 };
    auto const kpv { vctrDot(lv, k.v - l.u) / ln2 };
    auto const lpu { vctrDot(kv, l.u - k.u) / kn2 };
    auto const lpv { vctrDot(kv, l.v - k.u) / kn2 };

    CrdType kpud;
    if (fLess(kpu, 0.))
        kpud = lineNorm({ k.u, l.u });
    else if (fLess(1., kpu))
        kpud = lineNorm({ k.u, l.v });
    else
        kpud = lineNorm({ k.u, l.u + kpu * lv });

    CrdType kpvd;
    if (fLess(kpv, 0.))
        kpvd = lineNorm({ k.v, l.u });
    else if (fLess(1., kpv))
        kpvd = lineNorm({ k.v, l.v });
    else
        kpvd = lineNorm({ k.v, l.u + kpv * lv });

    CrdType lpud;
    if (fLess(lpu, 0.))
        lpud = lineNorm({ l.u, k.u });
    else if (fLess(1., lpu))
        lpud = lineNorm({ l.u, k.v });
    else
        lpud = lineNorm({ l.u, k.u + lpu * kv });

    CrdType lpvd;
    if (fLess(lpv, 0.))
        lpvd = lineNorm({ l.v, k.u });
    else if (fLess(1., lpv))
        lpvd = lineNorm({ l.v, k.v });
    else
        lpvd = lineNorm({ l.v, k.u + lpv * kv });
    
    return std::min({ kpud, kpvd, lpud, lpvd });
}


CrdType
pointLineDistance(smr::Point const & p, smr::Line const & l) noexcept
{
    auto const lv  { l.v - l.u                             };
    auto const ln2 { std::pow(lv.x, 2) + std::pow(lv.y, 2) };
    auto const t   { vctrDot(lv, p - l.u) / ln2            };
    
    if (fLess(t, 0.))
        return euclideanDistance(p, l.u);
    else if (fLess(1., t))
        return euclideanDistance(p, l.v);
    else
        return euclideanDistance(p, l.u + t * lv);
}

