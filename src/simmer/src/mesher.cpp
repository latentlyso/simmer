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

#include "mesher.hpp"


std::vector<smr::Edge>
Mesher::mesh(std::vector<std::vector<smr::Point>> const & polys) const
{
    std::vector<CDT::V2d<CrdType>> vrt;
    std::vector<CDT::Edge>         edg;

    CDT::VertInd xCnt {};
    CDT::VertInd yCnt {};
    
    for (auto const & poly : polys)
    {
        for (auto itr { poly.cbegin() }; itr != poly.cend(); itr++)
        {
            CDT::V2d<CrdType> v { itr -> x, itr -> y };
            vrt.push_back(v);

            if ((itr + 1) != poly.cend())
            {
                CDT::Edge e { yCnt, yCnt + 1 };
                edg.push_back(e);
            }
            else
            {
                CDT::Edge e { yCnt, xCnt };
                edg.push_back(e);
            }

            yCnt++;
        }

        xCnt = yCnt;
    }

    /* first triangulation */
    auto [vrtT, triVec] { triangulateVCT(vrt, edg) };

    /* extract vertices */
    vrt = std::move(vrtT);

    /* set 'tris' AND 'trisFlg' */
    tris.clear();
    tris.reserve(triVec.size());

    for (auto const & t : triVec)
    {
        auto const & u { vrt[t.vertices[0]] };
        auto const & v { vrt[t.vertices[1]] };
        auto const & w { vrt[t.vertices[2]] };

        tris.emplace_back(TriangleType { { u.x, u.y }, { v.x, v.y }, { w.x, w.y } });
    }

    trisFlg = true;

    /* recurse for positive 'spt' */
    if (spt)
    {
        /* all but the last recursion */
        for (IdxType i { spt - 1 }; i; i--)
            vrt = triangulateVC(vrt, edg);

        /* last recursion */
        triVec = triangulateT(vrt, edg);
    }

    std::vector<smr::Edge> edges;
    edges.reserve(3 * triVec.size());

    for (IdxType i {}; i < triVec.size(); i++)
    {
        auto const & t { triVec[i] };

        auto const & u { vrt[t.vertices[0]] };
        auto const & v { vrt[t.vertices[1]] };
        auto const & w { vrt[t.vertices[2]] };

        smr::Point const up { u.x, u.y };
        smr::Point const vp { v.x, v.y };
        smr::Point const wp { w.x, w.y };

        /* idx == 0 is reserved for /null/ in 'smr::Edge' ctor */
        smr::Edge e0 { up, vp, 3 * i + 0, 3 * i + 1, 3 * i + 2 };
        smr::Edge e1 { vp, wp, 3 * i + 1, 3 * i + 2, 3 * i + 0 };
        smr::Edge e2 { wp, up, 3 * i + 2, 3 * i + 0, 3 * i + 1 };

        orderPoints(e0);
        orderPoints(e1);
        orderPoints(e2);
        
        edges.emplace_back(std::move(e0));
        edges.emplace_back(std::move(e1));
        edges.emplace_back(std::move(e2));
    }

    edges.shrink_to_fit();

    return edges;
}


std::vector<TriangleType>
Mesher::zerothOrderTriangles(std::vector<std::vector<smr::Point>> const & polys) const
{
    if (trisFlg)
        return tris;
    
    std::vector<CDT::V2d<CrdType>> vrt;
    std::vector<CDT::Edge>         edg;

    CDT::VertInd xCnt {};
    CDT::VertInd yCnt {};
    
    for (auto const & poly : polys)
    {
        for (auto itr { poly.cbegin() }; itr != poly.cend(); itr++)
        {
            CDT::V2d<CrdType> v { itr -> x, itr -> y };
            vrt.push_back(v);

            if ((itr + 1) != poly.cend())
            {
                CDT::Edge e { yCnt, yCnt + 1 };
                edg.push_back(e);
            }
            else
            {
                CDT::Edge e { yCnt, xCnt };
                edg.push_back(e);
            }

            yCnt++;
        }

        xCnt = yCnt;
    }

    auto const triVec { triangulateT(vrt, edg) };

    tris.clear();
    tris.reserve(triVec.size());

    for (auto const & t : triVec)
    {
        auto const & u { vrt[t.vertices[0]] };
        auto const & v { vrt[t.vertices[1]] };
        auto const & w { vrt[t.vertices[2]] };
        
        tris.emplace_back(TriangleType { { u.x, u.y }, { v.x, v.y }, { w.x, w.y } });
    }
    
    return tris;
}


CDT::TriangleVec
triangulateT(std::vector<CDT::V2d<CrdType>> & vrt, std::vector<CDT::Edge> & edg)
{
    CDT::Triangulation<CrdType> cdt;
    
    cdt.insertVertices(vrt);
    cdt.insertEdges(edg);
    cdt.eraseOuterTrianglesAndHoles();
    
    return cdt.triangles;
}


std::vector<CDT::V2d<CrdType>>
triangulateVC(std::vector<CDT::V2d<CrdType>> & vrt, std::vector<CDT::Edge> & edg)
{
    CDT::Triangulation<CrdType> cdt;
    
    cdt.insertVertices(vrt);
    cdt.insertEdges(edg);
    cdt.eraseOuterTrianglesAndHoles();

    auto const triVec { cdt.triangles };
    auto       vrtc   { cdt.vertices  };

    for (auto const & t : triVec)
    {
        auto const & u { vrt[t.vertices[0]] };
        auto const & v { vrt[t.vertices[1]] };
        auto const & w { vrt[t.vertices[2]] };

        auto const vwx { std::midpoint(v.x, w.x) };
        auto const vwy { std::midpoint(v.y, w.y) };

        auto const cx { std::lerp(u.x, vwx, 2. / 3) };
        auto const cy { std::lerp(u.y, vwy, 2. / 3) };
        
        vrtc.emplace_back(CDT::V2d<CrdType> { static_cast<CrdType>(cx), static_cast<CrdType>(cy) });
    }
    
    return vrtc;
}


std::pair<std::vector<CDT::V2d<CrdType>>, CDT::TriangleVec>
triangulateVCT(std::vector<CDT::V2d<CrdType>> & vrt, std::vector<CDT::Edge> & edg)
{
    CDT::Triangulation<CrdType> cdt;
    
    cdt.insertVertices(vrt);
    cdt.insertEdges(edg);
    cdt.eraseOuterTrianglesAndHoles();

    auto const triVec { cdt.triangles };
    auto       vrtc   { cdt.vertices  };

    for (auto const & t : triVec)
    {
        auto const & u { vrt[t.vertices[0]] };
        auto const & v { vrt[t.vertices[1]] };
        auto const & w { vrt[t.vertices[2]] };

        auto const vwx { std::midpoint(v.x, w.x) };
        auto const vwy { std::midpoint(v.y, w.y) };

        auto const cx { std::lerp(u.x, vwx, 2. / 3) };
        auto const cy { std::lerp(u.y, vwy, 2. / 3) };
        
        vrtc.emplace_back(CDT::V2d<CrdType> { static_cast<CrdType>(cx), static_cast<CrdType>(cy) });
    }
    
    return { vrtc, triVec };
}


template <typename T>
auto &
operator+=(std::vector<T> & a, std::unordered_set<T> const & b)
{
    a.reserve(a.size() + b.size());
    a.insert(a.cend(), b.cbegin(), b.cend());
    return a;
}

