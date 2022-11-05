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

#include "geometry.hpp"
#include "spawner.hpp"


using Navi = std::tuple<std::vector<smr::Line>, std::vector<DuoType>>;

class Finder;

class Router
{
public:

             Router() = delete;
    virtual ~Router() = default;
    
    Router(Geometry const & geometry, Finder const & finder);

    IdxType
    findLine(IdxType cIdx, smr::Point const & pt) const noexcept;
    
    std::pair<IdxType, smr::Line>
    findCell(std::vector<smr::Line> const & lines,
             std::vector<DuoType>   const & cells,
             CrdType                const   s    ) const noexcept;
    
    Navi
    findVisible(IdxType            cIdx                  ,
                smr::Point const & pt                    ,
                IdxType            hop                   ,
                CrdType    const   cpa  = smr::Param::CPA) const;

    Navi
    findVisibleG(IdxType            cIdx                  ,
                 smr::Point const & pt                    ,
                 IdxType            hop                   ,
                 CrdType    const   cpa  = smr::Param::CPA) const;

    smr::Line
    translate(IdxType cIdxP, IdxType cIdxS, smr::Line const & l) const noexcept;

    smr::Point
    translate(IdxType cIdxP, IdxType cIdxS, smr::Point const & p) const noexcept;
    
protected:

    void consolidate  ();
    void formDicts    ();
    void patchUp      ();
    void populateNexts();

    DuoType
    nextMark(IdxType cIdx, IdxType sIdx) noexcept;

    using PseudoType = std::unordered_map<IdxType, std::unordered_set<IdxType>>;

    void
    subtractInfc(PseudoType & pseudoz,
                 IdxType      cIdxP  ,
                 IdxType      sIdxP  ) const noexcept;

    IdxType gIdx {};
    
    Geometry const & geometry;
    Finder   const & finder  ;

    /* local matrices */
    std::vector<std::vector<IdxType>> pathMCSs;
    std::vector<std::vector<CrdType>> distMCSs;

    /* global matrix */
    std::vector<IdxType> pathM  ;
    std::vector<CrdType> distM  ;
    std::vector<CrdType> distMCS;

    std::vector<QudType> quads;

    /* global indices ('gIdx') of EXIT lines */
    std::vector<IdxType> gEIds;

    std::vector<std::vector<IdxType>> gIdz;

    // the distance of the closest EXIT line to a subsolid line;
    // used to speed up finding the shortest exit distance for
    // a nonsolid line in a given cell
    std::vector<std::pair<IdxType, CrdType>> gShrts;

    /* the distance of each nonsolid line to the nearest exit */
    std::vector<std::vector<CrdType>> lShrtz;

    // store the next line on the path of a nonsolid line
    // to nearest exit
    std::vector<std::vector<DuoType>> nextz;

    std::vector<std::unordered_map<IdxType, DctType>> dcts;
    
public:

    static IdxType constexpr DICHI { 7 };
};

