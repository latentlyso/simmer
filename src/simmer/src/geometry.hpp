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

#include <unordered_set>

#include "augmenter.hpp"
#include "geometry/cell.hpp"


class Mesher;

class Geometry
{
public:

             Geometry() = delete;
    virtual ~Geometry() = default;
    
    Geometry(Mesher const & mesher, bool vFlg = true) noexcept;

    void
    addParityFlag(IdxType cIdxP, IdxType cIdxS) noexcept;
    
    std::optional<std::string>
    processCell(Cell && cell) noexcept;

    /* called after 'Parser' augments 'Geometry' */
    std::optional<std::string>
    finalize();

    /* bool (*)(cIdx, sIdx) */
    bool isInterface(IdxType, IdxType) const;
    bool isSubsolid (IdxType, IdxType) const;
    bool andIsExit  (IdxType, IdxType) const;  // called inside 'isSubsolid' blocks
    bool isExit     (IdxType, IdxType) const;

    bool
    isInsideCell(smr::Point const & p   ,
                 IdxType            cIdx) const noexcept;

    bool
    isInsideCellX(smr::Point const & p                     ,
                  IdxType            cIdx                  ,
                  CrdType            pad  = smr::Param::CPA) const noexcept;
    
    bool
    intersectsWalls(smr::Line const & l                    ,
                    IdxType           cIdx                 ,
                    CrdType           cpa = smr::Param::CPA) const noexcept;

    bool
    intersectsWalls(smr::Line                   const & l                    ,
                    IdxType                             cIdx                 ,
                    std::unordered_set<IdxType> const & pseudos              ,
                    CrdType                             cpa = smr::Param::CPA) const noexcept;
    
    std::vector<IdxType>
    linesPerCell() const;

    auto const & getNosoz   () const { return nosoz   ; }
    auto const & getSusoExtz() const { return susoExtz; }
    auto const & getSusoMaps() const { return susoMaps; }
    auto const & getWallz   () const { return wallz   ; }
    auto const & getNbrz    () const { return nbrz    ; }
    auto const & getCMapR   () const { return cMapR   ; }

    auto const & getBlob (IdxType cIdx, IdxType sIdx) const noexcept
    {
        return blobz[cIdx][blobMaps[cIdx].at(sIdx)];
    }

    std::unordered_set<IdxType>
    getPMaps (IdxType idx) const noexcept
    {
        if (pMaps.contains(idx))
            return pMaps.at(idx);

        return {};
    }
    
    auto
    isDummy (IdxType cIdx) const noexcept { return dummys.contains(cIdx); }

protected:

    std::optional<std::string>
    validate() const;

    void patchUp          () noexcept;
    void shrink           () noexcept;
    void constructSusoMaps()         ;

    virtual void
    processCellExt([[ maybe_unused ]] Cell & cell)
    {
        return;
    }

    virtual void
    finalizeExt()
    {
        return;
    }

    Augmenter const augmenter;

    /* validation flag */
    bool const vFlg;

    /* global counter for indexing cells */
    IdxType cellIdx {};

    std::unordered_set<IdxType> dummys;
    
    // triangles of cells from non-recursive meshing
    // used to test if a point is inside a given cell
    std::vector<std::vector<TriangleType>> triz;

    /* sets of wall/solid lines of cells */
    std::vector<std::vector<smr::Line>> wallz;

    // sets of extended data { sIdx, cIdx, oIdx } associated
    // with subsolid lines of cells
    std::vector<std::vector<TriType>> susoExtz;

    /* sets of non-solid lines of cells */
    std::vector<std::vector<smr::Line>> nosoz;

    // forward-backward(R) nominal-sequential cell
    // indexing dictionary
    std::unordered_map<IdxType, IdxType> cMap ;
    std::unordered_map<IdxType, IdxType> cMapR;

    /* forward nominal-sequential 'sIdx' dictionary */
    std::vector<std::unordered_map<IdxType, IdxType>> sMaps;

    // IdxType is susoExtz[][].sIdx
    // TriType is susoExtz[][idx] with 'sIdx' replaced with 'idx'
    std::vector<std::unordered_map<IdxType, TriType>> susoMaps;

    std::vector<std::vector<BlobType>>                blobz   ;
    std::vector<std::unordered_map<IdxType, IdxType>> blobMaps;

    std::vector<std::vector<IdxType>> nbrz;

    // map of sets of cells with non-zero parity flag wrt to
    // the key cell index (Router::formDicts)
    std::unordered_map<IdxType, std::unordered_set<IdxType>> pMaps;
};

bool
isInsideTriangle (smr::Point const & p, TriangleType const & t) noexcept;


