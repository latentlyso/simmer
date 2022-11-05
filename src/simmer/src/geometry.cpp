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

#include <utility>
#include <queue>

#include "geometry.hpp"


Geometry::Geometry(Mesher const & mesher, bool vFlg) noexcept
        
    : augmenter { mesher },
      vFlg      { vFlg   }
{}


void
Geometry::addParityFlag(IdxType cIdxP, IdxType cIdxS) noexcept
{
    pMaps[cIdxP].insert(cIdxS);
    pMaps[cIdxS].insert(cIdxP);
}


std::optional<std::string>
Geometry::processCell(Cell && cell) noexcept
{
    auto const cellIdxN { cell.getIdx() };
    
    if (not cellIdxN)
        return {};

    if (vFlg)
    {
        if (cMap.contains(cellIdxN))
            return fmt::format("duplicate cell index, {}, encountered", cellIdxN);
        
        if (auto const err { cell.validate() })
            return err.value();
    }
    
    auto [edges, tris] { augmenter.augment(cell.getPolys(), cell.getWalls()) };
    
    triz .emplace_back(std::move(tris         ));
    wallz.emplace_back(std::move(cell.getWalls()));

    if (cell.isDummy())
        dummys.insert(cellIdx);

    cMapR.insert({ cellIdx,  cellIdxN  });
    cMap .insert({ cellIdxN, cellIdx++ });

    std::vector<IdxType> nbrs;
    nbrs.reserve(4 * edges.size());
    
    for (auto & e : edges)
        for (auto & nbr : e.nbrs)
            nbrs.emplace_back(std::move(nbr));
    nbrs.shrink_to_fit();
    nbrz.emplace_back(std::move(nbrs));

    std::vector<smr::Line> nosos;
    nosos.reserve(edges.size());
    for (auto const & e : edges)
        nosos.push_back({ e.u, e.v });
    nosos.shrink_to_fit();
    
    auto const & susos    { cell.getSusos   () };
    auto       & susoExts { cell.getSusoExts() };

    std::unordered_map<IdxType, IdxType> sMap;
    sMap.reserve(susos.size());

    std::unordered_map<IdxType, IdxType> SCMap;

    if (vFlg)
        for (IdxType i {}; i < susos.size(); i++)
        {
            auto const itr { std::find(nosos.cbegin(), nosos.cend(), susos[i]) };

            auto & trio { susoExts[i] };

            /* inequality ignores EXIT lines */
            if ((cellIdxN != trio.cIdx) and sMap.contains(trio.sIdx))
                return fmt::format("duplicate s-index {} in cell {}", trio.sIdx, cellIdxN);

            auto const idx
            {
                static_cast<IdxType>(std::distance(nosos.cbegin(), itr))
            };

            sMap.insert({ std::exchange(trio.sIdx, idx), idx });
            SCMap.insert({ trio.sIdx, trio.cIdx });
        }
    else
        for (IdxType i {}; i < susos.size(); i++)
        {
            auto const itr { std::find(nosos.cbegin(), nosos.cend(), susos[i]) };

            auto const idx
            {
                static_cast<IdxType>(std::distance(nosos.cbegin(), itr))
            };

            auto & trio { susoExts[i] };

            sMap.insert({ std::exchange(trio.sIdx, idx), idx });
            SCMap.insert({ trio.sIdx, trio.cIdx });
        }

    /* augment 'blobz' for the current cell */
    auto const & blobsC { cell.getBlobs() };

    std::vector<BlobType> blobs;
    
    decltype(blobMaps)::value_type blobMap {};

    IdxType bIdx {};
    for (auto const & blobC : blobsC)
    {
        blobs.emplace_back(BlobType {});

        auto cIdx { SCMap.at(sMap.at(blobC.front())) };
        
        for (IdxType i {}; i < (blobC.size() - 1); i++)
        {
            auto const sIdx { sMap.at(blobC[i]) };

            if (SCMap.at(sIdx) != cIdx)
            {
                blobs.emplace_back(BlobType {});
                bIdx++;
            }
            
            blobs.back().push_back(sIdx);
            blobMap.insert({ sIdx, bIdx });
        }

        /* complete the cycle */
        
        auto const sIdxL { sMap.at(blobC.back()) };

        auto const cIdxL { SCMap.at(sIdxL) };
        
        if (cIdxL != cIdx)
        {
            auto const sIdxF { sMap.at(blobC.back()) };
            
            if (cIdxL == SCMap.at(sIdxF))
            {
                auto const bIdxF { blobMap[sIdxF] };
                blobs[bIdxF].push_back(sIdxL);
                blobMap.insert({ sIdxL, bIdxF });
            }
            else
            {
                blobs.emplace_back(BlobType {});
                bIdx++;

                blobs.back().push_back(sIdxL);
                blobMap.insert({ sIdxL, bIdx });
            }
        }
        else
        {
            blobs.back().push_back(sIdxL);
            blobMap.insert({ sIdxL, bIdx });
        }
        
        bIdx++;
    }
    
    blobz   .emplace_back(std::move(blobs  ));
    blobMaps.emplace_back(std::move(blobMap));
    
    nosoz.emplace_back(std::move(nosos));
    sMaps.emplace_back(std::move(sMap ));

    susoExts.shrink_to_fit();

    susoExtz.emplace_back(std::move(susoExts));

    /* process extra cell attributes */
    processCellExt(cell);

    return {};
}


std::optional<std::string>
Geometry::finalize()
{
    if (vFlg)
        if (auto const err { validate() })
            return err.value();
    
    patchUp          ();
    shrink           ();
    constructSusoMaps();
    finalizeExt      ();

    return {};
}


/** Checks whether the line is INFC (interface) */
bool
Geometry::isInterface(IdxType cIdx, IdxType sIdx) const
{
    auto const & susoMap { susoMaps[cIdx] };

    if (susoMap.contains(sIdx) and (susoMap.at(sIdx).cIdx != cIdx))
        return true;
    else
        return false;
}


/** Checks whether the line is subsolid */
bool
Geometry::isSubsolid(IdxType cIdx, IdxType sIdx) const
{
    auto const & susoMap { susoMaps[cIdx] };

    if (susoMap.contains(sIdx))
        return true;
    else
        return false;
}


/** Checks whether a subsolid line is also an EXIT */
bool
Geometry::andIsExit(IdxType cIdx, IdxType sIdx) const
{
    auto const & susoMap { susoMaps[cIdx] };

    if (susoMap.at(sIdx).cIdx == cIdx)
        return true;
    else
        return false;
}


/** Checks whether the line is EXIT */
bool
Geometry::isExit(IdxType cIdx, IdxType sIdx) const
{
    auto const & susoMap { susoMaps[cIdx] };

    if (susoMap.contains(sIdx) and (susoMap.at(sIdx).cIdx == cIdx))
        return true;
    else
        return false;
}


bool
Geometry::isInsideCell(smr::Point const & p, IdxType cIdx) const noexcept
{
    for (auto const & t : triz[cIdx])
        if (isInsideTriangle(p, t))
            return true;

    /* no need for wall check */
    for (auto const & tri : susoExtz[cIdx])
        if (fELess(pointLineDistance(p, nosoz[cIdx][tri.sIdx]), smr::Param::CPA))
            return true;
            
    return false;
}


bool
Geometry::isInsideCellX(smr::Point const & p, IdxType cIdx, CrdType pad) const noexcept
{
    bool any { false };
    for (auto const & t : triz[cIdx])
        if (isInsideTriangle(p, t))
            any = true;
    if (not any)
        return false;

    for (auto const & w : wallz[cIdx])
        if (fELess(pointLineDistance(p, w), pad))
            return false;

    return true;
}


bool
Geometry::intersectsWalls(smr::Line const & l, IdxType cIdx, CrdType cpa) const noexcept
{
    for (auto const & w : wallz[cIdx])
        if (intersectionFlag(l, w) or fELess(nonIntSegmentDistance(l, w), cpa))
            return true;

    return false;
}


bool
Geometry::intersectsWalls(smr::Line                   const & l      ,
                          IdxType                             cIdx   ,
                          std::unordered_set<IdxType> const & pseudos,
                          CrdType                             cpa    ) const noexcept
{
    for (auto const & w : wallz[cIdx])
        if (intersectionFlag(l, w) or fELess(nonIntSegmentDistance(l, w), cpa))
            return true;

    auto const & nosos { nosoz[cIdx] };
    
    for (auto const idx : pseudos)
    {
        auto const & w { nosos[idx] };

        if (intersectionFlag(l, w) or fELess(nonIntSegmentDistance(l, w), cpa))
            return true;
    }
    
    return false;
}


/** Returns the number of non-solid lines in a cell */
std::vector<IdxType>
Geometry::linesPerCell() const
{
    std::vector<IdxType> lpc;
    lpc.reserve(nosoz.size());

    for (auto const & noso : nosoz)
        lpc.push_back(noso.size());

    lpc.shrink_to_fit();
    
    return lpc;    
}


/** Second level grammar and EXIT reachability
 *  tests for cells of the partition (Geometry)
 */
std::optional<std::string>
Geometry::validate() const
{
    /* isolated cells: it is an error if 'isos' remains non-empty */
    std::unordered_set<IdxType> isos;
    for (IdxType i {}; i < cellIdx; i++)
        isos.insert(i);
    
    std::vector<bool> mask;
    mask.resize(cellIdx);

    std::queue<IdxType> que;

    std::unordered_map<IdxType, std::unordered_set<IdxType>> nbrz;
    
    for (IdxType i {}; i < cellIdx; i++)
    {
        auto const & susoExts { susoExtz[i] };
        auto const & sMap     { sMaps[i]    };

        bool directExit {};
        
        std::unordered_set<IdxType> nbrs;

        for (IdxType j {}; j < susoExts.size(); j++)
        {
            auto const & trioP { susoExts[j] };

            [[ unlikely ]]
            if (not cMap.contains(trioP.cIdx))
                return fmt::format("invalid target cell index encountered in cell {}:\nsIdx : ?\ncIdx : {}\noIdx : {}\n",
                                   cMapR.at(i), trioP.cIdx, trioP.oIdx);

            auto const cIdxS { cMap.at(trioP.cIdx) };

            [[ unlikely ]]
            if (cIdxS == i)  /* ignore EXIT lines */
            {
                directExit = true;
                
                continue;
            }

            nbrs.insert(cIdxS);
            
            auto const sIdxPF { (* std::find_if(sMap.cbegin(), sMap.cend(),
                                                [& trioP] (auto const & pair) { return pair.second == trioP.sIdx; })).first };

            [[ unlikely ]]
            if (not sMaps[cIdxS].contains(trioP.oIdx))
                return fmt::format("invalid target line index encountered in cell {}:\nsIdx : {}\ncIdx : {}\noIdx : {}\n",
                                   cMapR.at(i), sIdxPF, trioP.cIdx, trioP.oIdx);
            
            auto const sIdxS { sMaps[cIdxS].at(trioP.oIdx) };

            auto const & susoExtsS { susoExtz[cIdxS] };

            auto const itr { std::find_if(susoExtsS.cbegin(), susoExtsS.cend(),
                                         [& sIdxS] (TriType trio)
                                             { return (trio.sIdx == sIdxS); }) };

            [[ unlikely ]]
            if (itr == susoExtsS.cend())
                return fmt::format("forward dual not found in cell {}:\nsIdx : {}\ncIdx : {}\noIdx : {}\n",
                                   cMapR.at(i), sIdxPF, trioP.cIdx, trioP.oIdx);

            auto const & trioS { * itr };

            [[ unlikely ]]
            if ((i != cMap.at(trioS.cIdx)) or (trioS.oIdx != sIdxPF) or (not cMap.contains(trioS.cIdx)))
                return fmt::format("roundtrip mismatch encountered in cell {}\n", cMapR.at(i)) +
                    fmt::format("source trio:\nsIdx : {}\ncIdx : {}\noIdx : {}\n",
                               sIdxPF, trioP.cIdx, trioP.oIdx) +
                    fmt::format("target trio:\nsIdx : {}\ncIdx : {}\noIdx : {}\n",
                               trioP.oIdx, trioS.cIdx, trioS.oIdx);
        }

        nbrz[i] = std::move(nbrs);
        
        if (directExit)
            que.push(i);
    }

    while (not que.empty())
    {
        auto const & u { que.front() };

        mask[u] = true;
        isos.erase(u);
        
        for (auto const & nbr : nbrz.at(u))
            if (not mask[nbr])
                que.push(nbr);

        que.pop();
    }
    
    if (not isos.empty())
    {
        std::string error { fmt::format("the following cells have no route to an exit:") };
        
        for (auto const & iso : isos)
            error += fmt::format(" {},", cMapR.at(iso));
        error.pop_back();

        return error;
    }

    return {};
}


void
Geometry::patchUp() noexcept
{
    for (IdxType i {}; i < susoExtz.size(); i++)
    {
        auto & susoExt { susoExtz[i] };

        for (IdxType j {}; j < susoExt.size(); j++)
        {
            auto & cIdx { susoExt[j].cIdx };
            cIdx = cMap[cIdx];

            auto & oIdx { susoExt[j].oIdx };
            oIdx = sMaps[cIdx][oIdx];
        }
    }

    /* update pMaps */
    decltype(pMaps) pMapsT;
    for (IdxType i {}; i < nosoz.size(); i++)
        for (auto const & cIdxS : pMaps[cMapR[i]])
            pMapsT[i].insert(cMap[cIdxS]);
    pMaps = std::move(pMapsT);
}


void
Geometry::shrink() noexcept
{
    wallz   .shrink_to_fit();
    nosoz   .shrink_to_fit();
    nbrz    .shrink_to_fit();
    susoExtz.shrink_to_fit();

    /* unless there is subsequent use for sMaps.. */
    sMaps.clear();
}


void
Geometry::constructSusoMaps()
{
    susoMaps.reserve(susoExtz.size());

    for (auto const & susoExts : susoExtz)
    {
        std::unordered_map<IdxType, TriType> susoMap;
        susoMap.reserve(susoExts.size());

        for (IdxType i {}; i < susoExts.size(); i++)
        {
            auto const & t { susoExts[i] };
            susoMap.insert({ t.sIdx, { i, t.cIdx, t.oIdx } });
        }

        susoMaps.emplace_back(std::move(susoMap));
    }
}


bool
isInsideTriangle(smr::Point const & p, TriangleType const & t) noexcept
{
    auto const & p1 { t.u };
    auto const & p2 { t.v };
    auto const & p3 { t.w };

    auto const l1 { (p.x-p1.x)*(p2.y-p1.y) - (p2.x-p1.x)*(p.y-p1.y) };
    auto const l2 { (p.x-p2.x)*(p3.y-p2.y) - (p3.x-p2.x)*(p.y-p2.y) };
    auto const l3 { (p.x-p3.x)*(p1.y-p3.y) - (p1.x-p3.x)*(p.y-p3.y) };
   
    return (fELess(0., l1) and fELess(0., l2) and fELess(0., l3))
        or (fELess(l1, 0.) and fELess(l2, 0.) and fELess(l3, 0.));
}

