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

#include "router.hpp"
#include "finder.hpp"


Router::Router(Geometry const & geometry, Finder const & finder)

: geometry { geometry },
  finder   { finder   }
{
    auto const xSize { geometry.getNosoz().size() };

    pathMCSs.resize(xSize);
    distMCSs.resize(xSize);
    dcts    .resize(xSize);

    /* dispatch finder.findLocal .. */
    auto ftr { std::async([this, & finder] { finder.findLocal(pathMCSs, distMCSs); }) };

    /* .. and overlap independent work */
    consolidate();
    formDicts  ();
    
    ftr.get      ();
	patchUp      ();
	populateNexts();
}


IdxType
Router::findLine(IdxType cIdx, smr::Point const & pt) const noexcept
{
    auto const & nosos { geometry.getNosoz()[cIdx] };

    auto const cnt { std::min(static_cast<std::size_t>(DICHI), nosos.size()) };

    auto const vec
    {
        [& nosos, & pt, cnt]
        {
            std::vector<IdxType> vec;
            vec.reserve(nosos.size());

            for (IdxType i {}; i < nosos.size(); i++)
                vec.push_back(i);

            std::partial_sort(vec.begin(), vec.begin() + cnt, vec.end(), [& nosos, & pt] (auto const & a, auto const & b)
                {
                    return euclideanPLDistance(pt, nosos[a]) < euclideanPLDistance(pt, nosos[b]);
                });

            return vec;
        }()
    };

    std::vector<CrdType> dst;
    dst.reserve(cnt);

    for (IdxType i {}; i < cnt; i++)
        if (not geometry.intersectsWalls({ pt, linePoint(nosos[vec[i]]) }, cIdx))
            dst.push_back(euclideanPLDistance(pt, nosos[vec[i]]) + lShrtz[cIdx][vec[i]]);
        else
            dst.push_back(std::numeric_limits<CrdType>::infinity());

    return vec[std::distance(dst.cbegin(), std::min_element(dst.cbegin(), dst.cend()))];
}


/*
 *   assumption  : 0 <= s <= 1
 * | smr::Line.u : position in target cell
 * | smr::Line.v : direction (unit-norm velocity)
 */
std::pair<IdxType, smr::Line>
Router::findCell(std::vector<smr::Line> const & lines,
                 std::vector<DuoType>   const & cells,
                 CrdType                const   s    ) const noexcept
{
    IdxType idx { IdxTypeMax };

    for (IdxType i {}; i < lines.size(); i++)
    {
        auto       const & ln { lines[i]                  };
        smr::Point const   pt { (1 - s) * ln.u + s * ln.v };

        if (geometry.isInsideCell(pt, cells[i].cIdx))
            idx = i;
    }

    auto const & ln { lines[idx] };

    return { idx, { ln.u + s * (ln.v - ln.u), unitVctr(ln.v - ln.u) } };
}


Navi
Router::findVisible(IdxType            cIdx,
                    smr::Point const & pt  ,
                    IdxType            hop ,
                    CrdType    const   cpa ) const
{
    auto const & nosoz { geometry.getNosoz() };

    DuoType duoP { cIdx, findLine(cIdx, pt) };

    [[ maybe_unused ]]
    auto const sIdx { duoP.sIdx };

    /* the two vectors maintain an identical size */
    std::vector<DuoType>    cells { duoP };
    std::vector<smr::Point> tails { pt   };
    /* tails is the translation of pt in all viable cells */

    std::vector<smr::Line> lines { { pt, linePoint(nosoz[duoP.cIdx][duoP.sIdx]) } };

    std::unordered_map<IdxType, std::unordered_set<IdxType>> pseudoz;

    IdxType dmp {};

    while (hop--)
    {
        bool visible { true  };
        bool cTrnsn  { false };  /* cell transition flag */
        bool hitExit { false };
        
        auto const & duoS = nextz[duoP.cIdx][duoP.sIdx];
        
        [[ unlikely ]]
        if (duoS.cIdx != duoP.cIdx)
        {
            cells.push_back(duoS);
            tails.emplace_back(translate(duoP.cIdx, duoS.cIdx, tails.back()));

            subtractInfc(pseudoz, duoP.cIdx, duoP.sIdx);

            cTrnsn = true;
        }
        else if (geometry.isExit(duoS.cIdx, duoS.sIdx))
        {
            if (pseudoz.contains(duoS.cIdx))
                pseudoz[duoS.cIdx].erase(duoS.sIdx);

            hitExit = true;
        }
        
        std::vector<smr::Line> linesT;
        linesT.resize(cells.size());
        
        auto head { linePoint(nosoz[duoS.cIdx][duoS.sIdx]) };
        linesT.back() = { tails.back(), head };

        if (geometry.intersectsWalls(linesT.back(), duoS.cIdx, pseudoz[duoS.cIdx], cpa))
            visible = false;

        if (visible)
        {
            for (auto i { cells.size() - 1 }; i > 0; i--)
            {
                auto const ccIdx { cells[i-1].cIdx };

                head = translate(cells[i].cIdx, ccIdx, head);

                linesT[i-1] = { tails[i-1], head };

                if (geometry.intersectsWalls(linesT[i-1], ccIdx, pseudoz[ccIdx], cpa))
                {
                    visible = false;

                    break;
                }
            }
        }

        if (not visible)
        {
            if (cTrnsn)
                dmp++;
    
            [[ unlikely ]]
            if (hitExit)
                break;

            duoP = duoS;

            continue;
        }
        dmp = 0;
        
        lines = std::move(linesT);

        [[ likely ]]
        if (not cTrnsn)                     /* update the 'sIdx' field */
            cells.back().sIdx = duoS.sIdx;

        duoP = duoS;
        
        [[ unlikely ]]
        if (hitExit)
            break;
    }

    cells.resize(cells.size() - dmp);
    
    return { lines, cells };
}


Navi
Router::findVisibleG(IdxType            cIdx,
                     smr::Point const & pt  ,
                     IdxType            hop ,
                     CrdType    const   cpa ) const
{
    auto const & nosoz { geometry.getNosoz() };
    
    DuoType duoP { cIdx, findLine(cIdx, pt) };

    /* the two vectors maintain an identical size */
    std::vector<DuoType>    cells { duoP };
    std::vector<smr::Point> tails { pt   };

    std::vector<smr::Line> lines { { pt, linePoint(nosoz[duoP.cIdx][duoP.sIdx]) } };
    
    bool visible { true };
    
    while (hop--)
    {
        auto const & duoS = nextz[duoP.cIdx][duoP.sIdx];
        
        bool cTrnsn { false };

        [[ unlikely ]]
        if (duoS.cIdx != duoP.cIdx)
        {
            cells.push_back(duoS);
            tails.emplace_back(translate(duoP.cIdx, duoS.cIdx, tails.back()));
            cTrnsn = true;
        }

        std::vector<smr::Line> linesT;
        linesT.resize(cells.size());
        
        auto head { linePoint(nosoz[duoS.cIdx][duoS.sIdx]) };
        linesT.back() = { tails.back(), head };

        if (geometry.intersectsWalls(linesT.back(), duoS.cIdx, cpa))
            visible = false;

        if (visible)
        {
            for (auto i { cells.size() - 1 }; i > 0; i--)
            {
                head = translate(cells[i].cIdx, cells[i-1].cIdx, head);
                linesT[i-1] = { tails[i-1], head };

                if (geometry.intersectsWalls(linesT[i-1], cells[i-1].cIdx, cpa))
                {
                    visible = false;

                    break;
                }
            }
        }

        [[ unlikely ]]
        if (not visible)
        {
            if (cTrnsn)
                cells.pop_back();
            
            break;
        }

        lines = std::move(linesT);

        duoP = duoS;
        
        [[ unlikely ]]
        if (geometry.isExit(duoS.cIdx, duoS.sIdx))
            break;
    }

    cells.back().sIdx = duoP.sIdx;

    return { lines, cells };
}


smr::Line
Router::translate(IdxType cIdxP, IdxType cIdxS, smr::Line const & l) const noexcept
{
    auto const & d { dcts[cIdxP].at(cIdxS) };

    auto const u { rotate(l.u - d.tP, d.a) + d.tS };
    auto const v { rotate(l.v - d.tP, d.a) + d.tS };

    return { u, v };
}


smr::Point
Router::translate(IdxType cIdxP, IdxType cIdxS, smr::Point const & p) const noexcept
{
    auto const & d { dcts[cIdxP].at(cIdxS) };

    return rotate(p - d.tP, d.a) + d.tS;
}


void
Router::consolidate()
{
    auto const & susoExtz { geometry.getSusoExtz() };
    
    /* temporary bookkeeping containers */
	std::vector<std::unordered_map<IdxType, IdxType>> clonz;
	decltype(clonz)::value_type                       clons;

    for (IdxType i {}; i < susoExtz.size(); i++)
    {
        auto       & susoExts     { susoExtz[i]     };
        auto const   susoExtsSize { susoExts.size() };

        decltype(gIdz)::value_type gIds;
		gIds.reserve(susoExtsSize);
        

        for (IdxType j {}; j < susoExtsSize; j++)
        {
			auto const & [sIdx, cIdx, oIdx] { susoExts[j] };

            if (cIdx >= i)
            {
				gIds.push_back(gIdx);

				quads.push_back({ i, sIdx, IdxTypeMax, IdxTypeMax });

                if (cIdx == i)
                    gEIds.push_back(gIdx);

				clons.insert({ sIdx, gIdx });

                gIdx++;
            }
            else
            {
				auto const gIdxO { clonz[cIdx].at(oIdx) };

				gIds.push_back(gIdxO);

				auto & quad { quads[gIdxO] };
				quad = { quad.cIdxP, quad.sIdxP, i, sIdx };
			}
        }

		clonz.emplace_back(std::move(clons ));
		gIdz .emplace_back(std::move(gIds));
    }
}


void
Router::formDicts()
{
    auto const & susoExtz { geometry.getSusoExtz() };
    auto const & nosoz    { geometry.getNosoz()    };
    
    for (IdxType i {}; i < susoExtz.size(); i++)
    {
        auto const & susoExts { susoExtz[i] };

        auto const pMap { geometry.getPMaps(i) };
        
        for (IdxType j {}; j < susoExts.size(); j++)
        {
            auto const & tri { susoExts[j] };

            /* equality excludes EXIT lines */
            if ((i == tri.cIdx) or dcts[i].contains(tri.cIdx))
                continue;
            
            auto const & lineP { nosoz[i       ][tri.sIdx] };
            auto const & lineS { nosoz[tri.cIdx][tri.oIdx] };

            smr::Point const ptP { lineP.v - lineP.u };
            smr::Point const ptS { lineS.v - lineS.u };

            /* construct the dictionary */
            DctType dctPS;

            /* sign */
            dctPS.s = fELess(0., vctrDot(ptP, ptS)) ? true : false;

            /* translation */
            dctPS.tP = linePoint(lineP);
            dctPS.tS = linePoint(lineS);

            auto const pty { pMap.contains(tri.cIdx) };

            /* rotation */
            /* order of arguments matters */
            dctPS.a = vctrAngle(lineP.v - dctPS.tP, ((pty xor dctPS.s) ? lineS.v : lineS.u) - dctPS.tS);
            
            /* xS = R(a) * (xP - tP) + tS */

            /* dispatch */
            dcts[i].insert({ tri.cIdx, std::move(dctPS) });
        }
    }
}


void
Router::patchUp()
{
	/* for rerouting */
	distM  .clear();
	pathM  .clear();
	distMCS.clear();
	gShrts .clear();
	
    distM.resize(gIdx * gIdx);
    pathM.resize(gIdx * gIdx);

    auto const g
    {
        [this]
        {
            auto const & susoExtz { geometry.getSusoExtz() };
            
            GraphType g;
            g.resize(gIdx);

            for (IdxType i {}; i < susoExtz.size(); i++)
            {
                auto const & susoExts     { susoExtz[i]     };
                auto const & susoExtsSize { susoExts.size() };

                for (IdxType j {}; j < susoExtsSize; j++)
                {
                    auto const gIdxJ { gIdz[i][j] };
                    
                    auto & gIdxJMap { g[gIdxJ] };

                    for (IdxType k {}; k < susoExtsSize; k++)
                    {
                        auto const gIdxK { gIdz[i][k] };

                        auto const wgt { distMCSs[i][susoExts[j].sIdx * susoExtsSize + k] };

                        if (not gIdxJMap.contains(gIdxK))
                            gIdxJMap.insert({ gIdxK, wgt });
                        else
                            gIdxJMap.at(gIdxK) = std::min(gIdxJMap.at(gIdxK), wgt);
                    }
                }
            }

            return g;
        }()
    };
    
    finder.pathFinderGlobal(pathM, distM, g);

	IdxType const xSize { gIdx                               };
	IdxType const ySize { static_cast<IdxType>(gEIds.size()) };

    distMCS.resize(xSize * ySize);

    for (IdxType j {}; j < ySize; j++)
    {
        auto const J { gEIds[j] };

        for (IdxType i {}; i < xSize; i++)
            distMCS[i * ySize + j] = distM[i * xSize + J];
    }
    /* keeping distM for the sake of prospective extensions */
    // distM.clear();
    
	gShrts.reserve(xSize);

	for (IdxType i {}; i < xSize; i++)
    {
		auto const itr
            {
                std::min_element(distMCS.cbegin() + (i + 0) * ySize,
                                 distMCS.cbegin() + (i + 1) * ySize )
            };

		gShrts.push_back(
            {
                gEIds[std::distance(distMCS.cbegin() + i * ySize, itr)], * itr
            });
	}
}


void
Router::populateNexts()
{
	auto const & nosoz { geometry.getNosoz() };

    lShrtz.resize(nosoz.size());

	for (IdxType i {}; i < nosoz.size(); i++)
    {
		auto const & nosos { nosoz[i] };

        lShrtz[i].resize(nosos.size());

		std::vector<DuoType> next;
		next.reserve(nosos.size());

		for (IdxType j {}; j < nosos.size(); j++)
			next.push_back(nextMark(i, j));

        next.shrink_to_fit();
		nextz.emplace_back(std::move(next));
	}

	nextz.shrink_to_fit();
}


DuoType
Router::nextMark(IdxType cIdx, IdxType sIdx) noexcept
{
    /* indices of the next global destination (tentative values) */
	IdxType cIdxD { cIdx };
	IdxType sIdxD {      };

    auto const & susoMaps { geometry.getSusoMaps() };
	auto const & susoMap  { susoMaps[cIdxD]        };
	
    auto ySize { susoMap.size() };

    [[ unlikely ]]
	if (geometry.isSubsolid(cIdxD, sIdx))
    {
        [[ unlikely ]]
        if (geometry.andIsExit(cIdxD, sIdx))
            return { cIdxD, sIdx };

		auto const gIdxS { gIdz[cIdxD][susoMap.at(sIdx).sIdx] };
		auto const gIdxT { gShrts[gIdxS].first                };
		auto const gIdxD { pathM[gIdxS * gIdx + gIdxT]        };

        lShrtz[cIdx][sIdx] = gShrts[gIdxD].second;
        
		auto const & quad { quads[gIdxD]     };
        auto const & trio { susoMap.at(sIdx) };

        /* adjacent/other susoMap */
        auto const & susoA { susoMaps[trio.cIdx] };

        bool switchCell { true };

		if (cIdxD == quad.cIdxP)          /* destination cell is the current */
        {
            [[ likely ]]
            if (trio.cIdx != quad.cIdxS)  /* destination line does NOT share */
            {                             /* /other/ cell with the current   */
                sIdxD = susoMap.at(quad.sIdxP).sIdx;
                switchCell = false;
            }
            else                       /* otherwise, the local shortest path */
            {                          /* may pass through the /other/ cell  */
                auto const dstS { distMCSs[cIdxD    ][sIdx      * ySize        + susoMap.at(quad.sIdxP).sIdx] };
                auto const dstO { distMCSs[trio.cIdx][trio.oIdx * susoA.size() + susoA  .at(quad.sIdxS).sIdx] };

                // if (fELess(dstS, dstO))
                if (dstS < dstO)
                {
                    sIdxD = susoMap.at(quad.sIdxP).sIdx;
                    switchCell = false;
                }
            }
        }
        else if (cIdxD == quad.cIdxS)     /* destination cell is the current */
        {
            [[ likely ]]
            if (trio.cIdx != quad.cIdxP)  /* destination line does NOT share */
            {                             /* /other/ cell with the current   */
                sIdxD = susoMap.at(quad.sIdxS).sIdx;
                switchCell = false;
            }
            else                       /* otherwise, the local shortest path */
            {                          /* may pass through the /other/ cell  */
                auto const dstS { distMCSs[cIdxD    ][sIdx      * ySize        + susoMap.at(quad.sIdxS).sIdx] };
                auto const dstO { distMCSs[trio.cIdx][trio.oIdx * susoA.size() + susoA  .at(quad.sIdxP).sIdx] };

                // if (fELess(dstS, dstO))
                if (dstS < dstO)
                {
                    sIdxD = susoMap.at(quad.sIdxS).sIdx;
                    switchCell = false;
                }
            }
        }
        
		if (switchCell)    /* next destination is in the adjacent/other cell */
        {            
			cIdxD = trio.cIdx;
			sIdx  = trio.oIdx;

			ySize = susoA.size();

			if (cIdxD == quad.cIdxP)
				sIdxD = susoA.at(quad.sIdxP).sIdx;
			else
				sIdxD = susoA.at(quad.sIdxS).sIdx;
		}
	}
    else
    {
		std::vector<CrdType> dist;
		dist.reserve(ySize);

		auto const & distMCST { distMCSs[cIdxD] };

		for (IdxType i {}; i < ySize; i++)
			dist.push_back(distMCST[sIdx * ySize + i] + gShrts[gIdz[cIdxD][i]].second);

		auto const itr { std::min_element(dist.cbegin(), dist.cend()) };

		sIdxD = static_cast<IdxType>(std::distance(dist.cbegin(), itr));

        lShrtz[cIdx][sIdx] = * itr;
	}

	IdxType sIdxM { pathMCSs[cIdxD][sIdx * ySize + sIdxD] };
        
	return { cIdxD, sIdxM };
}


void
Router::subtractInfc(PseudoType & pseudoz,
                     IdxType      cIdxP  ,
                     IdxType      sIdxP  ) const noexcept
{
    auto const & susoExtz { geometry.getSusoExtz() };
    auto const & susoMaps { geometry.getSusoMaps() };

    auto const & blobP { geometry.getBlob(cIdxP, sIdxP) };

    if (pseudoz.contains(cIdxP))
        for (auto const sIdx : blobP)
            pseudoz[cIdxP].erase(sIdx);
    else
    {
        pseudoz[cIdxP] = {};
        
        for (auto const & trio : susoExtz[cIdxP])
            pseudoz[cIdxP].insert(trio.sIdx);
        
        for (auto const sIdx : blobP)
            pseudoz[cIdxP].erase(sIdx);
    }
    
    auto const & [_, cIdxS, sIdxS] { susoMaps[cIdxP].at(sIdxP) };

    auto const & blobS { geometry.getBlob(cIdxS, sIdxS) };
    
    if (pseudoz.contains(cIdxS))
        for (auto const sIdx : blobS)
            pseudoz[cIdxS].erase(sIdx);
    else
    {
        pseudoz[cIdxS] = {};
        
        for (auto const & trio : susoExtz[cIdxS])
            pseudoz[cIdxS].insert(trio.sIdx);
        
        for (auto const sIdx : blobS)
            pseudoz[cIdxS].erase(sIdx);
    }
}


