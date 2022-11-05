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

#include "actuator.hpp"


Actuator::Actuator(IdxType            nIdx    ,
                   IdxType            cIdx    ,
                   smr::Point         pos     ,
                   Geometry   const & geometry,
                   Router     const & router  )

    : idx       { gIdx++   },
      nIdx      { nIdx     },
      cIdx      { cIdx     },
      pos       { pos      },
      geometry  { geometry },
      router    { router   }
{
    path.emplace_back(CellPathType { cIdx, {} });
    path.back().second.emplace_back(pos);
}


void
Actuator::operator()(std::queue<IdxType>       & que ,
                     std::mutex                & queM,
                     ViewType            const   iVue,
                     ViewType                    oVue,
                     std::mutex                & vueM)
{
    [[ maybe_unused ]]
    auto const nbrs
    {
        [this, & iVue, & vueM]
        {
            std::unique_lock const lock { vueM };
            
            if (iVue.contains(cIdx))
                       return iVue.at(cIdx);
            return std::vector<smr::Line> {};
        }()
    };

    auto const [lines, cells] { router.findVisible(cIdx, pos, IdxTypeMax) };

    auto const & lastLine    { cells.back()                                  };
    auto const   exitInSight { geometry.isExit(lastLine.cIdx, lastLine.sIdx) };

    auto const dptA { lineNorm(lines[0]) };                   // Avaiable 'dpt'

    if (exitInSight)
        dpt = dptM;
    else
        dpt = std::min(dptA, dptM);
    
    if ((not (exitInSight and fELess(dptA, dpt))) and (not geometry.isDummy(cIdx)))  // else, it is /out/
    {
        auto const where { router.findCell(lines, cells, dpt / dptA) };
        
        pos = where.second.u;
        vel = where.second.v;
    
        auto const cIdxT { cells[where.first].cIdx };
        
        if (cIdx != cIdxT)
            path.emplace_back(CellPathType { cIdxT, {} });
        path.back().second.emplace_back(pos);
        
        cIdx = cIdxT;

        {
            std::unique_lock const lock { vueM };
            oVue[cIdx].push_back({ pos, dpt * vel});
        }
        
        {
            std::unique_lock const lock { queM };
            que.push(idx);
        }
    }
}


