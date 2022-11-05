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

#include "augmenter.hpp"
#include "mesher.hpp"


std::pair<std::vector<smr::Edge>, std::vector<TriangleType>>
Augmenter::augment(std::vector<std::vector<smr::Point>> const & polys,
                   std::vector<smr::Line>               const & walls) const
{
    auto edges { subtractLines(mesher.mesh(polys), walls) };
    
    sortNCorrectNbrs(edges);

    uniqeNRename(edges, formUniques(edges));

    return { edges, mesher.zerothOrderTriangles(polys) };
}


std::vector<smr::Edge>
subtractLines(std::vector<smr::Edge>       && edgesC,
              std::vector<smr::Line> const &  walls )
{
    for (auto const & line : walls)
    {
		smr::Edge const qeury { line.u, line.v, 0, 0, 0 };             // temp/find ctor
        auto const itr { std::find(edgesC.begin(), edgesC.end(), qeury) };

        auto & edge { * itr };
        
        edge.idx = 0;                                         // signal SOLD line (wall)

        for (int j {}; j < 2; j++)                 // count on compiler's loop unrolling
            if (IdxType const nbrIdx { edge.nbrs[j] })
                edgesC[nbrIdx - 1].nbrs[(j + 1) % 2] = 0;        // -1 for'idx' shift +1
	}

    std::vector<smr::Edge> edges;
    edges.reserve(edgesC.size() - walls.size());

    for (auto & edge : edgesC)
        if (edge.idx)
			edges.push_back(edge);

    return edges;
}


void
sortNCorrectNbrs(std::vector<smr::Edge> & edges)
{
    std::sort(edges.begin(), edges.end());

    auto const m
    {
        [& edges]
        {
        
        std::unordered_map<IdxType, IdxType> m;
        m.reserve(edges.size());
        for (IdxType i {}; i < edges.size(); i++)
            m.insert({ edges[i].idx, i });

        return m;
        }()
    };

    // correct the record of nbrs of
    //   1.         the remaining one of duplicate edges
    //   2. nbrs of the leaving   one of duplicate edges
    for (IdxType i {}; i < (edges.size() - 1); i++)
    {
        auto & e0 = edges[i  ];
        auto & e1 = edges[i+1];

        if (e0 == e1)                              // middle edge of the diamond
        {
            for (char j {}; j < 2; j++)    // count on compiler's loop unrolling
            {
                auto const & nCIdx = e1.nbrs[j];
                e0.nbrs[2+j] = nCIdx;

                if (nCIdx)
                {
                    // the search /has to/ be idx-based since duplicates (with
                    // identical endpoints) are present; it's also more
                    // efficient than coordinate-based search.
                    // auto & nbrs { edges[m.at(nCIdx)].nbrs };
                    auto & nbrs { edges[m.at(nCIdx)].nbrs };

                    // exactly one match is replaced
                    auto itr { std::find(nbrs.begin(), nbrs.end(), e1.idx) };
                    // if (itr != nbrs.end())
                        * itr = e0.idx;
                }
            }

            i++;                                      // skip the duplicate (e1)
        }
    }
}


std::vector<smr::Edge>
formUniques(std::vector<smr::Edge> & edges)
{
    std::vector<smr::Edge> edgesU;  // unique edges
    edgesU.reserve(edges.size());

    IdxType dups {};
    
    for (IdxType i {}; i < (edges.size() - 1); i++)
    {
        auto & e0 = edges[i  ];
        auto & e1 = edges[i+1];

        if (e0 == e1)               // middle edge of the diamond
        {                  
            i++;                       // skip the duplicate (e1)
            dups++;
        }

        edgesU.push_back(e0);
    }
    
    auto last { edges.end() - 1 };
    if ( *(last - 1) != *last )
        edgesU.push_back(*last);

    edgesU.shrink_to_fit();

    return edgesU;
}


void
uniqeNRename(std::vector<smr::Edge> &  edges ,
             std::vector<smr::Edge> && edgesU)
{
    edges = edgesU;

    auto const m
    {
        [& edges]
        {
            std::unordered_map<IdxType, IdxType> m;
            m.reserve(edges.size());
            for (IdxType i {}; i < edges.size(); i++)
                m.insert({ edges[i].idx, i });

            return m;
        }()
    };
    
    /* rename idices */
    for (IdxType i {}; i < edges.size(); i++)
    {
        auto const & edgeU = edgesU[i];
        auto       & edge  = edges [i];

        /* update nbrs' record */
        for (char j {}; j < 4; j++)             // count on compiler's loop unrolling
            if (auto const & oldIdx { edgeU.nbrs[j] })
            {
                // the search /has to/ be coordinate-based since edge.idx is changing
                // auto const newIdx { m.at(oldIdx) };
                auto const newIdx { m.at(oldIdx) };
                    
                if (newIdx > i)
                {
                    auto & nbrs { edges[newIdx].nbrs };

                    // replace /at most only one/ match
                    auto itr { std::find(nbrs.begin(), nbrs.end(), edge.idx) };
                    if (itr != nbrs.end())
                        * itr = 1 + i;
                        
                    edge.nbrs[j] = 1 + newIdx;    // idx == 0 is reserved for /null/
                }
            }
        
        edge.idx = 1 + i;                         // idx == 0 is reserved for /null/
    }

    edges.shrink_to_fit();
}


