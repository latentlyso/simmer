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

#include "finder.hpp"
#include "geometry.hpp"


Finder::Finder(Geometry      const & geometry,
               ThreadCntType         ntdi,
               ThreadCntType         ntdo    )

    : geometry { geometry },
      ntdi     { ntdi     },
      ntdo     { ntdo     }
{}


void
Finder::findLocal(std::vector<std::vector<IdxType>> & pathMCSs,
                  std::vector<std::vector<CrdType>> & distMCSs) const
{
    auto const & nosoz { geometry.getNosoz() };

    /* process cells in the order of their size */
    auto const nososSize
    {
        [& nosoz]
        {
            std::vector<std::pair<IdxType, IdxType>> nososSize;
            nososSize.reserve(nosoz.size());
            for (IdxType i {}; i < nosoz.size(); i++)
                nososSize.push_back({ i, nosoz[i].size() });

            std::sort(nososSize.begin(), nososSize.end(),
                      [] <typename T> (T const & a, T const & b)
                      {
                          return a.second < b.second;
                      }
                );

            return nososSize;
        }()
    };

    std::queue<IdxType> que;
    for (IdxType i {}; i < nososSize.size(); i++)
        que.push(nososSize[i].first);

    auto const lambda
    {
        [this, & pathMCSs, & distMCSs] (auto const idx)
        {
            formMCS(pathMCSs, distMCSs, idx);
        }
    };
    
    Spawner spawner { ntdo };
    spawner.spawn<CallPattern::FNIDX>(que, lambda);
}


void
Finder::pathFinderGlobal(std::vector<IdxType>       & pathM,
                         std::vector<CrdType>       & distM,
                         GraphType            const & g    ) const
{
    std::queue<IdxType> que;
    for (IdxType i {}; i < g.size(); i++)
        que.push(i);
    
    std::mutex mu;

    auto lambda
    {
        [& pathM, & distM, & g, & que, & mu]
        {
        
            std::unique_lock lock { mu };

            while (!que.empty())
            {
                auto i { que.front() };
                que.pop();

                lock.unlock();

                // function body +
                dijkstraPQ(g, pathM, distM, i);
                // function body -

                lock.lock();
            }
        }
    };

    std::vector<std::thread> tds;

    for (IdxType i {}; i < ntdi; i++)
        tds.emplace_back(std::thread { lambda });

    for (auto & td : tds)
        td.join();
}


void
Finder::pathFinderLocal(std::vector<IdxType>       & pathMCS,
                        std::vector<CrdType>       & distMCS,
                        GraphType            const & g      ,
                        PairedIdxQueType           & que    ) const
{
    std::mutex mu;

    auto lambda
    {
        [& pathMCS, & distMCS, & g, & que, & mu]
        {
        
            std::unique_lock lock { mu };

            while (!que.empty())
            {
                auto const st { que.front() };
                que.pop();

                lock.unlock();

                // function body +
                dijkstraPQCS(pathMCS, distMCS, g, st);
                // function body -

                lock.lock();
            }
        }
    };

    std::vector<std::thread> tds;

    for (IdxType i {}; i < ntdi; i++)
        tds.emplace_back(std::thread { lambda });

    for (auto & td : tds)
        td.join();
}


void
Finder::formMCS(std::vector<std::vector<IdxType>> & pathMCSs,
                std::vector<std::vector<CrdType>> & distMCSs,
                IdxType                             idx     ) const
{
    auto const & nosos    { geometry.getNosoz   ()[idx] };
    auto const & susoExts { geometry.getSusoExtz()[idx] };
    
    IdxType const xSize { static_cast<IdxType>(nosos   .size()) };
    IdxType const ySize { static_cast<IdxType>(susoExts.size()) };

    auto & pathMCS { pathMCSs[idx] };
    auto & distMCS { distMCSs[idx] };
    
    pathMCS.clear();
    distMCS.clear();
    
    pathMCS.resize(xSize * ySize);
    distMCS.resize(xSize * ySize);

    auto const & nbrs { geometry.getNbrz()[idx] };

    auto const g
    {
        [& nosos, & nbrs, xSize]
        {
            GraphType g;
            g.resize(xSize);

            for (IdxType i {}; i < xSize; i++)
            {
                auto const & line  { nosos[i] };

                for (IdxType j {}; j < 4; j++)
                    if (auto const & J { nbrs[i * 4 + j] })
                    {
                        auto const Jm { J - 1 };

                        g[i].insert({ Jm, euclideanLLDistance(line, nosos[Jm]) });
                    }
            }

            return g;
        }()
    };
    
    PairedIdxQueType que;
    for (IdxType i {}; i < ySize; i++)
        que.push({ susoExts[i].sIdx, i });
    
    pathFinderLocal(pathMCS, distMCS, g, que);
}


/**
 * lazy implementation of Dijkstra SSSP
 * adpated from http://nmamano.com/blog/dijkstra/dijkstra.html
 */
void
dijkstraPQCS(std::vector<IdxType>              & pathMCS,
             std::vector<CrdType>              & distMCS,
             GraphType                   const & g      ,
             std::pair<IdxType, IdxType> const   st     )
{
    auto const xSize { g.size()               };
    auto const ySize { pathMCS.size() / xSize };

    auto const [s, t] { st };

    // distance
    std::vector<CrdType> d;
    d.resize(xSize, std::numeric_limits<CrdType>::infinity());
    d[s] = 0.;

    // parent
    std::vector<IdxType> p;
    p.resize(xSize, IdxTypeMax);
    p[s] = s;

    // mask
    std::vector<bool> m;
    m.resize(xSize);
    
    std::priority_queue<NbrType, std::vector<NbrType>, std::greater<NbrType>> q;
    q.push({ 0., s });

    while (not q.empty())
    {
        auto const u { q.top().second };
        q.pop();
        
        if (m[u])
            continue;
        m[u] = true;
        
        
        for (auto const & [v, wgt] : g[u])
        {
            auto const dv { d[u] + wgt };
            
            // if (fLess(dv, d[v]))
            if (dv < d[v])
            {
                d[v] = dv;
                p[v] = u;
                
                q.push({ d[v], v });
            }
        }
    }

    for (IdxType i {}; i < xSize; i++)
    {
        pathMCS[i * ySize + t] = p[i];
        distMCS[i * ySize + t] = d[i];
    }
}


/**
 * lazy implementation of Dijkstra SSSP
 * adpated from http://nmamano.com/blog/dijkstra/dijkstra.html
 */
void
dijkstraPQ(GraphType            const & graph,
           std::vector<IdxType>       & pathM,
           std::vector<CrdType>       & distM,
           IdxType              const   s    )
{
    auto const xSize { graph.size() };

    // distance
    std::vector<CrdType> d;
    d.resize(xSize, std::numeric_limits<CrdType>::infinity());
    d[s] = 0.;

    // parent
    std::vector<IdxType> p;
    p.resize(xSize, IdxTypeMax);
    p[s] = s;

    // mask
    std::vector<bool> m;
    m.resize(xSize);
    
    std::priority_queue<NbrType, std::vector<NbrType>, std::greater<NbrType>> q;
    q.push({ 0., s });

    while (not q.empty())
    {
        auto const u { q.top().second };
        q.pop();
        
        if (m[u])
            continue;
        m[u] = true;
        
        for (auto const & [v, wgt] : graph[u])
        {
            auto const dv { d[u] + wgt };
            
            // if (fLess(dv, d[v]))
            if (dv < d[v])
            {
                d[v] = dv;
                p[v] = u;
                
                q.push({ d[v], v });
            }
        }
    }

    for (IdxType i {}; i < xSize; i++)
    {
        pathM[i * xSize + s] = p[i];
        distM[i * xSize + s] = d[i];
    }
}

