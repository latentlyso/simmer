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

#include "simmer.hpp"


Simmer::Simmer(Geometry                               & geometry,
               Router                                 & router  ,
               std::vector<std::unique_ptr<Actuator>> & actrs   ,
               ThreadCntType                            ntd     )

    : geometry { geometry },
      router   { router   },
      actrs    { actrs    },
      ntd      { ntd      }
{
    for (IdxType i {}; i < actrs.size(); i++)
        iQue.push(i);
    
    iVue.reserve(actrs.size());
    for (auto const & actor : actrs)
    {
        auto where { actor->getWhere() };

        iVue[where.first].emplace_back(std::move(where.second));
    }

    oVue.reserve(actrs.size());
    
    intervene();

    // /* single-threaded */
    // for (auto & actor : actrs)
    //     (*actor)(oQue, queM, iVue, oVue, vueM);
    // do {

    //     iQue = {};
    //     std::swap(iQue, oQue);

    //     std::swap(iVue, oVue);
    //     oVue.clear();

    //     for (auto & actor : actrs)
    //         (*actor)(oQue, queM, iVue, oVue, vueM);

    // } while (not oQue.empty());

    /* multi-threaded */
    std::barrier barry { ntd + 1 };
    // 'this_thread'           ^

    Pooler pooler { ntd, barry };

    pooler.pool<CallPattern::FNOBP>(iQue, actrs, oQue, queM, iVue, oVue, vueM);
    barry.arrive_and_wait();                                    // parity shift
    do
    {
        std::swap(iQue, oQue);
        std::swap(iVue, oVue);
        oVue.clear();

        barry.arrive_and_wait();
        intervene();
        barry.arrive_and_wait();                                // parity shift
        
    } while (!oQue.empty());
    pooler.shutdown();
}

