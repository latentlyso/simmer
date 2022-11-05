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

#include "spawner.hpp"


using NbrType   = std::pair<CrdType, IdxType>;
using GraphType = std::vector<std::unordered_map<IdxType, CrdType>>;

class Geometry;

class Finder
{
public:

             Finder() = delete;
    virtual ~Finder() = default;
    
    Finder(Geometry      const & geometry       ,
           ThreadCntType         ntdi     = NTDI,
           ThreadCntType         ntdo     = NTDO);

    void
    findLocal(std::vector<std::vector<IdxType>> & pathMCSs,
              std::vector<std::vector<CrdType>> & distMCSs) const;
    
    virtual void
    pathFinderGlobal(std::vector<IdxType>       & pathM,
                     std::vector<CrdType>       & distM,
                     GraphType            const & g    ) const;
    
protected:

    Geometry const & geometry;

    ThreadCntType const ntdi;
    ThreadCntType const ntdo;

    using PairedIdxQueType = std::queue<std::pair<IdxType, IdxType>>;
    virtual void
    pathFinderLocal(std::vector<IdxType>       & pathMCS,
                    std::vector<CrdType>       & distMCS,
                    GraphType            const & g      ,
                    PairedIdxQueType           & que    ) const;
    
    void
    formMCS(std::vector<std::vector<IdxType>> & pathMCSs,
            std::vector<std::vector<CrdType>> & distMCSs,
            IdxType                             idx     ) const;
    
public:

    static ThreadCntType constexpr NTDI { 8 };
    static ThreadCntType constexpr NTDO { 1 };
};


void
dijkstraPQCS(std::vector<IdxType>              & pathMCS,
             std::vector<CrdType>              & distMCS,
             GraphType                   const & g      ,
             std::pair<IdxType, IdxType> const   st     );

void
dijkstraPQ(GraphType            const & graph,
           std::vector<IdxType>       & pathM,
           std::vector<CrdType>       & distM,
           IdxType              const   s    );


