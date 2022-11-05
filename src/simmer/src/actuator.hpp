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

#include <list>

#include "router.hpp"


using ViewType     = std::unordered_map<IdxType, std::vector<smr::Line>>;
using CellPathType = std::pair<IdxType, std::list<smr::Point>>;

class Actuator
{    

public:

             Actuator() = delete;
    virtual ~Actuator() = default;
    
    Actuator(IdxType            nIdx    ,
             IdxType            cIdx    ,
             smr::Point         pos     ,
             Geometry   const & geometry,
             Router     const & router  );

                Actuator(Actuator const & src) = delete;
    Actuator & operator=(Actuator const & rhs) = delete;
    
    Actuator(Actuator && src) noexcept = default;

    auto         getNIdx() const { return nIdx; }
    auto const & getPath() const { return path; }

    std::pair<IdxType, smr::Line>
    getWhere() const noexcept { return { cIdx, { pos, vel } }; }
    
    virtual void
    operator()(std::queue<IdxType>       & que ,
               std::mutex                & queL,
               ViewType            const   iVue,
               ViewType                    oVue,
               std::mutex                & vueM);

protected:

    /* agent's assigned index */
    IdxType const idx;

    /*
     * agent's nominal index
     * it is client code's responsibility to ensure
     * that all agent indices are mutually distinct
    */
    IdxType const nIdx;

    IdxType cIdx;
    
    smr::Point pos;
    smr::Point vel;

    Geometry  const & geometry;
    Router    const & router  ;
    
    CrdType dpt;

    /* maximum distance traveled per time step (velocity) */
    CrdType const dptM { DPTM };

    /* history is made here */
    std::list<CellPathType> path;

    inline static IdxType gIdx {};

public:

    static IdxType constexpr HOP  { 10 };
    static CrdType constexpr DPTM { .9 };
};


