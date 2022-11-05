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

#include <filesystem>
#include <ios>
#include <fstream>

#include "actuator.hpp"


class Actuator;

// TODO(): eliminate the remaining magic numbers
class Plotter
{
public:

             Plotter() = delete;
    virtual ~Plotter() = default;

    Plotter(Geometry                               const & geometry       ,
            std::vector<std::unique_ptr<Actuator>> const & actrs          ,
            std::filesystem::path                        & svgPath        ,
            CrdType                                        dMAX     = DMAX) noexcept;

    void plot();

    void setBckgClr(std::string clr) { bckgClr = clr; };
    void setInfcClr(std::string clr) { infcClr = clr; };
    void setExitClr(std::string clr) { exitClr = clr; };
    void setSoldClr(std::string clr) { soldClr = clr; };
    void setMetaClr(std::string clr) { metaClr = clr; };
    void setInlpClr(std::string clr) { inlpClr = clr; };
    void setFnlpClr(std::string clr) { fnlpClr = clr; };
    
    void setpathOpc(CrdType opc) { pathOpc = opc; };

protected:

    void initialize  ()         ;
    void setExtm     () noexcept;
    void plotGeometry()         ;
    void plotActrs   ()         ;
    void finalize    ()         ;

    void
    compare(smr::Line const & line) noexcept;

    CrdType
    prpX(CrdType x) const noexcept { return (x - xMin + ofSt ) * scl; }

    CrdType
    prpY(CrdType y) const noexcept { return (yMax - y + ofSt ) * scl; }
    
    Geometry                               const & geometry;
    std::vector<std::unique_ptr<Actuator>> const & actrs   ;
    std::filesystem::path                  const & svgPath ;
    std::ofstream                                  svgFile ;

    CrdType xMin { + std::numeric_limits<CrdType>::infinity() };
    CrdType xMax { - std::numeric_limits<CrdType>::infinity() };
    CrdType yMin { + std::numeric_limits<CrdType>::infinity() };
    CrdType yMax { - std::numeric_limits<CrdType>::infinity() };

    CrdType const dMAX;
    
    CrdType dMax {    };
    CrdType ofSt { .5 };
    CrdType scl  { 40 };

    /* colors */
    std::string
        bckgClr { "#131415" },
        infcClr { "#596b2e" },
        exitClr { "#596b2e" },
        soldClr { "#1d6d7f" },
        metaClr { "#363636" },
        inlpClr { "#6b2e3b" },
        fnlpClr { "#52678f" };

    /* opacities */
    CrdType
        pathOpc { .75 };

public:

    static CrdType constexpr DMAX { 2000 };
};



