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

#include <set>

#include "plotter.hpp"


Plotter::Plotter(Geometry                               const & geometry,
                 std::vector<std::unique_ptr<Actuator>> const & actrs   ,
                 std::filesystem::path                        & svgPath ,
                 CrdType                                        dMAX    ) noexcept

    : geometry { geometry },
      actrs    { actrs    },
      svgPath  { svgPath  },
      dMAX     { dMAX     }
{
    svgFile = { svgPath, std::ios_base::trunc };
}


void
Plotter::plot()
{
    initialize  ();
    plotGeometry();
    plotActrs   ();
    finalize    ();
}


inline void
Plotter::initialize()
{
    setExtm();

    CrdType xSize { ((xMax - xMin) + 2 * ofSt) * scl };
    CrdType ySize { ((yMax - yMin) + 2 * ofSt) * scl };

    svgFile << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>\n<svg xmlns=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http://www.w3.org/1999/xlink\" ";

    svgFile << fmt::format("width=\"{0:.2f}pt\" height=\"{1:.2f}pt\" "
                           "viewBox=\"0 0 {0:.2f} {1:.2f}\"",
                           xSize, ySize)
            << " version=\"1.1\">\n";

    std::string const bckgStr
    {
        "  <rect x=\"0\" y=\"0\" width=\"{:.2f}\" height=\"{:.2f}\" style=\"fill: " +
        bckgClr +
        ";\"/>\n"
    };
    svgFile << dyna_print(bckgStr, xSize, ySize);
}


inline void
Plotter::setExtm() noexcept
{
    for (auto const & walls : geometry.getWallz())
        for (auto const & line : walls)
            compare(line);

    auto const & susoExtz { geometry.getSusoExtz() };
    auto const & nosoz    { geometry.getNosoz   () };

    for (IdxType i {}; i < susoExtz.size(); i++)
        for (auto const & trio : susoExtz[i])
            compare(nosoz[i][trio.sIdx]);
    
    dMax = std::max(xMax - xMin, yMax - yMin);

    ofSt = dMax * .01;

    scl  = dMAX / dMax ;
}


inline void
Plotter::plotGeometry()
{
    auto const & nosoz { geometry.getNosoz() };
    
    std::vector<smr::Line> infcLine;
    std::vector<smr::Line> exitLine;

    std::string const metaStr
    {
        "  <g style=\"stroke: " +
        metaClr +
        "; fill: none; stroke-width: {:.2f};\">\n    <path d=\""
    };
    svgFile << dyna_print(metaStr, .75);
    for (IdxType cIdx {}; cIdx < nosoz.size(); cIdx++)
    {
        if (geometry.isDummy(cIdx))
            continue;
        
        auto const & nosos { nosoz[cIdx] };

        for (IdxType sIdx {}; sIdx < nosos.size(); sIdx++)
        {
            auto const & line { nosos[sIdx] };

            if (geometry.isSubsolid(cIdx, sIdx))
            {
                if (geometry.andIsExit(cIdx, sIdx))
                    exitLine.push_back(line);
                else
                    infcLine.push_back(line);
            }
            else
                svgFile << fmt::format(" M {:.2f},{:.2f} {:.2f},{:.2f}", prpX(line.u.x), prpY(line.u.y), prpX(line.v.x), prpY(line.v.y));
        }
    }
    svgFile << "\"/>\n  </g>\n";

    std::string const infcStr
    {
        "  <g style=\"stroke: " +
        infcClr +
        "; fill: none; stroke-width: {:.2f}; stroke-linecap: round; stroke-linejoin: round; stroke-dasharray: 4, 4;\">\n    <path d=\""
    };
    svgFile << dyna_print(infcStr, 2.);
    for (auto const & line : infcLine)
        svgFile << fmt::format(" M {:.2f},{:.2f} {:.2f},{:.2f}", prpX(line.u.x), prpY(line.u.y), prpX(line.v.x), prpY(line.v.y));
    svgFile << "\"/>\n  </g>\n";

    std::string const exitStr
    {
        "  <g style=\"stroke: " +
        exitClr +
        "; fill: none; stroke-width: {:.2f}; stroke-linecap: round; stroke-linejoin: round; stroke-dasharray: 4, 4;\">\n    <path d=\""
    };
    svgFile << dyna_print(exitStr, 2.);
    for (auto const & line : exitLine)
        svgFile << fmt::format(" M {:.2f},{:.2f} {:.2f},{:.2f}", prpX(line.u.x), prpY(line.u.y), prpX(line.v.x), prpY(line.v.y));
    svgFile << "\"/>\n  </g>\n";

    std::string const soldStr
    {
        "  <g style=\"stroke: " +
        soldClr +
        "; fill: none; stroke-width: {:.2f}; stroke-linecap: round; stroke-linejoin: round;\">\n    <path d=\""
    };
    svgFile << dyna_print(soldStr, 10.0);
    auto const & wallz { geometry.getWallz() };
    for (IdxType cIdx {}; cIdx < wallz.size(); cIdx++)
    {
        if (geometry.isDummy(cIdx))
            continue;

        for (auto const & line : wallz[cIdx])
            svgFile << fmt::format(" M {:.2f},{:.2f} {:.2f},{:.2f}", prpX(line.u.x), prpY(line.u.y), prpX(line.v.x), prpY(line.v.y));
    }
    svgFile << "\"/>\n  </g>\n";
}


inline void
Plotter::plotActrs()
{
    /* extremes containers */
    std::set<smr::Point> inlPt;
    std::set<smr::Point> fnlPt;
    
    /* plot paths */
    std::string const pathStr
    {
        "  <g style=\"stroke: " +
        inlpClr +
        "; fill: none; stroke-width: {:.2f}; stroke-linejoin: round;\" opacity=\"{:.2f}\">\n"
    };
    svgFile << dyna_print(pathStr, 1.75, pathOpc);

    /* plot paths: intermediates */
    std::string const intmStr
    {
        "    <circle cx=\"{:.2f}\" cy=\"{:.2f}\" r=\"{:.2f}\" style=\"fill: " +
        bckgClr +
        ";\"/>\n"
    };
    
    for (auto const & actor : actrs)
    {
        svgFile << "    <path d=\"M";
        
        auto const & aPath { actor->getPath() };

        inlPt.insert(aPath.front().second.front());
        fnlPt.insert(aPath.back().second.back());

        for (auto const & pathPair : aPath)
        {
            auto path { pathPair.second };

            for (auto const & p : path)
                svgFile << fmt::format(" {:.2f},{:.2f}", prpX(p.x), prpY(p.y));
        }
            
        svgFile << "\"/>\n";

        for (auto const & pathPair : aPath)
        {
            auto path { pathPair.second };

            for (auto const & p : path)
                svgFile << dyna_print(intmStr, prpX(p.x), prpY(p.y), 1.5);
        }
    }
    
    /* plot paths: extremes */
    std::string const inlpStr
    {
        "    <circle cx=\"{:.2f}\" cy=\"{:.2f}\" r=\"{:.2f}\" style=\"fill: " +
        inlpClr +
        "; stroke: none\"/>\n"
    };
    for (auto const & p : inlPt)
        svgFile << dyna_print(inlpStr, prpX(p.x), prpY(p.y), 3.25);

    std::string const fnlpStr
    {
        "    <circle cx=\"{:.2f}\" cy=\"{:.2f}\" r=\"{:.2f}\" style=\"fill: " +
        fnlpClr +
        "; stroke: none\"/>\n"
    };
    for (auto const & p : fnlPt)
        svgFile << dyna_print(fnlpStr, prpX(p.x), prpY(p.y), 3.25);

    svgFile << "  </g>\n";    
}


inline void
Plotter::finalize()
{
    svgFile << "</svg>\n";
}


inline void
Plotter::compare(smr::Line const & line) noexcept
{
    auto const & ux { line.u.x };
    auto const & uy { line.u.y };

    xMin = std::min({ xMin, ux });
    xMax = std::max({ xMax, ux });
    yMin = std::min({ yMin, uy });
    yMax = std::max({ yMax, uy });

    auto const & vx { line.v.x };
    auto const & vy { line.v.y };

    xMin = std::min({ xMin, vx });
    xMax = std::max({ xMax, vx });
    yMin = std::min({ yMin, vy });
    yMax = std::max({ yMax, vy });
}

