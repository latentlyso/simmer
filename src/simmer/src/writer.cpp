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

#include "writer.hpp"
#include "actuator.hpp"


Writer::Writer(Geometry                               const & geometry,
               std::vector<std::unique_ptr<Actuator>> const & actrs   ,
               std::filesystem::path                  const & otptPath)

    : geometry { geometry },
      actrs    { actrs    },
      otptPath { otptPath }
{

    otptFile = { otptPath, std::ios_base::trunc };

    /* initialize */
    otptFile << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>\n";
    otptFile << fmt::format("<agents xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\">\n");

    /* specify depths */
    const std::string dpt1 ( 1 * indent, ' ' );
    const std::string dpt2 ( 2 * indent, ' ' );
    const std::string dpt3 ( 3 * indent, ' ' );

    std::string const agntStr { "<agent idx=\"{" + idxFmt + "}\">\n"                       };
    std::string const cellStr { "<cell idx=\"{" + idxFmt + "}\">\n"                        };
    std::string const pontStr { "<point x=\"{" + crdFmt + "}\" y=\"{" + crdFmt + "}\"/>\n" };
    
    auto const & cMapR { geometry.getCMapR() };
        
    /* loop over agents */
    for (auto const & actor : actrs)
    {
        otptFile << dpt1 << dyna_print(agntStr, actor->getNIdx());
        
        auto const & aPath { actor->getPath() };

        for (auto const & pathPair : aPath)
        {
            auto const cIdx { cMapR.at(pathPair.first) };
            auto const path { pathPair.second          };

            otptFile <<  dpt2 << dyna_print(cellStr, cIdx);

            for (auto const & p : path)
                otptFile << dpt3 << dyna_print(pontStr, p.x, p.y);

            otptFile << dpt2 << "</cell>\n";
        }
            
        otptFile << dpt1 << "</agent>\n";
    }
    
    /* finalize */
    otptFile << "</agents>\n";
}

