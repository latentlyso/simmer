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

#include "geometry.hpp"
#include "partition.hpp"
#include "parser.hpp"


Parser::Parser(std::filesystem::path const & geomPath ,
               Geometry                    & geometry ,
               Partition             const & partition) noexcept

    : geomPath  { geomPath  },
      geometry  { geometry  },
      partition { partition }
{}


std::optional<std::string>
Parser::parse()
{
    pugi::xml_document doc;
    auto const result { doc.load_file(geomPath.string().c_str()) };
    if (not result)
        return fmt::format("Could not load the input geometry file\n {}", result.description());

    auto const xPartition { doc.child("geometry").child("partition") };
    if (not xPartition)
        return "Expected a 'partition' node.";

    if (auto const err { parsePartition(xPartition) })
        return err.value();
    
    return {};
}


std::optional<std::string>
Parser::parsePartition(pugi::xml_node const & xPartition) const
{
    /* types employed from the 'Partition' class */
    auto const cellS    { partition.cell()    };
    auto const idxS     { partition.idx()     };
    auto const dummyS   { partition.dummy()   };
    auto const polygonS { partition.polygon() };
    auto const pointS   { partition.point()   };
    auto const xCordS   { partition.xCord()   };
    auto const yCordS   { partition.yCord()   };
    auto const lineCTS  { partition.lineCT()  };
    auto const sIdxS    { partition.sIdx()    };
    auto const cIdxS    { partition.cIdx()    };
    auto const oIdxS    { partition.oIdx()    };
    auto const parityS  { partition.parity()  };
    
    auto const infcColor { static_cast<LCType>(LineColor::INFC) };
    auto const exitColor { static_cast<LCType>(LineColor::EXIT) };
    auto const soldColor { static_cast<LCType>(LineColor::SOLD) };
    auto const invdColor { static_cast<LCType>(LineColor::INVD) };

    std::unordered_set<IdxType> cellIds;

    /* initialize 'ftr' */
    auto ftr { std::async([] ([[ maybe_unused ]] Cell && cell) -> std::optional<std::string> { return {}; }, Cell { {}, {} }) };
    
    for (auto xCell { xPartition.child(cellS) };
         xCell;
         xCell = xCell.next_sibling(cellS)     )
    {
        auto const cellIdx { xAsIdxType(xCell.attribute(idxS).value()) };
        if (not cellIdx)
            return fmt::format("invalid cell index ({}) encountered; expected positive", cellIdx);

        if (cellIds.contains(cellIdx))
            return fmt::format("duplicate cell index ({}) encountered; expected unique", cellIdx);
        else
            cellIds.insert(cellIdx);
        
        bool const dummy {xCell.attribute(dummyS).as_bool(false)};  // dummy cell flag
        
        Cell cell { cellIdx, dummy };

        // - the first poly should be the one enclosing/defining the cell;
        //   all subsequent polys should be obstacles within it.
        //   assuming CCW orientation, DT should be done here so that meta-lines
        //   are added to (the cell and) the router in their
        //   proper (memory) locality.
        // - inner polys (obstacles) shall not intersect with the outer (first)
        //   poly, and with each other; otherwise triangulators may fail (CGAL, eg.).
        //   this is not a limitation, only a restriction.
        // - inner polys shall not contain (completing) metalines.
        for (auto xPolygons { xCell.child(polygonS) };
             xPolygons;
             xPolygons = xPolygons.next_sibling(polygonS))
        {
            std::vector<smr::Point> poly;
            std::vector<smr::Line>  walls;
            std::vector<smr::Line>  susos;
            std::vector<TriType>    susoExts;
            std::vector<BlobType>   blobs;

            bool inBlob { false };
            
            auto const xPoint0 { xPolygons.child(pointS) };
            
            for (auto xPoint { xPolygons.child(pointS) };
                 xPoint;
                 xPoint = xPoint.next_sibling(pointS)   )
            {
                smr::Point u { xAsDouble (xPoint.attribute(xCordS).value()), 
                               xAsDouble (xPoint.attribute(yCordS).value()) };
                
                /* for subsolid lines, 'cIdx == cellIdx' signals an EXIT line */
                LCType  const cTyp { xAsLCType (xPoint.attribute(lineCTS).value()) };  // color   type
                IdxType const sIdx { xAsIdxType(xPoint.attribute(sIdxS  ).value()) };  // (s)elf  idx
                IdxType       cIdx { xAsIdxType(xPoint.attribute(cIdxS  ).value()) };  // o.cell  idx
                IdxType const oIdx { xAsIdxType(xPoint.attribute(oIdxS  ).value()) };  // (o)ther idx
                bool    const pFlg { xPoint.attribute(parityS).as_bool(false)      };  // parity  flag

                if (cTyp == exitColor)
                    cIdx = cellIdx;
                
                if ( cTyp >= invdColor)
                    return fmt::format("invalid line color ({}) encountered", cTyp);

                if ((cTyp == infcColor) and not (sIdx and cIdx and oIdx))
                    return fmt::format("invalid interface index combination ({}, {}, {})"
                                       " encountered in cell {}; expected only positive values",
                                       sIdx, cIdx, oIdx, cellIdx);
                
                auto xNextPoint { xPoint.next_sibling(pointS) };
                if (not xNextPoint)
                    xNextPoint = xPoint0;

                if (pFlg)
                    geometry.addParityFlag(cellIdx, cIdx);
                
                smr::Point v { xAsDouble(xNextPoint.attribute(xCordS).value()),
                               xAsDouble(xNextPoint.attribute(yCordS).value()) };

                poly.push_back(u);
                
                smr::Line line { u, v };
                orderPoints(line);

                switch (cTyp)
                {
                case soldColor:
                    
                    walls.push_back(std::move(line));

                    inBlob = false;

                    break;

                case infcColor:

                    if (inBlob)
                        blobs.back().push_back(sIdx);
                    else
                    {
                        blobs.emplace_back(BlobType { sIdx });
                        inBlob = true;
                    }
                    [[ fallthrough ]];
                case exitColor:

                    susos   .push_back(std::move(line));
					susoExts.push_back(TriType { sIdx, cIdx, oIdx });

                    break;
                }
            }

            cell.addPoly(std::move(poly),
                         std::move(walls),
                         std::move(susos),
                         std::move(susoExts),
                         std::move(blobs)   );
        }
        
        if (auto const err { parseCellExt(xCell, partition) })
            return err.value();

        if (auto const err { ftr.get() })
            return err.value();
        
        ftr = std::async(
            [this] (Cell && cell)
            {
                return geometry.processCell(std::move(cell));
            },
            std::move(cell));
    }
    if (auto const err { ftr.get() })
        return err.value();

    return {};
}


