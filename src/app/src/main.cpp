
#include "cxxopts/cxxopts.hpp"

#include "partition.hpp"
#include "parser.hpp"
#include "mesher.hpp"
#include "finder.hpp"
#include "simmer.hpp"
#include "writer.hpp"
#include "plotter.hpp"


std::vector<std::filesystem::path>
argParser(int argc, char ** argv);


/** measures durations in seconds of type double */
class Timer
{
    using ClockType = std::chrono::steady_clock;

public:
    
    double duration() const
    {
        return (std::chrono::duration<double> (ClockType::now() - t)).count();
    }

    void now() { t = ClockType::now(); }

private:

    std::chrono::time_point<ClockType> t { ClockType::now() };
};


inline int
randr(int max) noexcept
{
    return std::rand() / (RAND_MAX / (max + 0u));
}


class ActuatorD : public Actuator
{

public:

    ActuatorD(IdxType            nIdx,
              IdxType            cIdx,
              smr::Point         pos,
              Geometry   const & geometry,
              Router     const & router  )
        
        : Actuator { nIdx, cIdx, pos, geometry, router }
        {}
};


int main(int argc, char ** argv)
{
    Timer timer;
    
    std::cout << std::endl;  /* good measure */

    auto argVec { argParser(argc, argv) };

    auto geomPath { argVec[0] };
    auto otptPath { argVec[1] };

    /* plot the plot */
    bool ptp { true };
    std::filesystem::path plotPath {};
    if (argVec.size() == 3)
        plotPath = argVec[2];
    else
        ptp = false;

    Mesher   mesher   { 4      };
    Geometry geometry { mesher };
    
    timer.now();
    Partition partition {                               };
    Parser    parser    { geomPath, geometry, partition };

    auto const parserError { parser.parse() };
    if (parserError)
        throw std::logic_error(parserError.value());
    
    std::cout << fmt::format("Parser: {:7.3f} secs", timer.duration()) << std::endl;

    auto const geometryError { geometry.finalize() };
    if (geometryError)
        throw std::logic_error(geometryError.value());

    /*
     * outer thread count: 2
     * inner thread count: 4
     */
    timer.now();
    Finder finder { geometry, 2, 4   };
    Router router { geometry, finder };

    std::cout << fmt::format("Router: {:7.3f} secs", timer.duration()) << std::endl;

    /* randomly distribute the agents on midpoints of nonsolid lines */
    auto const seed { std::time(nullptr) % (1 << 24) };
    std::srand(seed);
    std::cout << fmt::format("Seed #:   {}", seed) << std::endl;

    IdxType const agnts { 400 };

    std::vector<std::unique_ptr<Actuator>> actr;
    actr.reserve(agnts);

    /* do not place agents too close to the walls */
    /* initial minimum distance from walls        */
    CrdType const imdw { .3 };
    
    auto cnt { geometry.linesPerCell() };

    for (IdxType i {}; i < agnts; i++)
    {
        IdxType const cIdx { static_cast<IdxType>(randr(cnt.size())) };
        IdxType const sIdx { static_cast<IdxType>(randr(cnt[cIdx] )) };

        /* test for well-paddedness and dummy cells */
        auto const pos { linePoint(geometry.getNosoz()[cIdx][sIdx]) };
        if ((not geometry.isInsideCellX(pos, cIdx, imdw)) or (geometry.isDummy(cIdx)))
        {
            i--;
            continue;
        }

        actr.push_back(std::make_unique<ActuatorD>( i, cIdx, pos, geometry, router ));
    }

    /* commence the simulation */
    timer.now();
    Simmer simmer { geometry, router, actr, 7 };
    std::cout << fmt::format("Simmer: {:7.3f} secs", timer.duration()) << std::endl;

    /*
     * primary output,
     * an XML file:
     * main node: 'agents'
     * child nodes: nominal agent ids
     * (child)^2 nodes : cells
     * (child)^3 nodes : position in cell
     */
    timer.now();
    Writer writer { geometry, actr, otptPath };
    std::cout << fmt::format("Writer: {:7.3f} secs", timer.duration()) << std::endl;

    /* secondary output */
    if (ptp)
    {
        /* assuming a unified 2D coordinate system */
        timer.now();
        
        Plotter plotter { geometry, actr, plotPath };
        
        plotter.setBckgClr("#FFFFFF");
        plotter.setMetaClr("#9F9F9F");
        plotter.setSoldClr("#707070");
        plotter.setInfcClr("#808080");

        std::string const actClr { "#912d40" };
        
        plotter.setFnlpClr(actClr);
        plotter.setInlpClr(actClr);
        
        plotter.plot();
        
        std::cout << fmt::format("Plottr: {:7.3f} secs", timer.duration()) << std::endl;
    }
    
    return EXIT_SUCCESS;
}


std::vector<std::filesystem::path>
argParser(int argc, char ** argv)
{
    cxxopts::Options options { "simmerApp", "Console access to the Simmer library" };

    options.add_options()
        ("g,geometry", "Geometry specification file", cxxopts::value<std::string>())
        ("o,output"  , "Output trajectory file"     , cxxopts::value<std::string>())
        ("p,plot"    , "Plot file"                  , cxxopts::value<std::string>())
        ;
    
    auto result { options.parse(argc, argv) };

    if (result.count("help"))
    {
      std::cout << options.help({ "" }) << std::endl;
      exit(0);
    }

    std::vector<std::filesystem::path> argVec;
        
    if (result.count("g"))
    {
        std::filesystem::path            geomPath   { result["g"].as<std::string>() };
        std::filesystem::directory_entry geomPathDE { geomPath                      };

        if ((not geomPathDE.exists()) or (not geomPathDE.is_regular_file()))
        {
            std::cout << "Invalid geometry file" << std::endl;
            exit(1);
        }
        
        argVec.push_back(geomPath);
    }
    else
    {
        std::cout << "Expected a geometry file" << std::endl;
        exit(1);
    }

    if (result.count("o"))
    {
        std::filesystem::path otptPath { result["o"].as<std::string>() };

        if (not static_cast<std::filesystem::directory_entry>(otptPath.parent_path()).exists())
        {
            std::cout << "Output file directory does not exist" << std::endl;
            exit(1);
        }
        
        argVec.push_back(otptPath);
    }
    else
    {
        std::cout << "Expected an output file" << std::endl;
        exit(1);
    }
    
    if (result.count("p"))
    {
        std::filesystem::path plotPath { result["p"].as<std::string>() };

        if (not static_cast<std::filesystem::directory_entry>(plotPath.parent_path()).exists())
        {
            std::cout << "Plot file directory does not exist" << std::endl;
            exit(1);
        }
        
        argVec.push_back(plotPath);
    }
    
    return argVec;
}


