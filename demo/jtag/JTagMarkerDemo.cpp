#include <rwlibs/algorithms/jtag/JTagMarker.hpp>
#include <rwlibs/algorithms/jtag/JTagGenerator.hpp>

using namespace rwlibs::algorithms;

int main(int argc, char** argv){
	
    if (argc != 2){
        std::cerr << "Usage: " << argv[0] << " <marker id>" << std::endl;
        return -1;
    }
    int id = std::atoi(argv[1]) & 0x3F; // only 6 bit marker id
    
	// first generate some markers
    std::ostringstream filename;
    filename << "jtagmarker" << id <<".ps";
	JTagMarker mark(id);
	// draw JTag marker to a PostScript file
	JTagGenerator::DrawJTagToPS(mark, filename.str(), 100);
	
    return 0;
}
