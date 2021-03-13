#include "jpeg.hpp"

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "Usage: " <<  argv[0]  << " [input file]" << std::endl;
        return 0;
    }
	myJpeg::decode(argv[1]);
    return 0;
}
