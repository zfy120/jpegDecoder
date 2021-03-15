#include "jpeg.hpp"
#include <filesystem>
#include <boost/algorithm/string.hpp>    
int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "Usage: " <<  argv[0]  << " [JPEG image]" << std::endl;
        return 0;
    }
	std::string ext = std::filesystem::path(argv[1]).extension();
	boost::algorithm::to_lower(ext);
	if (ext != ".jpg" && ext != ".jpeg") {
		std::cerr << "Not JPEG image." << std::endl;
		exit(1);
	}
	myJpeg::decode(argv[1]);
    return 0;
}
