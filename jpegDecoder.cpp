#include "jpeg.hpp"
#include <vector> 
#include <array> 
#include <map>
#include <utility> 
#include <functional> 
#include <bitset> 

#define SOI  0xffd8
#define APP0 0xffe0
#define DQT  0xffdB
#define SOF0 0xffc0
#define DHT  0xffc4
#define SOS  0xffda
#define EOI  0xffd9

template<class T>
using vector2d = std::vector< std::vector<T> >;
template<class T>
using vector3d = std::vector< vector2d<T> >;
using pairInt_t = std::pair<int, int>;
using Block = std::array<std::array<double, 8>, 8>;

class MCU {
public:
	MCU(int max_vs, int max_hs)
		: height(max_vs*8), width(max_hs*8), mcu(4, vector2d<Block>(max_vs, std::vector<Block>(max_hs))) 
	{}

	int height;
	int width;
	vector3d<Block> mcu; // mcu[#components][max vs][max hs]
	void decodeMCU();
};

struct rgb_t {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
};

struct image {
	int height;
	int width;
	int max_hs; // max horizonral sampling factor
	int max_vs; // max vertical sampling factor
	vector2d<rgb_t> pixels; //RGB pixels
} img;

struct component {
	int hs; // horizontal sampling factor, max 4
	int vs; // vertical sampling factor, max 4
	int qtID; // quantization table ID
	uint8_t dc_htID; // dc huffman table ID
	uint8_t ac_htID; // ac huffman table ID
} comp[4]; // comp[1]: Y, comp[2]: Cb, comp[3]: Cr

std::ofstream logfile("log.txt"); // log file
std::map<pairInt_t, int> code2symb[2][2]; // code2symb[htType][htID], (huffcodeLen, huffcode) -> symbol
int qt[4][64]; // quantization table

auto get2bytes = [](std::ifstream &ifs) { return ((ifs.get() << 8) | ifs.get()); };

void readAPP0(std::ifstream &ifs) {
	int len = get2bytes(ifs) - 2; // length of segment excluding marker
	ifs.seekg(len, ifs.cur);	
}

void readDHT(std::ifstream &ifs) {
	int len = get2bytes(ifs) - 2; // length of segment excluding marker

	//contains one or more HTs within the same marker
	std::vector<int> leaf(16); // # of leaves at level i, max 16 levels
	std::vector<pairInt_t> huffcode; // (codelen, code)

	while (len > 0) {
		int htType = ifs.get();
		int htID = htType & 0xf;
		htType >>= 4;

		int tot_leaf = 0;
		for (int i = 0; i < 16; i++) {
			leaf[i] = ifs.get();
			tot_leaf += leaf[i];
		}

		int code = 0;
		for (int i = 0; i < 16; i++) {
			for (int j = 0; j < leaf[i]; j++) {
				huffcode.push_back(std::make_pair(i+1, code));
				code++;
			}
			code <<= 1;
		}

		for (int i = 0; i < tot_leaf; i++)
			code2symb[htType][htID][huffcode[i]] = ifs.get();
	
		len -= (1+16+tot_leaf);
#ifdef DEBUG
		logfile << htType << " " << htID << '\n';
		for (int i = 0; i < tot_leaf; i++) {
			logfile << "(" << huffcode[i].first << ", " << huffcode[i].second << ") -> " << code2symb[htType][htID][huffcode[i]] << '\n';
		}
#endif
		
	}
}

void readDQT(std::ifstream &ifs) {
	int len = get2bytes(ifs) - 2; // length of segment excluding marker

	// contains (len/65) QTs within the same marker
	for (int i = 0; i < len/65; i++) {
		int precision = ifs.get(); // 0 or 1
		int qtID = precision & 0xf;
		precision >>= 4;
		
		//64's 1byte or 2bytes, according to precision
		if (precision == 0) {
			for (int j = 0; j < 64; j++) {
				qt[qtID][j] = ifs.get();
			}
		} else {
			for (int j = 0; j < 64; j++) {
				qt[qtID][j] = get2bytes(ifs);
			}
		}
#ifdef DEBUG
		logfile << "qtID: " << qtID << '\n';
		for (int j = 0; j < 64; j++) {
			logfile << qt[qtID][j] << " \n"[j % 8 == 7];
		}
#endif
	}
}

void readSOF0(std::ifstream &ifs) {
	int len = get2bytes(ifs) - 2; // length of segment excluding marker
	logfile << "segment length: " << len+2 << '\n';
	ifs.seekg(1, ifs.cur); // precision, baseline JPEG is 8
	img.height = get2bytes(ifs);
	img.width = get2bytes(ifs);
	logfile << "(image height, image width): (" << img.height << ", " << img.width << ")\n";
	ifs.seekg(1, ifs.cur); // # of components, 3

	for (int i = 0; i < 3; i++) {
		int compID = ifs.get();
		int sampFactor = ifs.get(); // sampling factor
		comp[compID].hs = sampFactor >> 4; 
		comp[compID].vs = sampFactor & 0xF;
		img.max_hs = std::max(img.max_hs, comp[compID].hs);
		img.max_vs = std::max(img.max_vs, comp[compID].vs);
		comp[compID].qtID = ifs.get();
		logfile << "compID: " << compID << '\n';
		logfile << "hs: " << comp[compID].hs << '\n';
		logfile << "vs: " << comp[compID].vs << '\n';
		logfile << "qtID: " << comp[compID].qtID << '\n';
	}
	logfile << "max_hs: " << img.max_hs << '\n';
	logfile << "max_vs: " << img.max_vs << '\n';
}



void MCU::decodeMCU() {
	
}

uint8_t getBit(std::ifstream &ifs) {
	static uint8_t buf; //buffer of 8 bits
    static uint8_t idx = 0;
    if ((idx %= 8) == 0) {
		buf = ifs.get();
        if (buf == 0xff) {
			if (ifs.get() != 0) {
				std::cerr << "0xff did not followed by 0x00" << std::endl;
				exit(1);
            }
        }
    }
    return (buf >> (7-idx++)) & 1;
}

uint8_t getSymbol(std::ifstream &ifs, uint8_t htType, uint8_t htID) {
    int code = 0;
	int codeLen = 0;
	// max codeLen is 16
	while (codeLen < 16) {
        code = (code<<1) + getBit(ifs);
		codeLen++;
        if (code2symb[htType][htID].find(std::make_pair(codeLen, code)) != code2symb[htType][htID].end()) {
			return code2symb[htType][htID][std::make_pair(codeLen, code)];
		}
    }
	std::cerr << "Could not find symbol" << std::endl;
	exit(1);
}

int get_coeff(std::ifstream &ifs, uint8_t nbits) {
    uint8_t firstb = getBit(ifs), b;
    int code = 1;
    for (int i = 1; i < nbits; i++) {
        b = getBit(ifs);
        code <<= 1;
        code += (firstb)? b : !b;
    }
    return (firstb)? code: -code;
}

void readMCU(std::ifstream &ifs, MCU &mcu) {
	int code;
	for (int k = 1; k <= 3; k++) { // YCbCr
		for (int i = 0; i < comp[k].vs; i++) {
			for (int j = 0; j < comp[k].hs; j++) {


				//1 DC coefficient
				uint8_t symbol;
				static int dc_coeff[4] = {0}; //  Y:1, Cb:2, Cr:3
				symbol = getSymbol(ifs, 0, comp[k].dc_htID); // read next (symbol)'s bits
				if (symbol == 0) {
					mcu.mcu[k][i][j][0][0] = dc_coeff[k];
				} else {
					code = get_coeff(ifs, symbol);
					mcu.mcu[k][i][j][0][0] = (dc_coeff[k] += code); //dc[i] = dc[i-1] + diff
					//logfile << "DC: len " << (int)symbol << ", code " << code << std::endl;
				}

				//63 AC coefficients
				uint8_t nzeros;
				uint8_t nbits;
				for (int ac_coeff_cnt = 1; ac_coeff_cnt < 64;) {
					symbol = getSymbol(ifs, 1, comp[k].ac_htID);
					if (symbol == 0) { 
						// all remaining AC coefficients are 0
						for (;ac_coeff_cnt < 64; ac_coeff_cnt++)
							mcu.mcu[k][i][j][ac_coeff_cnt/8][ac_coeff_cnt%8] = 0;
						break;
					} else if (symbol == 0xf0) { 
						// next 16 AC coefficients are 0
						for (int c = 0; c < 16; c++, ac_coeff_cnt++)
							mcu.mcu[k][i][j][ac_coeff_cnt/8][ac_coeff_cnt%8] = 0;
					} else {
						nzeros = symbol >> 4; // next (nzeros)'s AC coefficients are 0
						nbits = symbol & 0xf; // read next (nbits)'s bits
						for (int c = 0; c < nzeros; c++, ac_coeff_cnt++)
							mcu.mcu[k][i][j][ac_coeff_cnt/8][ac_coeff_cnt%8] = 0;
						code = get_coeff(ifs, nbits);
						mcu.mcu[k][i][j][ac_coeff_cnt/8][ac_coeff_cnt%8] = code;
						ac_coeff_cnt++;
						//logfile << "AC: " << (int)nbits << " " << (int)nzeros << " " << code << std::endl;
					}
				}
			}
		}
    }
}

void readImageData(std::ifstream &ifs) {
	logfile << "========== Compressed Data ==========" << std::endl;
	//MCU's height = max_vs * 8
	//MCU's width  = max_hs * 8
	MCU mcu(img.max_vs, img.max_hs);
	int row = (img.height-1) / mcu.height + 1;
	int col = (img.width-1) / mcu.width + 1;

	//(row*col)'s MCUs
	for (int i = 0; i < row; i++) {
		for (int j = 0; j < col; j++) {
			readMCU(ifs, mcu);
			mcu.decodeMCU();
			//TODO
		}
	}
}

void readSOS(std::ifstream &ifs) {
	int len = get2bytes(ifs) - 2; // length of segment excluding marker
	ifs.seekg(1, ifs.cur); // # of components
	uint8_t compID;
	uint8_t htID;
	for (int i = 0; i < 3; i++) {
		compID = ifs.get();
		htID = ifs.get();
		comp[compID].dc_htID = htID >> 4;
		comp[compID].ac_htID = htID & 0xf;
		logfile << "compID: " << (int)compID << std::endl; 
		logfile << "dc_htID: " << (int)comp[compID].dc_htID << std::endl; 
		logfile << "ac_gtID: " << (int)comp[compID].dc_htID << std::endl; 
	}
	ifs.seekg(3, ifs.cur);
	readImageData(ifs);
}

void myJpeg::decode(const char *filename) {
	std::ifstream ifs(filename, std::ios::binary);
	if (!ifs.is_open()) {
		std::cerr << "Could not open " << filename << std::endl;
		exit(1);
	}

	int marker = 0;
	while (marker != EOI) {
		marker = get2bytes(ifs);
		switch(marker) {
			case SOI:
				logfile << "========== SOI ==========" << std::endl;
				break;
			case APP0:
				logfile << "========== APP0 ==========" << std::endl;
				readAPP0(ifs);
				break;
			case SOF0:
				logfile << "========== SOF0 ==========" << std::endl;
				readSOF0(ifs);
				break;
			case DQT:
				logfile << "========== DQT ==========" << std::endl;
				readDQT(ifs);
				break;
			case DHT:
				logfile << "========== DHT ==========" << std::endl;
				readDHT(ifs);
				break;
			case SOS:
				logfile << "========== SOS ==========" << std::endl;
				readSOS(ifs);
				break;
			case EOI:
				logfile << "========== EOI ==========" << std::endl;
				break;
			default:
				break;
		}
	}
}

