#include "jpeg.hpp"
#include "ppm.hpp" 

#define SOI		0xffd8
#define APP0	0xffe0
#define DQT		0xffdB
#define SOF0	0xffc0
#define DHT		0xffc4
#define SOS		0xffda
#define EOI		0xffd9
#define COM		0xfffe 

template<class T>
using vector2d = std::vector<std::vector<T>>;
using Block = std::array<double, 64>;

struct MCU {
	int height;
	int width;
	vector2d<Block> mcu; // mcu[#components][max_vs * max_hs]
};

struct JPEGimage {
	int height;
	int width;
	uint8_t max_hs; // max horizonral sampling factor
	uint8_t max_vs; // max vertical sampling factor
} img;

struct Component {
	uint8_t hs; // horizontal sampling factor, max 4
	uint8_t vs; // vertical sampling factor, max 4
	uint8_t qtID; // quantization table ID
	uint8_t dc_htID; // dc huffman table ID
	uint8_t ac_htID; // ac huffman table ID
} comp[4]; // comp[1]: Y, comp[2]: Cb, comp[3]: Cr

std::ofstream logfile("log.txt"); // log file
std::unordered_map<std::pair<int, int>, uint8_t, boost::hash<std::pair<int, int>>> code2symb[2][2]; // code2symb[htType][htID], (huffcodeLen, huffcode) -> symbol
Block qt[4]; // quantization table

auto get2bytes = [](std::ifstream &ifs) { return ((ifs.get() << 8) | ifs.get()); };

void readAPP0(std::ifstream &ifs) {
	int len = get2bytes(ifs) - 2; // length of segment excluding marker
	ifs.seekg(len, ifs.cur);	
}

void readCOM(std::ifstream &ifs) {
	int len = get2bytes(ifs) - 2; // length of segment excluding marker
	ifs.seekg(len, ifs.cur);	
}

void readDHT(std::ifstream &ifs) {
	int len = get2bytes(ifs) - 2; // length of segment excluding marker

	//contains one or more HTs within the same marker
	std::vector<uint8_t> leaf(16); // # of leaves at level i. (16 levels)
	std::vector<std::pair<int, int>> huffcode; // (codelen, code)

	while (len > 0) {
		int htType = ifs.get();
		int htID = htType & 0xf;
		htType >>= 4;

		int tot_leaf = 0;
		for (int i = 0, code = 0; i < 16; i++) {
			leaf[i] = ifs.get();
			tot_leaf += leaf[i];
			for (int j = 0; j < leaf[i]; j++) {
				huffcode.push_back(std::make_pair(i+1, code));
				code++;
			}
			code <<= 1;
		}

		for (int i = 0; i < tot_leaf; i++)
			code2symb[htType][htID][huffcode[i]] = ifs.get();
		len -= (1+16+tot_leaf);
		huffcode.clear();
	}
}

void readDQT(std::ifstream &ifs) {
	int len = get2bytes(ifs) - 2; // length of segment excluding marker

	// contains (len/65) QTs within the same marker
	for (int i = 0; i < len/65; i++) {
		uint8_t precision = ifs.get(); 
		uint8_t qtID = precision & 0xf;
		precision >>= 4; //0 or 1
		
		//64's 1byte or 2bytes, according to precision
		if (precision == 0) {
			for (int j = 0; j < 64; j++) 
				qt[qtID][j] = ifs.get();
		} else {
			for (int j = 0; j < 64; j++) 
				qt[qtID][j] = get2bytes(ifs);
		}
	}
}

void readSOF0(std::ifstream &ifs) {
	int len = get2bytes(ifs) - 2; // length of segment excluding marker
	ifs.seekg(1, ifs.cur); // precision, baseline JPEG is 8
	img.height = get2bytes(ifs);
	img.width = get2bytes(ifs);
	ifs.seekg(1, ifs.cur); // # of components, 3

	for (int i = 0; i < 3; i++) {
		uint8_t compID = ifs.get();
		uint8_t sampFactor = ifs.get(); // sampling factor
		comp[compID].hs = sampFactor >> 4; 
		comp[compID].vs = sampFactor & 0xF;
		img.max_hs = std::max(img.max_hs, comp[compID].hs);
		img.max_vs = std::max(img.max_vs, comp[compID].vs);
		comp[compID].qtID = ifs.get();
	}
}

double idct_table[150];
void create_idct_table() {
	for (int i = 0; i < 150; i++)
		idct_table[i] = cos(i*M_PI/16.0);
}

void buildBlock(Block &block, int k) {
	std::transform(block.begin(), block.end(), qt[comp[k].qtID].begin(), block.begin(), std::multiplies<double>());
	Block zigzag = {
		 0,  1,  5,  6, 14, 15, 27, 28,
		 2,  4,  7, 13, 16, 26, 29, 42,
		 3,  8, 12, 17, 25, 30, 41, 43,
		 9, 11, 18, 24, 31, 40, 44, 53,
		10, 19, 23, 32, 39, 45, 52, 54,
		20, 22, 33, 38, 46, 51, 55, 60,
		21, 34, 37, 47, 50, 56, 59, 61,
		35, 36, 48, 49, 57, 58, 62, 63
	};
	for (int b = 0; b < 64; b++)
		zigzag[b] = block[zigzag[b]];

	//idct
	auto C = [](int x) -> double { return (x == 0)? 1.0/sqrt(2): 1; };
	Block s = {};
	for (int j = 0; j < 8; j++)
		for (int x = 0; x < 8; x++)
			for (int y = 0; y < 8; y++)
				s[j*8+x] += C(y) * zigzag[x*8+y] * idct_table[(2*j+1)*y];
	Block tmp = {};
	for (int i = 0; i < 8; i++) {
		for (int j = 0; j < 8; j++) {
			for (int x = 0; x < 8; x++) {
				tmp[i*8+j] += C(x) * s[j*8+x] * idct_table[(2*i+1)*x];
			}
			tmp[i*8+j] /= 4;
		}
	}
	block = tmp;
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
        if (code2symb[htType][htID].find(std::make_pair(codeLen, code)) != code2symb[htType][htID].end()) 
			return code2symb[htType][htID][std::make_pair(codeLen, code)];
    }
	std::cerr << "Could not find symbol" << std::endl;
	exit(1);
}

int get_coeff(std::ifstream &ifs, uint8_t nbits) {
	auto flipNbits = [](int code, uint8_t nbits) { return (1<<nbits)-1 - code; };
	int code = 0;
	for (int i = 0; i < nbits; i++)
		code = (code<<1) + getBit(ifs);
    return (code >> (nbits-1))? code: -flipNbits(code, nbits);
}

void readMCU(std::ifstream &ifs, MCU &mcu) {
	for (int k = 1; k <= 3; k++) { 
		for (int i = 0; i < comp[k].vs; i++) {
			for (int j = 0; j < comp[k].hs; j++) {
				//1 DC coefficient
				static int dc_coeff[4] = {0}; //  Y:1, Cb:2, Cr:3
				uint8_t symbol = getSymbol(ifs, 0, comp[k].dc_htID); // read next (symbol)'s bits
				if (symbol == 0) {
					mcu.mcu[k][i*img.max_hs+j][0] = dc_coeff[k];
				} else {
					mcu.mcu[k][i*img.max_hs+j][0] = (dc_coeff[k] += get_coeff(ifs, symbol)); //dc[i] = dc[i-1] + diff
				}

				//63 AC coefficients
				uint8_t nzeros;
				uint8_t nbits;
				for (int ac_coeff_cnt = 1; ac_coeff_cnt < 64;) {
					symbol = getSymbol(ifs, 1, comp[k].ac_htID);
					if (symbol == 0) { 
						// all remaining AC coefficients are 0
						for (;ac_coeff_cnt < 64; ac_coeff_cnt++)
							mcu.mcu[k][i*img.max_hs+j][ac_coeff_cnt] = 0;
						break;
					} else if (symbol == 0xf0) { 
						// next 16 AC coefficients are 0
						for (int c = 0; c < 16; c++, ac_coeff_cnt++)
							mcu.mcu[k][i*img.max_hs+j][ac_coeff_cnt] = 0;
					} else {
						nzeros = symbol >> 4; // next (nzeros)'s AC coefficients are 0
						nbits = symbol & 0xf; // read next (nbits)'s bits
						for (int c = 0; c < nzeros; c++, ac_coeff_cnt++)
							mcu.mcu[k][i*img.max_hs+j][ac_coeff_cnt] = 0;
						mcu.mcu[k][i*img.max_hs+j][ac_coeff_cnt] = get_coeff(ifs, nbits);
						ac_coeff_cnt++;
					}
				}
				buildBlock(mcu.mcu[k][i*img.max_hs+j], k);
			}
		}
    }
}

rgb_t convert2rgb(MCU &mcu, int x, int y) {
	int mx, my;
	mx = x * comp[1].vs / img.max_vs;
	my = y * comp[1].hs / img.max_hs;
	double Y = mcu.mcu[1][mx/8*img.max_hs + my/8][(mx%8)*8 + (my%8)];
	mx = x * comp[2].vs / img.max_vs;
	my = y * comp[2].hs / img.max_hs;
	double Cb = mcu.mcu[2][mx/8*img.max_hs + my/8][(mx%8)*8 + (my%8)];
	mx = x * comp[3].vs / img.max_vs;
	my = y * comp[3].hs / img.max_hs;
	double Cr = mcu.mcu[3][mx/8*img.max_hs + my/8][(mx%8)*8 + (my%8)];
	auto clamp = [](double x) -> uint8_t { return (x > 255)? 255: ((x < 0)? 0: x); };
	return {clamp(Y+1.402*Cr+128), clamp(Y-0.34414*Cb-0.71414*Cr+128), clamp(Y+1.772*Cb+128)};
}


void readImageData(std::ifstream &ifs) {
	//MCU's height: img.max_vs * 8
	//MCU's width:  img.max_hs * 8
	MCU mcu = {img.max_vs*8, img.max_hs*8, vector2d<Block>(4, std::vector<Block>(img.max_vs*img.max_hs))}; 
	int row = (img.height-1) / mcu.height + 1;
	int col = (img.width-1) / mcu.width + 1;
	
	RGBimage im(mcu.height*row, mcu.width*col);
	create_idct_table();
	for (int i = 0; i < row; i++) {
		for (int j = 0; j < col; j++) {
			readMCU(ifs, mcu);
			rgb_t color;
			for (int x = 0; x < mcu.height; x++) {
				for (int y = 0; y < mcu.width; y++) {
					color = convert2rgb(mcu, x, y);
					im.writePixel(x+i*mcu.height, y+j*mcu.width, color);
				}
			}

		}
	}
	im.outputPPM("out.ppm");
}

void readSOS(std::ifstream &ifs) {
	int len = get2bytes(ifs) - 2; // length of segment excluding marker
	ifs.seekg(1, ifs.cur); // # of components
	for (int i = 0; i < 3; i++) {
		uint8_t compID = ifs.get();
		uint8_t htID = ifs.get();
		comp[compID].dc_htID = htID >> 4;
		comp[compID].ac_htID = htID & 0xf;
	}
	ifs.seekg(3, ifs.cur);
	readImageData(ifs);
}

namespace myJpeg {
void decode(const char *filename) {
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
				break;
			case APP0:
				readAPP0(ifs);
				break;
			case SOF0:
				readSOF0(ifs);
				break;
			case DQT:
				readDQT(ifs);
				break;
			case DHT:
				readDHT(ifs);
				break;
			case SOS:
				readSOS(ifs);
				break;
			case EOI:
				break;
			case COM:
				readCOM(ifs);
				break;
			default:
				break;
		}
	}
	ifs.close();
}
}
