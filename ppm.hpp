#pragma once

struct rgb_t {
	uint8_t r;
	uint8_t g;
	uint8_t b;
};

class RGBimage {
public:
	RGBimage(int h, int w) : height(h), width(w) {
		pixel = new rgb_t[height*width];
	}

	~RGBimage() {
		if (pixel != nullptr) delete[] pixel;
		pixel = nullptr;
	}

	void writePixel(int x, int y, rgb_t color);
	void outputPPM(std::string filename);

private:
	rgb_t *pixel;
	int width;
	int height;
};

void RGBimage::writePixel(int x, int y, rgb_t color) {
	pixel[x*width + y] = color;
}

void RGBimage::outputPPM(std::string filename) {
	std::ofstream ofs(filename, std::ios::binary);
	if (!ofs.is_open()) {
		std::cerr << "Could not open " << filename << std::endl;
		exit(1);
	}
	ofs << "P6 " << width << " " << height << " 255\n";
	ofs.write(reinterpret_cast<char*>(pixel), height*width*3);
	ofs.close();
}
