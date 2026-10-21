#include <iostream>
#include <vector>
#include <string>

#define STB_IMAGE_IMPLEMENTATION
#include "../include/stb_image.h"

using namespace std;


//core geometric primitive
struct Point{
    double x;
    double y;
};

//1 = foreground, 0 = background
//this is the binary image
struct BinaryGrid{
    int width;
    int height;
    //this was flattened from 2 to 1D
    vector<uint8_t> data;


    //helper to get pixel state; checks the bounds before
    u_int8_t get(int x, int y) const{
        if(x < 0 || x >= width || y < 0 || y >= height){
            return 0;
        }

        return data[y * width + x];
    }
};

//Load in the PNG and convert it to a grid of 1s and 0s
BinaryGrid loadAndBinarizePNG(const string& filename, uint8_t threshold = 128){
    int width;
    int height;
    int channels;

    //loads in gray scale to channel 1 to determine brightness
    unsigned char* pixelData = stbi_load(filename.c_str(), &width, &height, &channels, 1);

    if(!pixelData){
        cerr << "Error: Failed to load image " << filename << "\n";
        return {0, 0, {}};
    }

    BinaryGrid grid{width, height, vector<uint8_t>(width * height)};

    //map the pixels in 1s and 0s, based on threshold
    for (int i = 0; i < (width * height); ++i){
        grid.data[i] = (pixelData[i] < threshold) ? 1 : 0;
        //darker than threshold is a 1 (foreground)
    }

    //free the pixel memory
    stbi_image_free(pixelData);

    cout << "Successfully ingested image: " << width << "x" << height << " pixels.\n";
    return grid;
}

int main() {
    string testFile = "test.png";
    BinaryGrid grid = loadAndBinarizePNG(testFile);

    if(grid.width > 0) {
        cout << "Grid memory allocated: " << grid.data.size() << " bytes.\n";
    }

    return 0;
}