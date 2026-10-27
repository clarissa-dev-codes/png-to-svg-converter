#include <iostream>
#include <vector>
#include <string>


#define STB_IMAGE_IMPLEMENTATION
#include "../include/stb_image.h"
#include "../include/vectorizer.h"

using namespace std;

int main() {
    string testFile = "test.png";
    
    // 1. Ingest image using our centralized class pipeline
    BinaryGrid grid = ImagePipeline::loadAndBinarize(testFile, 128);
    
    if (grid.width == 0) {
        cerr << "Failed to load and binarize the image.\n";
        return -1; 
    }
    
    // 2. Run Marching Squares contour extraction
    vector<Contour> raw_contours = ContourExtractor::extractContours(grid);
    
    cout << "Phase 1 & Phase 2 compiled and executed successfully!\n";
    return 0;
}
