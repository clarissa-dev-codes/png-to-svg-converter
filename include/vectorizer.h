#ifndef VECTORIZER_H
#define VECTORIZER_H

#include <vector>
#include <string>
#include <cstdint>

using namespace std;

// ♥─────────────────────────────────────────────♥
//   Core Data Structerus
// ♥─────────────────────────────────────────────♥

//2D geo coordinates
struct Point {
    double x;
    double y;
};

//closed vector shape
struct Contour{
    vector<Point> points;
    string color_hex; //starts as #000000 for binary tracking
};

//Binary image (1 = shape, 0 = background)
class BinaryGrid{
    public:
        int width = 0;
        int height = 0;
        vector<uint8_t> data;

        //returns 0 if the pixels are out of bounds
        uint8_t get(int x, int y) const{
            if(x < 0 || x>= width || y < 0 || y >= height){
                return 0;
            }
            return data[y * width + x];
        }
};

// ♥─────────────────────────────────────────────♥
//   Algorithm Implementations and Pipelines
// ♥─────────────────────────────────────────────♥

class ImagePipeline {
    public:
        // Decodes PNG and creates a binary 1/0 grid based on a brightness threshold
        static BinaryGrid loadAndBinarize(const std::string& filepath, uint8_t threshold = 128);
};

class ContourExtractor {
    public:
        // Runs the Marching Squares algorithm across the grid to extract raw boundary contours
        static std::vector<Contour> extractContours(const BinaryGrid& grid);

    private:
        // Internal helper to evaluate a 2x2 pixel cell and return its Marching Squares lookup configuration index (0 to 15)
        static int getCellIndex(const BinaryGrid& grid, int x, int y);
};

class PathOptimizer {
    public:
        // Simplifies jagged pixel coordinates using the Ramer-Douglas-Peucker (RDP) algorithm
        static std::vector<Contour> simplifyContours(const std::vector<Contour>& raw_contours, double epsilon = 1.0);

    private:
        // Core recursive RDP function that operates on a single slice of points
        static void rdpRecursive(const std::vector<Point>& points, size_t start_idx, size_t end_idx, double epsilon, std::vector<bool>& keep_flags);
        
        // Internal helper to calculate the perpendicular distance from a point to a line segment
        static double perpendicularDistance(const Point& p, const Point& line_start, const Point& line_end);
};

class SVGExporter {
    public:
        // Formats geometric vector data into an XML-compliant .svg file
        static bool saveToSVG(const std::string& filepath, const std::vector<Contour>& contours, int width, int height);
};
#endif