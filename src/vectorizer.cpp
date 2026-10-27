#include "../include/vectorizer.h"
#include "../include/stb_image.h"
#include <iostream>
#include <cmath>
#include <algorithm>

using namespace std;

// Implement BinaryGrid
BinaryGrid ImagePipeline::loadAndBinarize(const string& filePath, uint8_t threshold){
    int width;
    int height;
    int channels;
    unsigned char* pixelData = stbi_load(filePath.c_str(), &width, &height, &channels, 1);

    if(!pixelData) {
        cerr << "Error: Failed to load " << filePath << "\n";
        return{0, 0, {}};
    }

    BinaryGrid grid;
    grid.width = width;
    grid.height = height;
    grid.data.resize(width * height);

    for (int i = 0; i < width * height; ++i){
        grid.data[i] = (pixelData[i] < threshold) ? 1 : 0;
    }

    stbi_image_free(pixelData);
    return grid;
}

// ♥─────────────────────────────────────────────♥
//   Marching Squares Contour Extraction
// ♥─────────────────────────────────────────────♥

int ContourExtractor::getCellIndex(const BinaryGrid& grid, int x, int y) {
    int index = 0;
    // Read the 4 corner pixels of a 2x2 cell to form a 4-bit integer
    if (grid.get(x, y))     index |= 1; // Top-Left  (Bit 0)
    if (grid.get(x+1, y))   index |= 2; // Top-Right (Bit 1)
    if (grid.get(x+1, y+1)) index |= 4; // Bot-Right (Bit 2)
    if (grid.get(x, y+1))   index |= 8; // Bot-Left  (Bit 3)
    return index;
}

vector<Contour> ContourExtractor::extractContours(const BinaryGrid& grid) {
    vector<Contour> all_contours;
    
    
    Contour raw_mesh;
    raw_mesh.color_hex = "#000000";

    // Loop through every 2x2 cell overlapping the pixel grid
    for (int y = -1; y < grid.height; ++y) {
        for (int x = -1; x < grid.width; ++x) {
            int cell_idx = getCellIndex(grid, x, y);
            
            // Skip empty cells (0) and solid interior cells (15)
            if (cell_idx == 0 || cell_idx == 15) continue;

            // Define the center fractional coordinates of the edges of this cell
            double top_x = x + 0.5, top_y = y;
            double right_x = x + 1.0, right_y = y + 0.5;
            double bot_x = x + 0.5, bot_y = y + 1.0;
            double left_x = x, left_y = y + 0.5;

            // Generate structural line segments based on the Marching Squares configuration index
            switch (cell_idx) {
                case 1:  case 14: // Top-left corner
                    raw_mesh.points.push_back({left_x, left_y});
                    raw_mesh.points.push_back({top_x, top_y});
                    break;
                case 2:  case 13: // Top-right corner
                    raw_mesh.points.push_back({top_x, top_y});
                    raw_mesh.points.push_back({right_x, right_y});
                    break;
                case 3:  case 12: // Top horizontal line
                    raw_mesh.points.push_back({left_x, left_y});
                    raw_mesh.points.push_back({right_x, right_y});
                    break;
                case 4:  case 11: // Bottom-right corner
                    raw_mesh.points.push_back({right_x, right_y});
                    raw_mesh.points.push_back({bot_x, bot_y});
                    break;
                case 5: // Saddle point (Diagonal split)
                    raw_mesh.points.push_back({left_x, left_y});
                    raw_mesh.points.push_back({top_x, top_y});
                    raw_mesh.points.push_back({right_x, right_y});
                    raw_mesh.points.push_back({bot_x, bot_y});
                    break;
                case 6:  case 9: // Right vertical line
                    raw_mesh.points.push_back({top_x, top_y});
                    raw_mesh.points.push_back({bot_x, bot_y});
                    break;
                case 7:  case 8: // Bottom-left corner
                    raw_mesh.points.push_back({left_x, left_y});
                    raw_mesh.points.push_back({bot_x, bot_y});
                    break;
                case 10: // Saddle point (Opposite Diagonal split)
                    raw_mesh.points.push_back({top_x, top_y});
                    raw_mesh.points.push_back({right_x, right_y});
                    raw_mesh.points.push_back({left_x, left_y});
                    raw_mesh.points.push_back({bot_x, bot_y});
                    break;
            }
        }
    }

    all_contours.push_back(raw_mesh);
    cout << "Extracted raw contour path with " << raw_mesh.points.size() << " vector coordinates.\n";
    return all_contours;
}

// ♥─────────────────────────────────────────────♥
//   Ramer-Douglas-Peucker Simplification
// ♥─────────────────────────────────────────────♥
// Calculate the perpendicular distance from a point to a line segment
double PathOptimizer::perpendicularDistance(const Point& p, const Point& line_start, const Point& line_end){
    double dx = line_end.x - line_start.x;
    double dy = line_end.y - line_start.y;

    //if the line is a single poiont, it returns the Euclidean distance
    if (dx == 0.0 && dy == 0.0){
        return sqrt((p.x - line_start.x) * (p.x - line_start.x) + (p.y - line_start.y) * (p.y - line_start.y));
    }

    // point to line distance
    double numerator = abs(dy * p.x - dx * p.y + line_end.x * line_start.y - line_end.y * line_start.x);
    double denominator = sqrt(dx * dx + dy * dy);
    return numerator / denominator;
}

//Recursive divive and conquer function
void PathOptimizer::rdpRecursive(const vector<Point>& points, size_t start_idx, size_t end_idx, double epsilon, vector<bool>& keep_flags){
    if(end_idx <= start_idx + 1){
        return;
    }

    double max_dist = 0.0;
    size_t furthest_idx = start_idx;

    //finds the point with max distance from the line segment
    for(size_t i = start_idx + 1; i < end_idx; ++i){
        double dist = perpendicularDistance(points[i], points[start_idx], points[end_idx]);
        if(dist > max_dist){
            max_dist = dist;
            furthest_idx = i;
        }
    }

    //if the furthest point is beyond the threshold, split and recurse
    if(max_dist > epsilon) {
        keep_flags[furthest_idx] = true; //marks this poitn to be saves

        //split
        rdpRecursive(points, start_idx, furthest_idx, epsilon, keep_flags);
        rdpRecursive(points, furthest_idx, end_idx, epsilon, keep_flags);
    }
}

vector<Contour> PathOptimizer::simplifyContours(const vector<Contour>& raw_contours, double epsilon){
    vector<Contour> optimzed_contours;

    for (const auto& raw_contour : raw_contours){
        const auto& points = raw_contour.points;
        if(points.size() < 3){
            optimzed_contours.push_back(raw_contour);
            continue;
        }

        //initialize bool array tracking
        vector<bool> keep_flags(points.size(), false);
        keep_flags.front() = true; // Always keep the starting point
        keep_flags.back() = true;  // Always keep the ending point

        rdpRecursive(points, 0, points.size() - 1, epsilon, keep_flags);

        Contour optimzed_contour;
        optimzed_contour.color_hex = raw_contour.color_hex;

        for (size_t i = 0; i < points.size(); ++i) {
            if (keep_flags[i]) {
                optimzed_contour.points.push_back(points[i]);
            }
        }

        optimzed_contours.push_back(optimzed_contour);
        cout << "Optimized path points from " << points.size() << " down to " << optimzed_contour.points.size() << " elements.\n";
    }

    return optimzed_contours;
}