#pragma once

#include <cstdint>
#include <vector>
#include <stdexcept>
#include <cstddef>

namespace routing {

// A continuous (x,y) coordinate in board units (e.g., nanometers).
struct Coord {
    double x = 0.0;
    double y = 0.0;
};

// A grid cell index in (layer, x, y) space. All are integer cell coordinates.
struct Cell {
    int layer = 0;
    int x = 0;
    int y = 0;

    bool operator==(const Cell& o) const {
        return layer == o.layer && x == o.x && y == o.y;
    }
    bool operator!=(const Cell& o) const { return !(*this == o); }

    // Total ordering (for use in std::set / std::map).
    bool operator<(const Cell& o) const {
        if (layer != o.layer) return layer < o.layer;
        if (x != o.x) return x < o.x;
        return y < o.y;
    }
};

// Linear index for cells, with a stable contiguous memory layout:
//   idx = ((layer * height) + y) * width + x
// Grid constructor: validates dimensions BEFORE allocating the backing array so
// invalid dimensions throw std::invalid_argument (not std::length_error).
class Grid {
private:
    static size_t checked_size(int layers, int width, int height) {
        if (layers <= 0 || width <= 0 || height <= 0) {
            throw std::invalid_argument("Grid dimensions must be positive");
        }
        return static_cast<size_t>(layers) * width * height;
    }

public:
    Grid(int layers, int width, int height, double resolution)
        : layers_(layers), width_(width), height_(height), res_(resolution),
          data_(checked_size(layers, width, height), 0.0) {
        if (resolution <= 0.0) {
            throw std::invalid_argument("Grid resolution must be positive");
        }
    }

    int layers() const { return layers_; }
    int width() const { return width_; }
    int height() const { return height_; }
    double resolution() const { return res_; }

    size_t size() const { return data_.size(); }

    // -- Cell <-> linear index -------------------------------------------
    size_t index(int layer, int x, int y) const {
        return (static_cast<size_t>(layer) * height_ + y) * width_ + x;
    }
    size_t index(const Cell& c) const { return index(c.layer, c.x, c.y); }

    bool valid(int layer, int x, int y) const {
        return layer >= 0 && layer < layers_ &&
               x >= 0 && x < width_ &&
               y >= 0 && y < height_;
    }
    bool valid(const Cell& c) const { return valid(c.layer, c.x, c.y); }

    Cell unindex(size_t i) const {
        size_t w = static_cast<size_t>(width_);
        size_t h = static_cast<size_t>(height_);
        int layer = static_cast<int>(i / (w * h));
        size_t rem = i % (w * h);
        int y = static_cast<int>(rem / w);
        int x = static_cast<int>(rem % w);
        return Cell{layer, x, y};
    }

    // -- Coordinate <-> cell ---------------------------------------------
    // Quantizes a board coordinate (nanometers) to the nearest grid cell (floor).
    Cell to_cell(double x, double y, int layer = 0) const {
        int cx = static_cast<int>(x / res_);
        int cy = static_cast<int>(y / res_);
        return Cell{layer, cx, cy};
    }
    // Converts a cell back to board coordinates (center of the cell).
    Coord to_coord(const Cell& c) const {
        return Coord{(static_cast<double>(c.x) + 0.5) * res_,
                     (static_cast<double>(c.y) + 0.5) * res_};
    }

    // -- Cost access (mutable) -------------------------------------------
    double& at(int layer, int x, int y) {
        return data_[index(layer, x, y)];
    }
    double at(int layer, int x, int y) const {
        return data_[index(layer, x, y)];
    }
    double& at(const Cell& c) { return data_[index(c)]; }
    double at(const Cell& c) const { return data_[index(c)]; }

    // Fill
    void fill_all(double v) { std::fill(data_.begin(), data_.end(), v); }

    // Raw contiguous data (for observation export / fast ops).
    std::vector<double>& data() { return data_; }
    const std::vector<double>& data() const { return data_; }

private:
    int layers_;
    int width_;
    int height_;
    double res_;
    std::vector<double> data_;
};

} // namespace routing
