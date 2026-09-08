#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "routing/grid.hpp"

using namespace routing;

TEST_CASE("coord_round_trip") {
    Grid g(1, 100, 100, 10.0);
    // to_cell floors the coordinate; to_coord returns cell center.
    Cell c = g.to_cell(37.0, 12.0, 0);
    CHECK(c.x == 3);
    CHECK(c.y == 1);
    Coord back = g.to_coord(c);
    // center of cell (3,1): x=35, y=15; within half a resolution of original
    CHECK(std::abs(back.x - 37.0) <= 5.0 + 1e-9);
    CHECK(std::abs(back.y - 12.0) <= 5.0 + 1e-9);
}

TEST_CASE("dimensions_and_bounds") {
    Grid g(3, 50, 40, 5.0);
    CHECK(g.layers() == 3);
    CHECK(g.width() == 50);
    CHECK(g.height() == 40);
    CHECK(g.size() == 3ull * 50 * 40);
    CHECK(g.valid(0, 0, 0));
    CHECK(g.valid(2, 49, 39));
    CHECK(!g.valid(0, 50, 0));
    CHECK(!g.valid(3, 0, 0));
    CHECK(!g.valid(0, 0, -1));
}

TEST_CASE("layer_stride_contiguous") {
    Grid g(4, 2, 3, 1.0);
    // index(1,0,0) should be exactly width*height past index(0,0,0)
    CHECK(g.index(1, 0, 0) == 6ull);
    CHECK(g.index(0, 1, 2) == 5ull);   // (0*2+2)*3+1 == 5
    CHECK(g.index(1, 0, 0) - g.index(0, 0, 0) == 6ull);
}

TEST_CASE("cell_index_round_trip") {
    Grid g(2, 7, 5, 2.0);
    for (int l = 0; l < 2; ++l) {
        for (int y = 0; y < 5; ++y) {
            for (int x = 0; x < 7; ++x) {
                Cell c{l, x, y};
                Cell back = g.unindex(g.index(c));
                CHECK(back == c);
            }
        }
    }
}

TEST_CASE("cost_access") {
    Grid g(1, 4, 4, 1.0);
    g.at(0, 1, 2) = 9.0;
    CHECK(g.at(0, 1, 2) == doctest::Approx(9.0));
    // const access agrees
    const Grid& cg = g;
    CHECK(cg.at(Cell{0, 1, 2}) == doctest::Approx(9.0));
    g.fill_all(3.0);
    CHECK(g.at(0, 0, 0) == doctest::Approx(3.0));
    CHECK(g.at(0, 3, 3) == doctest::Approx(3.0));
}

TEST_CASE("bad_dimensions_throw") {
    CHECK_THROWS_AS(Grid(0, 10, 10, 1.0), std::invalid_argument);
    CHECK_THROWS_AS(Grid(1, 0, 10, 1.0), std::invalid_argument);
    CHECK_THROWS_AS(Grid(1, 10, 10, 0.0), std::invalid_argument);
    // Negative dimensions must throw std::invalid_argument (regression: previously
    // the backing array was allocated before validation, throwing length_error).
    CHECK_THROWS_AS(Grid(1, -1, 10, 1.0), std::invalid_argument);
    CHECK_THROWS_AS(Grid(-2, 10, 10, 1.0), std::invalid_argument);
    CHECK_THROWS_AS(Grid(1, 10, -3, 1.0), std::invalid_argument);
}
