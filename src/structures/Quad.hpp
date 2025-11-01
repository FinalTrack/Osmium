#pragma once
#include <array>
#include <vector>
#include <cmath>

//Maximum number of cells in QuadGrid
inline const int MAXSZ = 1<<18;

struct QuadGrid 
{
    int limit; //minimum cell size (in world units) for stopping the level subdivision
    int length; //smallest power-of-two length that covers the world extents

    std::array<std::vector<int>, MAXSZ> grid; 
    std::vector<int> levels; //level base indices (levels[i] = start index of level i in flat grid array)
    std::vector<int> occ; // occupancy flag per level

    QuadGrid(int worldSize, int lim = 16)
    {
        limit = lim;
        length = 1;
        while(length < worldSize)
            length <<= 1;
        
        int cnt = 1;
        int ind = 0;
        int tmp = length;
        while(tmp >= lim)
        {
            levels.push_back(ind);
            occ.push_back(0);
            ind += cnt;
            cnt *= 4;
            tmp >>= 1;
        }
    }

    // Get index of a specific grid cell
    int getIndex(int lvl, int x, int y)
    {
        int cnt = 1<<lvl;
        if(x < 0 || x >= cnt || y < 0 || y >= cnt)
            return -1;
        return levels[lvl] + y * (1<<lvl) + x;
    }

    //Get smallest level greater than sz
    int getLevel(float sz)
    {
        int lvl = 0;
        int curr = length;
        while(lvl < (int)levels.size()-1 && (curr >> 1) >= sz)
        {
            curr >>= 1;
            lvl++;
        }
        return lvl;
    }

    //Convert world coordinates to grid coordinates
    void gridCoord(int& gx, int& gy, int lvl, float x, float y)
    {
        int curr = length >> lvl;
        gx = (int)floor(x / curr);
        gy = (int)floor(y / curr);
    }
};
