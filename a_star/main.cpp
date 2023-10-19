#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include <vector>
#include <algorithm>

using std::cin;
using std::cout;
using std::ifstream;
using std::istringstream;
using std::sort;
using std::string;
using std::vector;

enum class State
{
    kEmpty,
    kObstacle,
    kClosed,
    kPath,
    kStart,
    kFinish
};

/**
 * @brief Parsing lines in the file for cell contents
 *
 * @param s1
 * @return vector<State>
 */
vector<State> ParseLine(string s1)
{
    istringstream ss(s1);
    int n;
    char c;
    vector<State> v1 = {};
    while (ss.good())
    {
        if (ss >> n)
        {
            if (n == 0)
            {
                v1.push_back(State::kEmpty);
            }
            else
            {
                v1.push_back(State::kObstacle);
            }
        }
        else if (ss.fail())
        {
            ss.clear();
            ss.ignore(1);
        }
    }
    return (v1);
}

vector<vector<State>> ReadBoardFile(string filePath)
{
    ifstream file1(filePath);
    vector<vector<State>> board;
    if (file1)
    {
        string line1;
        while (getline(file1, line1))
        {
            vector<State> v1;
            v1 = ParseLine(line1);
            if (!v1.empty())
            {
                board.push_back(v1);
            }
        }
    }
    return (board);
}

/**
 * @brief Reading board from file
 *
 * @return vector<vector<State>>
 */
vector<vector<State>> ReadBoard()
{
    vector<vector<State>> board = {};
    ifstream file1("new.board");
    string line1;
    while (std::getline(file1, line1))
    {
        vector<State> lineVector = ParseLine(line1);
        if (!lineVector.empty())
        {
            board.push_back(lineVector);
        }
    }
    return board;
}

/**
 * @brief Pretty printing of the cells in the grid
 *
 * @param s1
 * @return string
 */
string CellString(State s1)
{
    switch (s1)
    {
    case State::kObstacle:
        return "⛰️   ";
    case State::kPath:
        return "🚗   ";
    case State::kStart:
        return "🚦   ";
    case State::kFinish:
        return "🏁   ";
    default:
        return "0   ";
    }
}

/**
 * @brief Prints the entire board
 *
 * @param board
 */
void PrintBoard(const vector<vector<State>> board)
{
    for (int i = 0; i < board.size(); i++)
    {
        for (int j = 0; j < board[i].size(); j++)
        {
            cout << CellString(board[i][j]);
        }
        cout << "\n";
    }
}

/**
 * @brief Calculates Heuristic value based on the manhattan distance
 *
 * @param x1
 * @param y1
 * @param x2
 * @param y2
 * @return int
 */
int Heuristic(int x1, int y1, int x2, int y2)
{
    return (std::abs(x2 - x1) + std::abs(y2 - y1));
}

/**
 * @brief Adds cell to the vector of open nodes
 *
 * @param x x coordinate
 * @param y y coordinate
 * @param g cost values
 * @param h heuristic values
 * @param openList vector of open nodes
 * @param grid entire grid
 */
void AddToOpen(int x, int y, int g, int h, vector<vector<int>> &openList, vector<vector<State>> &grid)
{
    vector<int> openLine = {x, y, g, h};
    openList.push_back(openLine);
    grid[x][y] = State::kClosed;
}

/**
 * @brief Compares vectors based on f value that is a summation of cost and heuristic value
 *
 * @param a
 * @param b
 * @return true a>b
 * @return false b>=a
 */
bool Compare(const vector<int> a, const vector<int> b)
{
    int f1 = a[2] + a[3]; // f1 = g1 + h1
    int f2 = b[2] + b[3]; // f2 = g2 + h2
    return f1 > f2;
}

/**
 * @brief Sorts all vectors based on a custom compare function
 *
 * @param v
 */
void CellSort(vector<vector<int>> *v)
{
    sort(v->begin(), v->end(), Compare);
}

/**
 * @brief Checks if coordinates x and y are on the grid and are empty
 *
 * @param x
 * @param y
 * @param grid
 * @return true
 * @return false
 */
bool CheckValidCell(int x, int y, vector<vector<State>> &grid)
{
    bool on_grid_x = (x >= 0 && x < grid.size());
    bool on_grid_y = (y >= 0 && y < grid[0].size());
    if (on_grid_x && on_grid_y)
        return grid[x][y] == State::kEmpty;
    return false;
}

/**
 * @brief Checks all cells around the currentNode and adds to open if the adjacent cell is a viable path for the search algorithm.
 * It calculates the heuristic value(h) as well as the cost(g)
 *
 * @param currentNode vector<int> Current node to investigate
 * @param goal End position on the grid to reach. This is used in heuristic value calculation
 * @param open Vector of open nodes
 * @param grid Grid of form vector<vector<State>>
 */
void ExpandNeighbors(const vector<int> &currentNode, int goal[2], vector<vector<int>> &open, vector<vector<State>> &grid)
{

    // Current node's data.
    int x = currentNode[0];
    int y = currentNode[1];

    // Delta to calculate adjacent cells
    const int delta[4][2]{{-1, 0}, {0, -1}, {1, 0}, {0, 1}};
    // Loop through current node's potential neighbors.
    for (auto i : delta)
    {
        vector<int> potentialNeighbor = {x + i[0], y + i[1]};
        // Check that the potential neighbor's x2 and y2 values are on the grid and not closed.
        if (CheckValidCell(potentialNeighbor[0], potentialNeighbor[1], grid) == true)
        {
            // Increment g value, compute h value, and add neighbor to open list.
            int g = currentNode[2] + 1;
            // Heuristic value is always computed between potential node and the end goal
            int h = Heuristic(potentialNeighbor[0], potentialNeighbor[1], goal[0], goal[1]);
            AddToOpen(potentialNeighbor[0], potentialNeighbor[1], g, h, open, grid);
        }
    }
}

/**
 * @brief Recursively searches a grid for the most optimum path
 * from init (init[0]. init[1]) position until it reach the goal (goal[0], goal[1])
 *
 * @param grid Entire grid of x and y coordinates in the form of vector<vector<State>>
 * @param init Starting position to being searching for a path
 * @param goal End position where the search should stop
 * @return vector<vector<State>>
 */
vector<vector<State>> Search(vector<vector<State>> grid, int init[2], int goal[2])
{
    // Create the vector of open nodes.
    vector<vector<int>> open{};

    // Initialize the starting node.
    int x = init[0];
    int y = init[1];
    int g = 0;
    int h = Heuristic(x, y, goal[0], goal[1]);
    AddToOpen(x, y, g, h, open, grid);

    while (open.size() > 0)
    {
        // Get the next node
        CellSort(&open);
        auto current = open.back();
        open.pop_back();
        x = current[0];
        y = current[1];
        grid[x][y] = State::kPath;

        // Check if we're done.
        if (x == goal[0] && y == goal[1])
        {
            grid[init[0]][init[1]] = State::kStart;
            grid[goal[0]][goal[1]] = State::kFinish;
            return grid;
        }

        // If we're not done, expand search to current node's neighbors.
        ExpandNeighbors(current, goal, open, grid);
    }

    // We've run out of new nodes to explore and haven't found a path.
    cout << "No path found!"
         << "\n";
    return std::vector<vector<State>>{};
}

#include "test.cpp"

int main()
{
    int init[2]{0, 0};
    int goal[2]{4, 5};
    auto board = ReadBoardFile("1.board");
    auto solution = Search(board, init, goal);
    PrintBoard(solution);
    // Tests
    TestHeuristic();
    TestAddToOpen();
    TestCompare();
    TestSearch();
    TestCheckValidCell();
    TestExpandNeighbors();
}