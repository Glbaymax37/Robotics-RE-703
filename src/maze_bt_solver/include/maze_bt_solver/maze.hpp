#pragma once
#include <vector>
#include <utility>

class Maze
{
public:
  using Pos = std::pair<int, int>;

  Maze()
  {
    grid_ = {
      {'S', ' ', '#', ' ', ' '},
      {'#', ' ', '#', ' ', '#'},
      {' ', ' ', ' ', ' ', '#'},
      {'#', '#', '#', ' ', ' '},
      {' ', ' ', ' ', '#', 'G'}
    };

    start_ = {0, 0};
    goal_  = {4, 4};
    current_ = start_;
  }

  bool isGoalReached() const
  {
    return current_ == goal_;
  }

  bool move()
  {
    static std::vector<Pos> directions{
      {0,1}, {1,0}, {0,-1}, {-1,0}
    };

    for (auto d : directions)
    {
      Pos next = {current_.first + d.first, current_.second + d.second};
      if (isFree(next))
      {
        current_ = next;
        return true;
      }
    }
    return false;
  }

private:
  bool isFree(const Pos& p)
  {
    int x = p.first;
    int y = p.second;
    if (x < 0 || y < 0 || x >= 5 || y >= 5) return false;
    return grid_[x][y] != '#';
  }

  std::vector<std::vector<char>> grid_;
  Pos start_;
  Pos goal_;
  Pos current_;
};
