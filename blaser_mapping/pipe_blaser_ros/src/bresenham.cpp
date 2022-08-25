//
// Created by dcheng on 1/20/21.
//
#include <pipe_blaser_ros/bresenham.h>

void bhm_line(int x1, int y1, int x2, int y2, std::vector<cv::Point2i>& pts)
{
  pts.clear();
  int x, y, dx, dy, dx1, dy1, px, py, xe, ye, i;
  dx = x2 - x1;
  dy = y2 - y1;
  dx1 = fabs(dx);
  dy1 = fabs(dy);
  px = 2 * dy1 - dx1;
  py = 2 * dx1 - dy1;

  // if |slope| < 1
  if (dy1 <= dx1)
  {
    if (dx >= 0)
    {
      x = x1;
      y = y1;
      xe = x2;
    } else
    {
      x = x2;
      y = y2;
      xe = x1;
    }
    pts.emplace_back(x, y);
    for (i = 0; x < xe; i++)
    {
      x = x + 1;
      if (px < 0)
      {
        px = px + 2 * dy1;
      } else
      {
        if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0))
        {
          y = y + 1;
        } else
        {
          y = y - 1;
        }
        px = px + 2 * (dy1 - dx1);
      }
      pts.emplace_back(x, y);
    }
  }
  else // if |slope| > 1
  {
    if (dy >= 0)
    {
      x = x1;
      y = y1;
      ye = y2;
    } else
    {
      x = x2;
      y = y2;
      ye = y1;
    }
    pts.emplace_back(x, y);
    for (i = 0; y < ye; i++)
    {
      y = y + 1;
      if (py <= 0)
      {
        py = py + 2 * dx1;
      } else
      {
        if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0))
        {
          x = x + 1;
        } else
        {
          x = x - 1;
        }
        py = py + 2 * (dx1 - dy1);
      }
      pts.emplace_back(x, y);
    }
  }
}