#include "cvskeleton.h"
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

namespace cvskeleton {
	using namespace std;
	using namespace cv;

	cv::Mat thinImage(const cv::Mat & src, const int maxIterations)
	{
		assert(src.type() == CV_8UC1);
		cv::Mat dst;
		int width = src.cols;
		int height = src.rows;
		src.copyTo(dst);
		int count = 0;  //记录迭代次数    
		while (true)
		{
			count++;
			if (maxIterations != -1 && count > maxIterations) //限制次数并且迭代次数到达    
				break;
			std::vector<uchar *> mFlag; //用于标记需要删除的点    
										//对点标记    
			for (int i = 0; i < height; ++i)
			{
				uchar * p = dst.ptr<uchar>(i);
				for (int j = 0; j < width; ++j)
				{
					//如果满足四个条件，进行标记    
					//  p9 p2 p3    
					//  p8 p1 p4    
					//  p7 p6 p5    
					uchar p1 = p[j];
					if (p1 != 1) continue;
					uchar p4 = (j == width - 1) ? 0 : *(p + j + 1);
					uchar p8 = (j == 0) ? 0 : *(p + j - 1);
					uchar p2 = (i == 0) ? 0 : *(p - dst.step + j);
					uchar p3 = (i == 0 || j == width - 1) ? 0 : *(p - dst.step + j + 1);
					uchar p9 = (i == 0 || j == 0) ? 0 : *(p - dst.step + j - 1);
					uchar p6 = (i == height - 1) ? 0 : *(p + dst.step + j);
					uchar p5 = (i == height - 1 || j == width - 1) ? 0 : *(p + dst.step + j + 1);
					uchar p7 = (i == height - 1 || j == 0) ? 0 : *(p + dst.step + j - 1);
					if ((p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) >= 2 && (p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) <= 6)
					{
						int ap = 0;
						if (p2 == 0 && p3 == 1) ++ap;
						if (p3 == 0 && p4 == 1) ++ap;
						if (p4 == 0 && p5 == 1) ++ap;
						if (p5 == 0 && p6 == 1) ++ap;
						if (p6 == 0 && p7 == 1) ++ap;
						if (p7 == 0 && p8 == 1) ++ap;
						if (p8 == 0 && p9 == 1) ++ap;
						if (p9 == 0 && p2 == 1) ++ap;

						if (ap == 1 && p2 * p4 * p6 == 0 && p4 * p6 * p8 == 0)
						{
							//标记    
							mFlag.push_back(p + j);
						}
					}
				}
			}

			//将标记的点删除    
			for (std::vector<uchar *>::iterator i = mFlag.begin(); i != mFlag.end(); ++i)
			{
				**i = 0;
			}

			//直到没有点满足，算法结束    
			if (mFlag.empty())
			{
				break;
			}
			else
			{
				mFlag.clear();//将mFlag清空    
			}

			//对点标记    
			for (int i = 0; i < height; ++i)
			{
				uchar * p = dst.ptr<uchar>(i);
				for (int j = 0; j < width; ++j)
				{
					//如果满足四个条件，进行标记    
					//  p9 p2 p3    
					//  p8 p1 p4    
					//  p7 p6 p5    
					uchar p1 = p[j];
					if (p1 != 1) continue;
					uchar p4 = (j == width - 1) ? 0 : *(p + j + 1);
					uchar p8 = (j == 0) ? 0 : *(p + j - 1);
					uchar p2 = (i == 0) ? 0 : *(p - dst.step + j);
					uchar p3 = (i == 0 || j == width - 1) ? 0 : *(p - dst.step + j + 1);
					uchar p9 = (i == 0 || j == 0) ? 0 : *(p - dst.step + j - 1);
					uchar p6 = (i == height - 1) ? 0 : *(p + dst.step + j);
					uchar p5 = (i == height - 1 || j == width - 1) ? 0 : *(p + dst.step + j + 1);
					uchar p7 = (i == height - 1 || j == 0) ? 0 : *(p + dst.step + j - 1);

					if ((p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) >= 2 && (p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) <= 6)
					{
						int ap = 0;
						if (p2 == 0 && p3 == 1) ++ap;
						if (p3 == 0 && p4 == 1) ++ap;
						if (p4 == 0 && p5 == 1) ++ap;
						if (p5 == 0 && p6 == 1) ++ap;
						if (p6 == 0 && p7 == 1) ++ap;
						if (p7 == 0 && p8 == 1) ++ap;
						if (p8 == 0 && p9 == 1) ++ap;
						if (p9 == 0 && p2 == 1) ++ap;

						if (ap == 1 && p2 * p4 * p8 == 0 && p2 * p6 * p8 == 0)
						{
							//标记    
							mFlag.push_back(p + j);
						}
					}
				}
			}

			//将标记的点删除    
			for (std::vector<uchar *>::iterator i = mFlag.begin(); i != mFlag.end(); ++i)
			{
				**i = 0;
			}

			//直到没有点满足，算法结束    
			if (mFlag.empty())
			{
				break;
			}
			else
			{
				mFlag.clear();//将mFlag清空    
			}
		}
		return dst;
	}

}

