#ifndef  PCL_FILETERS_BILATERAL_H_
#define PCL_FILETERS_BILATERAL_H_

#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree.h>

namespace pcl
{
	template<typename PointT>
	class BilateralFilter :public Filter<PointT>
	{
		using Filer<PointT>::input_;
		typedef typename Filter<PointT>::PonitCloud PointCloud;
		typedef typename pcl::KdTree<PointT>::ptr KdTreePtr;
	public:
		
		BilateralFilter() :sigma_s_(0), sigma_r_(std::numeric_limits<double>::max())
		{
			;
		}
		void
			applyFilter(PointCloud &output);
		double
			computerPointWeight(const int pid,const std::vector<int>&indices,const std::vector<float>&distances);

		void
			setSigma(const double sigma_s)
		{
			sigma_s = sigma_s;
		}
		
		double
			getSigma()
		{
			return(sigma_s_);
		}
		
		void
			setSigmaR(const double sigma_r)
		{
			sigma_r_ = sigma_r;
		}
		
		double
			getSigmaR()
		{
			return(sigma_r_);
		}
		void
			setSearchMethod(const KdTreePtr &tree)
		{
			tree_ = tree;
		}
	private:
		inline double
			kernel(double x, double sigma)
		{
			return (exp(-(x*x) / (2 * sigma*sigma)));
		}
		double sigma_s_;
		double sigma_r_;
		KdTreePtr tree_;
	};
}
#endif // ! PCL_FILETERS_BILATERAL_H_
