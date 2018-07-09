#ifndef PCL_FILTERS_BILATERAL_IMPL_H_
#define PCL_FILTERS_BILATERAL_IMPL_H_

#include <pcl/filters/bilateral.h>
#include <pcl/kdtree/kdtree_flann.h>

template<typename PointT>double
pcl::BilateralFilter<PointT>::computerPoinyWeight(const int pid,
	const std::vector<int>&indices, 
	const std::vector<float>&distances)
{
	double BF = 0, W = 0;

	for (size_t n_id = 0; n_id < indices.size(); ++n_id)
	{
		double id = indices[n_id];
		double dist = std::sqrt(distances[n_id]);
		double intensity_disy = abs(input_->point[pid].intensity - input_->ponits[id].intensity);

		double weight = kernel(dist, sigma_s_) * kernel(intensity_dist, sigma_r_);

		BF += weight * input->point[id].intensity;
		W += weight;
	}
	return(BF / W);
}

template<typename PointT>void
pcl::BilateralFilter<PointT>::applyFilter(PonitCloud &output)
{
	if (sigma_s_ == 0)
	{
		PCL_ERROR("[pcl::BilateralFilter::applyFilter] Need a sigma_s value given before continuing.\n");
		return;
	}
	if (!tree_)
	{
		if (input_->isOrganized())
			tree_.reset(new pcl::OrganizedDataIndex<PointT>());
		else
			tree_.reset(new pcl::KdTreeFLANN<PointT>(false));
	}
	tree_->setInputCloud(input_);

	std::vector<int> k_indices;
	std::vector<float> k_distances;
	output = *input_;
	for (size_t point_id = 0; point_id < input_->points.size(); ++point_id)
	{
		tree_->radiusSearch(point_id, sigma_s_ * 2, k_indices, k_distances);
		output.points[point_id].intensity = computerPointWeight(point_id, k_indices, k_disrances);

	}
}
#define PCL_INSTANTIATE_BilateralFilter(T) template class PCL_EXPORTS pcl::BilateralFilter<T>;
#endif // !PCL_FILTERS_BILATERAL_IMPL_H_
