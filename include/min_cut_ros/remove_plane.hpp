//平面処理用の関数
void Remove_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr points, int counter, int MI, float DisTh, float Pr)
{
  //引数により設定した回数だけ平面除去を実行
  for (int i = 0; i < counter; i++)
    {
      ROS_INFO("Starting %d time removing plane", i+1);
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      seg.setOptimizeCoefficients(true);
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
      //平面除去前の準備(平面除去に必要な各パラメータの設定)
      seg.setInputCloud(points);
      seg.setModelType(pcl::SACMODEL_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setMaxIterations(MI);
      seg.setDistanceThreshold(DisTh);
      seg.setProbability(Pr);
      seg.segment(*inliers, *coefficients);
      //平面除去の処理の実行
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud(points);
      extract.setIndices(inliers);
      extract.setNegative(true);
      extract.filter(*points);

      ROS_INFO("Finished %d time removing plane", i+1);
      }
}
