void remove_plane_param(int& remove_num, int& MI, float& dist, float& pr)
{
  ros::NodeHandle pn("~");
  pn.getParam("remove_plane", remove_num);
  pn.getParam("MaxIter", MI);
  pn.getParam("Dist", dist);
  pn.getParam("Prob", pr);
}

void min_cut_param(float& sigma, float& radius, int& nn, float& sw)
{
  ros::NodeHandle pn("~");
  pn.getParam("Sigma", sigma);
  pn.getParam("Radius", radius);
  pn.getParam("NoN", nn);
  pn.getParam("SourceWeight", sw);
}
