//
// Created by chli on 09.04.20.
//

//
// Created by chli on 09.04.20.

#include "surface.h"

std::vector<float> road_feature (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    const clock_t  begin_time = clock();
    Eigen::Matrix3f  mat;
    Eigen::Vector3f value;
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cloud);
    mat = pca.getEigenVectors();
    value = pca.getEigenValues();
    //std::cout << "mat"<< mat << std::endl;
    //std::cout << "value" << value << std::endl;
    //std::cout << "time2" << float( clock () - begin_time ) /  CLOCKS_PER_SEC << std::endl;

    std::vector<float> vect;

    float slope = mat(0,2)/sqrt(pow(mat(0,0),2) + pow(mat(0,1),2))*100;
    float superelevation = mat(1,2) /sqrt(pow(mat(1,0),2) + pow(mat(1,1),2))*100;
    //std::cout << typeid(slope).name() << std::endl;
    vect.push_back(slope);
    vect.push_back(superelevation);

    //std::cout << "slope"<< slope << std::endl;
    //std::cout << "superelevation" << superelevation << std::endl;
    return vect;

}





void greedy_triangulation(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals,
        pcl::PolygonMesh::Ptr triangles)
{
    const clock_t  begin_time = clock();
    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);
    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;



    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (500);

    // Set typical values for the parameters
    gp3.setMu (100);
    gp3.setMaximumNearestNeighbors (1570);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (*triangles);
    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();
    std::cout << "time2  " << float( clock () - begin_time ) /  CLOCKS_PER_SEC << std::endl;
    // 显示结果图
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0); //设置背景
    viewer->addPolygonMesh(*triangles,"my"); //设置显示的网格
    //viewer->addCoordinateSystem (1.0); //设置坐标系
    viewer->initCameraParameters ();
    while (!viewer->wasStopped ()){
        viewer->spinOnce (100);
        //boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }


}

/*
三维重构之移动立方体算法
*/
void marchingcubes (pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals)
{

    //创建搜索树
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);

    //初始化 移动立方体算法 MarchingCubes对象，并设置参数
    pcl::MarchingCubes<pcl::PointNormal> *mc;
    mc = new pcl::MarchingCubesHoppe<pcl::PointNormal> ();
    /*
  if (hoppe_or_rbf == 0)
    mc = new pcl::MarchingCubesHoppe<pcl::PointNormal> ();
  else
  {
    mc = new pcl::MarchingCubesRBF<pcl::PointNormal> ();
    (reinterpret_cast<pcl::MarchingCubesRBF<pcl::PointNormal>*> (mc))->setOffSurfaceDisplacement (off_surface_displacement);
  }
    */

    //创建多变形网格，用于存储结果
    pcl::PolygonMesh mesh;

    //设置MarchingCubes对象的参数
    mc->setIsoLevel (0.0f);
    mc->setGridResolution (100, 100, 10);
    mc->setPercentageExtendGrid (0.0f);

    //设置搜索方法
    mc->setInputCloud (cloud_with_normals);

    //执行重构，结果保存在mesh中
    mc->reconstruct (mesh);

    //保存网格图
    pcl::io::savePLYFile("/mrtstorage/users/students/chli/real_data/test_data2/result2.ply", mesh);

    // 显示结果图
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0); //设置背景
    viewer->addPolygonMesh(mesh,"my"); //设置显示的网格
    //viewer->addCoordinateSystem (1.0); //设置坐标系
    viewer->initCameraParameters ();
    while (!viewer->wasStopped ()){
        viewer->spinOnce (100);
        //boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

}

void
PointCloud2Vector3d (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data)
{
    for (unsigned i = 0; i < cloud->size (); i++)
    {
        pcl::PointXYZ &p = cloud->at (i);
        if (!std::isnan (p.x) && !std::isnan (p.y) && !std::isnan (p.z))
            data.push_back (Eigen::Vector3d (p.x, p.y, p.z));
    }
}

void
visualizeCurve (ON_NurbsCurve &curve, ON_NurbsSurface &surface, pcl::visualization::PCLVisualizer &viewer)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr curve_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::on_nurbs::Triangulation::convertCurve2PointCloud (curve, surface, curve_cloud, 4);
    for (std::size_t i = 0; i < curve_cloud->size () - 1; i++)
    {
        pcl::PointXYZRGB &p1 = curve_cloud->at (i);
        pcl::PointXYZRGB &p2 = curve_cloud->at (i + 1);
        std::ostringstream os;
        os << "line" << i;
        viewer.removeShape (os.str ());
        viewer.addLine<pcl::PointXYZRGB> (p1, p2, 1.0, 0.0, 0.0, os.str ());
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr curve_cps (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int i = 0; i < curve.CVCount (); i++)
    {
        ON_3dPoint p1;
        curve.GetCV (i, p1);

        double pnt[3];
        surface.Evaluate (p1.x, p1.y, 0, 3, pnt);
        pcl::PointXYZRGB p2;
        p2.x = float (pnt[0]);
        p2.y = float (pnt[1]);
        p2.z = float (pnt[2]);

        p2.r = 255;
        p2.g = 0;
        p2.b = 0;

        curve_cps->push_back (p2);
    }
    viewer.removePointCloud ("cloud_cps");
    viewer.addPointCloud (curve_cps, "cloud_cps");
}

void bspline_fitting(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    const clock_t  begin_time = clock();
    pcl::visualization::PCLVisualizer viewer ("B-spline surface fitting");
    pcl::on_nurbs::NurbsDataSurface data;
    PointCloud2Vector3d (cloud, data.interior);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler (cloud, 0, 255, 0);
    viewer.addPointCloud<pcl::PointXYZ> (cloud, handler, "cloud_cylinder");
    printf ("  %lu points in data set\n", cloud->size ());

    // ############################################################################
    // fit B-spline surface

    // parameters

    unsigned order (2);
    unsigned refinement (2);
    //unsigned iterations (10);
    unsigned mesh_resolution (25);

    pcl::on_nurbs::FittingSurface::Parameter params;
    params.interior_smoothness = 0.2;
    params.interior_weight = 1.0;
    params.boundary_smoothness = 0.2;
    params.boundary_weight = 0.0;

    // initialize
    printf ("  surface fitting ...\n");
    ON_NurbsSurface nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox (order, &data);
    pcl::on_nurbs::FittingSurface fit (&data, nurbs);
    //  fit.setQuiet (false); // enable/disable debug output

    // mesh for visualization
    pcl::PolygonMesh mesh;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::Vertices> mesh_vertices;
    std::string mesh_id = "mesh_nurbs";
    pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh (fit.m_nurbs, mesh, mesh_resolution);
    viewer.addPolygonMesh (mesh, mesh_id);

    // surface refinement
    for (unsigned i = 0; i < refinement; i++)
    {
        fit.refine (0);
        fit.refine (1);
        fit.assemble (params);
        fit.solve ();
        pcl::on_nurbs::Triangulation::convertSurface2Vertices (fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
        viewer.updatePolygonMesh<pcl::PointXYZ> (mesh_cloud, mesh_vertices, mesh_id);
        viewer.spinOnce ();
    }

    std::cout << "time" << float( clock () - begin_time ) /  CLOCKS_PER_SEC << std::endl;

    /*
    // surface fitting with final refinement level
    for (unsigned i = 0; i < iterations; i++)
    {
        fit.assemble (params);
        fit.solve ();
        pcl::on_nurbs::Triangulation::convertSurface2Vertices (fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
        viewer.updatePolygonMesh<pcl::PointXYZ> (mesh_cloud, mesh_vertices, mesh_id);
        viewer.spinOnce ();
    }

    // ############################################################################
    // fit B-spline curve

    // parameters
    pcl::on_nurbs::FittingCurve2dAPDM::FitParameter curve_params;
    curve_params.addCPsAccuracy = 5e-2;
    curve_params.addCPsIteration = 3;
    curve_params.maxCPs = 200;
    curve_params.accuracy = 1e-3;
    curve_params.iterations = 100;

    curve_params.param.closest_point_resolution = 0;
    curve_params.param.closest_point_weight = 1.0;
    curve_params.param.closest_point_sigma2 = 0.1;
    curve_params.param.interior_sigma2 = 0.00001;
    curve_params.param.smooth_concavity = 1.0;
    curve_params.param.smoothness = 1.0;

    // initialisation (circular)
    printf ("  curve fitting ...\n");
    pcl::on_nurbs::NurbsDataCurve2d curve_data;
    curve_data.interior = data.interior_param;
    curve_data.interior_weight_function.push_back (true);
    ON_NurbsCurve curve_nurbs = pcl::on_nurbs::FittingCurve2dAPDM::initNurbsCurve2D (order, curve_data.interior);

    // curve fitting
    pcl::on_nurbs::FittingCurve2dASDM curve_fit (&curve_data, curve_nurbs);
    // curve_fit.setQuiet (false); // enable/disable debug output
    curve_fit.fitting (curve_params);
    visualizeCurve (curve_fit.m_nurbs, fit.m_nurbs, viewer);

    // ############################################################################
    // triangulation of trimmed surface

    printf ("  triangulate trimmed surface ...\n");
    viewer.removePolygonMesh (mesh_id);
    pcl::on_nurbs::Triangulation::convertTrimmedSurface2PolygonMesh (fit.m_nurbs, curve_fit.m_nurbs, mesh,
                                                                     mesh_resolution);
    viewer.addPolygonMesh (mesh, mesh_id);


    // save trimmed B-spline surface
    if ( fit.m_nurbs.IsValid() )
    {
        ONX_Model model;
        ONX_Model_Object& surf = model.m_object_table.AppendNew();
        surf.m_object = new ON_NurbsSurface(fit.m_nurbs);
        surf.m_bDeleteObject = true;
        surf.m_attributes.m_layer_index = 1;
        surf.m_attributes.m_name = "surface";

        ONX_Model_Object& curv = model.m_object_table.AppendNew();
        curv.m_object = new ON_NurbsCurve(curve_fit.m_nurbs);
        curv.m_bDeleteObject = true;
        curv.m_attributes.m_layer_index = 2;
        curv.m_attributes.m_name = "trimming curve";
        //model.Write(file_3dm.c_str());
        //printf("  model saved: %s\n", file_3dm.c_str());
    }
     */
    printf ("  ... done.\n");

    viewer.spin ();


}